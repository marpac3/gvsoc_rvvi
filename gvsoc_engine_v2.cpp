/*
 * Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and
 *                    University of Bologna
 * Copyright (C) 2026 Fondazione Chips-it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Authors: Marco Paci, Fondazione Chips-it (marco.paci@chips.it)
 */

/*
 * GVSOC engine wrapper - iss_v2 implementation.
 *
 * Same role and public API (gvsoc_engine.hpp) as gvsoc_engine.cpp, targeting
 * the statically-typed iss_v2 core model (cv32e40p-v2-standalone platforms)
 * instead of the v1 ISS. Differences that matter here:
 *  - the core component is the Iss class itself (no IssWrapper);
 *  - the module layout is fixed by the generated ISA header, which the
 *    Makefile force-includes (-include) so this file sees the exact same
 *    personality types (Cv32e40pCsr, Cv32e40pExec, Cv32e40pEvents) the
 *    model .so was built with;
 *  - the register file is a unified private array, accessed through the
 *    inline accessors (get_reg_untimed/set_reg/set_freg + get_reg_gid);
 *  - retires are consumed from the personality's commit stream
 *    (Cv32e40pEvents::commit_pc ring): a PC is pushed only once the
 *    instruction's writeback is architecturally visible, so sampling the
 *    regfile right after a pop cannot race an in-flight async load.
 *
 * ABI contract: this file MUST be compiled with the same defines and the
 * same generated ISA header used to build the ISS model .so, otherwise ISS
 * struct layouts mismatch and dereferencing them SIGSEGVs at runtime.
 *
 * Access contract: ISS state is read/written ONLY through public struct
 * members and header-inline accessors, never through methods compiled into
 * the model .so (not available at DPI link time).
 */

#include "gvsoc_engine.hpp"

#include <gv/gvsoc.hpp>
#include <cpu/iss_v2/include/iss.hpp>

/* The personality macros come from the generated ISA header the Makefile
 * force-includes; without it iss.hpp resolves to no concrete types at all. */
#if !defined(CONFIG_GVSOC_ISS_V2)
#error "gvsoc_engine_v2.cpp requires CONFIG_GVSOC_ISS_V2=1 (see Makefile ISS_DEFINES_V2)"
#endif
#if !defined(CONFIG_GVSOC_ISS_CSR)
#error "gvsoc_engine_v2.cpp must be built with the generated ISA header force-included (see Makefile ISA_HDR_V2)"
#endif

/* First line of defense on the ABI contract: if the defines or the ISA
 * header drift from what the model .so was built with, the register-file
 * size is the first thing to move (CONFIG_GVSOC_ISS_ZFINX alone shrinks
 * it by 256 bytes). Checked on the personality type actually instantiated
 * in Iss (Cv32e40pRegfile adds its write-back suppression flag on top of
 * the base Regfile). The runtime canary in engine_acquire_core() catches
 * what a same-size skew would still hide. */
#ifdef CONFIG_GVSOC_ISS_ZFINX
static_assert(sizeof(CONFIG_GVSOC_ISS_REGFILE) == 608,
              "Regfile layout drifted from the iss_v2 build contract (see Makefile)");
#else
static_assert(sizeof(CONFIG_GVSOC_ISS_REGFILE) == 864,
              "Regfile layout drifted from the iss_v2 build contract (see Makefile)");
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <unordered_map>
#include <vpi_user.h>   /* vpi_printf - prints to simulator transcript */

/* Log macros use vpi_printf: it flushes to the simulator transcript
 * immediately, whereas fprintf(stderr) is not flushed in DPI context. */
#define ENGINE_LOG(fmt, ...) \
    do { vpi_printf((char *)"[gvsoc-engine] " fmt "\n", ##__VA_ARGS__); } while(0)
#define ENGINE_ERR(fmt, ...) \
    do { vpi_printf((char *)"[gvsoc-engine] ERROR: " fmt "\n", ##__VA_ARGS__); } while(0)

/* --------------------------------------------------------------------------
 * GVSOC engine global state
 * ---------------------------------------------------------------------- */

/* Global because gvsoc_new(&g_conf) keeps the pointer for the engine's
 * lifetime: the configuration must outlive the call. */
static gv::GvsocConf  g_conf;
static gv::Gvsoc     *g_gvsoc      = nullptr;
static bool           g_running     = false;
static bool           g_finished    = false;
/* Clock period of the cv32e40p-v2-standalone platform (50 MHz), a constant
 * of the model: it must match the platform definition, not a runtime knob. */
static constexpr int64_t g_clock_ps = 20000;

static Iss           *g_iss         = nullptr;

/* CSR address -> pointer to the iss_reg_t value (public struct member).
 * Populated once after gvsoc_engine_init() succeeds. */
static std::unordered_map<uint32_t, iss_reg_t *> g_csr_value_map;

/* Stepping state */
static uint64_t      g_retire_count  = 0;
static iss_reg_t     g_retired_pc    = 0;  /* PC of last retired instruction */
static uint32_t      g_retired_opcode = 0; /* always 0: opcode not captured in DPI mode */
static uint64_t      g_popped_trap_seq = 0; /* trap_seq stamp of the last popped commit */
static FILE         *g_iss_trace_fp  = nullptr;  /* opt-in, env GVSOC_RVVI_ISS_TRACE */

/* When set, re-assert exec.skip_irq_check before every step(): the ISS
 * slow handler clears it after one use. */
static bool g_dpi_skip_irq = false;

/* --------------------------------------------------------------------------
 * Runaway detector (DPI co-sim only)
 *
 * If the ISS takes its own illegal-instruction trap after an IRQ-timing
 * desync, it zeroes mstatus.MIE and the bridge's MIE-based resync trigger
 * stops firing. The ISS then free-runs at a fixed PC: every step() burns the
 * full STEP_MAX_CYCLES budget with no retire, sim-time crawls, and the OS
 * timeout eventually reaps vsimk at a non-deterministic point.
 *
 * We count consecutive budget-exhausting timeouts with the ISS PC unchanged;
 * at RUNAWAY_THRESHOLD we latch g_runaway (sticky) so the bridge can report a
 * clean FAIL instead of a hang. A clean retire resets the counter, so a
 * passing test never trips it; WFI stalls and the exit-device path are
 * excluded in the step timeout path below. */
static constexpr int RUNAWAY_THRESHOLD = 16;
static int           g_runaway_count = 0;       /* consecutive stuck-PC timeouts */
static iss_reg_t     g_runaway_last_pc = 0;     /* ISS PC at the previous timeout */
static bool          g_runaway_last_valid = false;
static bool          g_runaway = false;         /* sticky once latched */

/* Set when mip bits are asserted, cleared after settle */
static bool          g_irq_pending_settle = false;

/* Interrupt wire bindings on the platform irq injector, indexed by RVVI net:
 * 0=MSWInterrupt->msi, 1=MTimerInterrupt->mti, 2=MExternalInterrupt->mei,
 * 3..18=LocalInterrupt0..15->external_irq_16..31. haltreq (net 19) is a
 * debug request, not an interrupt wire (see gvsoc_engine_set_irq). */
static constexpr int  IRQ_NB_WIRES = 19;
static gv::Wire_binding *g_irq_wire[IRQ_NB_WIRES] = {};
/* Callback sink required by wire_bind; the ISS never drives these wires
 * back, so it stays empty. */
static gv::Wire_user  g_irq_wire_user;

static const char *const g_irq_wire_name[IRQ_NB_WIRES] = {
    "msi", "mti", "mei",
    "external_irq_16", "external_irq_17", "external_irq_18", "external_irq_19",
    "external_irq_20", "external_irq_21", "external_irq_22", "external_irq_23",
    "external_irq_24", "external_irq_25", "external_irq_26", "external_irq_27",
    "external_irq_28", "external_irq_29", "external_irq_30", "external_irq_31",
};

/* --------------------------------------------------------------------------
 * Gvsoc_user callback to detect simulation end
 * ---------------------------------------------------------------------- */

class BridgeUser : public gv::Gvsoc_user
{
public:
    void has_ended(int status) override
    {
        g_finished = true;
        g_running  = false;  /* mark engine dead so stepping stubs out immediately */
        ENGINE_LOG("simulation ended (status=%d)", status);
    }

    void has_stopped() override
    {
        /* Nothing - we drive stepping manually */
    }
};

static BridgeUser g_user;

/* --------------------------------------------------------------------------
 * Build CSR address -> value pointer map from public struct members.
 * Avoids calling CsrAbtractReg::access() (compiled into the model .so, not
 * available at DPI link time). Only CSRs with named members and a public
 * .value field are mapped. The argument is the concrete personality class:
 * it REPLACES several base registers (mvendorid/marchid/mhartid RO views,
 * minstret) and owns the CV32E40P-only ones (tinfo, mhpmevent).
 * ---------------------------------------------------------------------- */

static void build_csr_map(Cv32e40pCsr &csr)
{
    g_csr_value_map.clear();

    /* Machine-level CSRs */
    g_csr_value_map[0x300] = &csr.mstatus.value;   /* mstatus */
    g_csr_value_map[0x301] = &csr.misa.value;       /* misa */
    g_csr_value_map[0x302] = &csr.medeleg.value;    /* medeleg */
    g_csr_value_map[0x303] = &csr.mideleg.value;    /* mideleg */
    g_csr_value_map[0x304] = &csr.mie.value;        /* mie */
    g_csr_value_map[0x305] = &csr.mtvec.value;      /* mtvec */
    g_csr_value_map[0x306] = &csr.mcounteren.value;  /* mcounteren */

    g_csr_value_map[0x340] = &csr.mscratch.value;   /* mscratch */
    g_csr_value_map[0x341] = &csr.mepc.value;       /* mepc */
    g_csr_value_map[0x342] = &csr.mcause.value;     /* mcause */
    g_csr_value_map[0x343] = &csr.mtval.value;      /* mtval */
    g_csr_value_map[0x344] = &csr.mip.value;        /* mip */

    /* Trigger CSRs */
    g_csr_value_map[0x7A0] = &csr.tselect.value;    /* tselect */
    g_csr_value_map[0x7A1] = &csr.tdata1.value;     /* tdata1 */
    g_csr_value_map[0x7A2] = &csr.tdata2.value;     /* tdata2 */
    g_csr_value_map[0x7A3] = &csr.tdata3.value;     /* tdata3 */
    g_csr_value_map[0x7A4] = &csr.tinfo.value;      /* tinfo */
    g_csr_value_map[0x7A8] = &csr.mcontext.value;   /* mcontext */
    g_csr_value_map[0x7AA] = &csr.scontext.value;   /* scontext */

    /* Debug CSRs - dcsr/depc are raw iss_reg_t, not CsrReg */
    g_csr_value_map[0x7B0] = &csr.dcsr;             /* dcsr */
    g_csr_value_map[0x7B1] = &csr.depc;             /* dpc */

    /* FPU CSR - fcsr.raw maps the full register (fflags+frm) */
    g_csr_value_map[0x003] = &csr.fcsr.raw;         /* fcsr */

    /* Vendor/implementation CSRs - the personality RO views, which carry
     * the CV32E40P reset values (the base mvendorid/marchid stay 0). */
    g_csr_value_map[0xF11] = &csr.mvendorid_ro.value;  /* mvendorid */
    g_csr_value_map[0xF12] = &csr.marchid_ro.value;    /* marchid */
    g_csr_value_map[0xF13] = &csr.mimpid.value;        /* mimpid */
    g_csr_value_map[0xF14] = &csr.mhartid_csr.value;   /* mhartid */

    /* Counter CSRs. With counting enabled mcycle.value is the personality's
     * frozen/offset anchor, not the live count - the co-sim compares only
     * mcountinhibit among these, so the anchor is what a resync needs. */
    g_csr_value_map[0xB00] = &csr.mcycle.value;     /* mcycle */
    g_csr_value_map[0xB80] = &csr.mcycleh.value;    /* mcycleh */
    g_csr_value_map[0xB02] = &csr.minstret.value;   /* minstret */
    g_csr_value_map[0xB82] = &csr.minstreth.value;  /* minstreth */
    g_csr_value_map[0x320] = &csr.mcountinhibit.value; /* mcountinhibit */

    /* NMI CSRs */
    g_csr_value_map[0x740] = &csr.mnscratch.value;  /* mnscratch */
    g_csr_value_map[0x741] = &csr.mnepc.value;      /* mnepc */
    g_csr_value_map[0x742] = &csr.mncause.value;    /* mncause */
    g_csr_value_map[0x744] = &csr.mnstatus.value;   /* mnstatus */

    /* Supervisor and vector CSRs are deliberately NOT mapped: CV32E40P has
     * neither, nothing reads them through this bridge, and every mapped
     * pointer is reach-in ABI surface to keep at zero benefit. */

    /* mhpmcounter3..31 (addresses 0xB03..0xB1F) */
    for (int i = 0; i < 29; i++)
    {
        g_csr_value_map[0xB03 + i] = &csr.mhpmcounter[i].value;
    }

#if ISS_REG_WIDTH == 32
    /* mhpmcounter3h..31h (addresses 0xB83..0xB9F) */
    for (int i = 0; i < 29; i++)
    {
        g_csr_value_map[0xB83 + i] = &csr.mhpmcounterh[i].value;
    }
#endif

    /* mhpmevent3..31 (addresses 0x323..0x33F) */
    for (int i = 0; i < 29; i++)
    {
        g_csr_value_map[0x323 + i] = &csr.mhpmevent[i].value;
    }

    ENGINE_LOG("CSR map built: %zu entries", g_csr_value_map.size());
}

/* --------------------------------------------------------------------------
 * Lifecycle - sub-functions for gvsoc_engine_init()
 * ---------------------------------------------------------------------- */

/* Invoke a GVSOC API call, logging any exception without propagating.
 * The checked variant reports whether the call completed. */
template<typename Fn>
static bool engine_call_checked(const char *name, Fn fn)
{
    try { fn(); return true; }
    catch (const std::exception &e) { ENGINE_ERR("%s threw: %s", name, e.what()); }
    catch (...)                      { ENGINE_ERR("%s threw unknown exception", name); }
    return false;
}

template<typename Fn>
static void engine_call_safe(const char *name, Fn fn)
{
    (void)engine_call_checked(name, fn);
}

/* Create GVSOC engine instance and bind callbacks.
 * Returns 0 on success, -1 on failure. */
static int engine_create(const char *config_path)
{
    g_conf.config_path = config_path;
    g_conf.api_mode    = gv::Api_mode_sync;

    if (!engine_call_checked("gvsoc_new", [](){ g_gvsoc = gv::gvsoc_new(&g_conf); }))
        return -1;

    if (!g_gvsoc)
    {
        ENGINE_ERR("gvsoc_new() returned null");
        return -1;
    }

    if (!engine_call_checked("bind", [](){ g_gvsoc->bind(&g_user); }))
        return -1;
    return 0;
}

/* Open and start the engine. Cleans up on failure.
 * Returns 0 on success, -1 on failure. */
static int engine_open_and_start(void)
{
    if (!engine_call_checked("open",  [](){ g_gvsoc->open();  }) ||
        !engine_call_checked("start", [](){ g_gvsoc->start(); }))
    {
        engine_call_safe("close", [](){ g_gvsoc->close(); });
        g_gvsoc = nullptr;
        return -1;
    }

    ENGINE_LOG("start() done");
    return 0;
}

/* Layout canary: read back, through the reach-in pointer map, values that are
 * known by construction right after reset. If the model .so was built with
 * different structure-affecting flags than this file, these pointers land on
 * the wrong fields and the mismatch surfaces here as a readable init error
 * instead of a silent divergence (or a SIGSEGV) later.
 * Returns 0 on success, -1 on mismatch. */
static int engine_layout_canary(void)
{
    static const struct { uint32_t addr; uint32_t expect; const char *name; } checks[] = {
        { 0xF11, 0x00000602, "mvendorid" },  /* OpenHW JEDEC, config reset value */
        { 0xF12, 0x00000004, "marchid"   },  /* CV32E40P architecture id */
        { 0x7B0, 0x40000003, "dcsr"      },  /* xdebugver=4, prv=M, Cv32e40pCsr reset */
        { 0x300, 0x00001800, "mstatus"   },  /* forced above: checks the write+read round trip */
    };
    for (const auto &c : checks)
    {
        uint32_t got = 0;
        if (!gvsoc_engine_get_csr(c.addr, &got) || got != c.expect)
        {
            ENGINE_ERR("layout canary FAILED on %s (0x%03x): read 0x%08x, expected 0x%08x - "
                       "the bridge and the ISS model .so disagree on struct layout "
                       "(ISS_DEFINES_V2 / ISA header drift? see Makefile ABI contract)",
                       c.name, c.addr, got, c.expect);
            return -1;
        }
    }
    ENGINE_LOG("layout canary OK (mvendorid/marchid/dcsr/mstatus)");
    return 0;
}

/* Acquire the ISS core component and build CSR map.
 * Returns 0 on success, -1 on failure. */
static int engine_acquire_core(void)
{
    void *comp = nullptr;
    if (!engine_call_checked("get_component",
                             [&](){ comp = g_gvsoc->get_component("soc/core"); }))
        return -1;

    if (!comp)
    {
        ENGINE_ERR("get_component(\"soc/core\") returned null");
        return -1;
    }

    g_iss = static_cast<Iss *>(comp);
    ENGINE_LOG("core component found at %p", (void *)g_iss);

    build_csr_map(g_iss->csr);

    /* depc is a raw iss_reg_t (not a CsrReg) and is NOT initialized by
     * Csr::reset(); it retains allocation garbage, which would mismatch the
     * DUT (0x0) at the first dpc comparison. dcsr is already reset to
     * 0x40000003 by the personality (RTL reset value). Force depc=0. */
    g_iss->csr.depc = 0;
    ENGINE_LOG("forced csr.depc=0x0 (RTL reset value)");

    /* Defense-in-depth: force RTL reset values into every trap/NMI/debug CSR
     * the bridge maps, before the first step(). Csr::reset() should already
     * zero these, but a reset-order/reset-scope hole in the v1 ISS has been
     * seen leaving stale content at the first trap comparison. Idempotent,
     * runs once. CV32E40P RTL reset: all trap CSRs reset to 0 except
     * mstatus, which resets to 0x1800 (FS=Off, MPP=M). */
    g_iss->csr.mepc.value      = 0;
    g_iss->csr.mcause.value    = 0;
    g_iss->csr.mtval.value     = 0;
    g_iss->csr.mscratch.value  = 0;
    g_iss->csr.mstatus.value   = 0x00001800;  /* FS=Off, MPP=M */
    g_iss->csr.mtvec.value     = 0;
    g_iss->csr.mie.value       = 0;
    g_iss->csr.mip.value       = 0;
    g_iss->csr.mnscratch.value = 0;
    g_iss->csr.mnepc.value     = 0;
    g_iss->csr.mncause.value   = 0;
    g_iss->csr.mnstatus.value  = 0;
    g_iss->csr.scratch0        = 0;
    g_iss->csr.scratch1        = 0;
    ENGINE_LOG("forced trap/NMI CSRs to RTL reset values "
               "(mstatus=0x1800, others=0)");

    /* RTL reset: both register files reset to 0 (flip-flops). The v2 model
     * poisons them with 0x57.. to catch use-before-init in standalone runs;
     * in a DUT-vs-ISS compare a propagated never-written register must read
     * the same on both sides. */
    for (int i = 1; i < 32; i++)
    {
        g_iss->regfile.set_reg(i, 0);
    }
    for (int i = 0; i < 32; i++)
    {
        g_iss->regfile.set_freg(g_iss->regfile.get_reg_gid(i), 0);
    }
    ENGINE_LOG("forced GPR/FPR files to RTL reset values (0)");

    ENGINE_LOG("initial PC=0x%08x",
               (unsigned)g_iss->exec.current_insn);

    return engine_layout_canary();
}

/* Bind the interrupt wires on the platform irq injector (official gv::
 * client API). Returns 0 on success, -1 on failure. */
static int engine_bind_irq_wires(void)
{
    for (int net = 0; net < IRQ_NB_WIRES; net++)
    {
        gv::Wire_binding *binding = nullptr;
        if (!engine_call_checked("wire_bind", [&](){
                binding = g_gvsoc->wire_bind(&g_irq_wire_user, "irq_injector",
                                             g_irq_wire_name[net]); }))
            return -1;

        if (!binding)
        {
            ENGINE_ERR("wire_bind(irq_injector, %s) returned null - "
                       "platform is missing the injector or the port",
                       g_irq_wire_name[net]);
            return -1;
        }
        g_irq_wire[net] = binding;
    }

    ENGINE_LOG("interrupt wires bound (%d lines on irq_injector)", IRQ_NB_WIRES);
    return 0;
}

/* --------------------------------------------------------------------------
 * Lifecycle - public API
 * ---------------------------------------------------------------------- */

int gvsoc_engine_init(const char *config_path)
{
    if (g_running)
    {
        ENGINE_LOG("re-init requested - shutting down previous instance");
        gvsoc_engine_shutdown();
    }

    if (!config_path || strlen(config_path) == 0)
    {
        ENGINE_ERR("config_path is empty");
        return -1;
    }

    ENGINE_LOG("initializing (config=%s)", config_path);

    if (engine_create(config_path) != 0)
        return -1;

    if (engine_open_and_start() != 0)
        return -1;

    if (engine_acquire_core() != 0 || engine_bind_irq_wires() != 0)
    {
        /* No graceful teardown here: stop() self-deadlocks in sync mode
         * (see the abnormal-shutdown path below). The simulator aborts on
         * the failed init anyway; the OS reclaims the engine. */
        g_iss   = nullptr;
        g_gvsoc = nullptr;
        return -1;
    }

    /* Commit-stream contract: the fast dispatch path skips the commit-FIFO
     * bookkeeping the stream is built on, so pin the core to the full
     * handlers for the whole co-simulation. */
    g_iss->exec.commit_stream_observed = true;

    /* Opt-in ISS-side PC trace for offline comparison. Off by default: an
     * always-on trace leaves an orphan file per run and costs one fprintf
     * per retire that nobody reads. */
    const char *trace_env = getenv("GVSOC_RVVI_ISS_TRACE");
    if (trace_env && trace_env[0] != '\0' && strcmp(trace_env, "0") != 0)
    {
        g_iss_trace_fp = fopen(trace_env, "w");
        if (g_iss_trace_fp)
            ENGINE_LOG("ISS trace -> %s", trace_env);
        else
            ENGINE_ERR("cannot open ISS trace file '%s'", trace_env);
    }

    g_retire_count  = 0;
    g_retired_pc    = 0;
    g_retired_opcode = 0;
    g_running       = true;
    g_finished   = false;

    /* Reset runaway detector state for this run. */
    g_runaway_count      = 0;
    g_runaway_last_pc    = 0;
    g_runaway_last_valid = false;
    g_runaway            = false;

    /* ISS-side IRQ checking is configured later via gvsoc_engine_skip_irq(),
     * once the DPI bridge knows whether the DUT drives interrupt flow. */

    ENGINE_LOG("initialized OK");
    return 0;
}

void gvsoc_engine_shutdown(void)
{
    if (!g_running)
        return;

    ENGINE_LOG("shutting down");

    g_csr_value_map.clear();

    /* The wire bindings belong to the engine instance being torn down. */
    for (int net = 0; net < IRQ_NB_WIRES; net++)
        g_irq_wire[net] = nullptr;

    if (g_iss_trace_fp) {
        fclose(g_iss_trace_fp);
        g_iss_trace_fp = nullptr;
    }

    if (g_gvsoc)
    {
        if (!g_finished)
        {
            /* Abnormal termination: the sim did not finish on its own (SV-side
             * abort - consecutive-mismatch watchdog, runaway, UVM timeout).
             * The graceful teardown MUST be skipped here:
             * - stop() self-deadlocks: in sync mode Controller::start() takes
             *   the engine mutex and the external loop (us) owns it until the
             *   internal sim-finished path releases it, which never ran; the
             *   stop() -> engine_lock() relock on the same non-recursive mutex
             *   hangs the simulator forever (observed: 35+ min "hangs" that
             *   were really this deadlock until an external kill).
             * - join() would RESUME the simulation: its sync branch loops on
             *   run_sync() until is_sim_finished.
             * The process is exiting anyway; the OS reclaims the engine. */
            ENGINE_LOG("abnormal shutdown (sim not finished) - skipping engine stop/join");
            g_gvsoc = nullptr;
        }
        else
        {
            engine_call_safe("stop",  [](){ g_gvsoc->stop();   });
            engine_call_safe("quit",  [](){ g_gvsoc->quit(0);  });
            engine_call_safe("join",  [](){ g_gvsoc->join();   });
            engine_call_safe("close", [](){ g_gvsoc->close();  });
            g_gvsoc = nullptr;
        }
    }

    g_iss      = nullptr;
    g_running  = false;
    g_finished = false;
}

bool gvsoc_engine_is_running(void)
{
    return g_running;
}

/* --------------------------------------------------------------------------
 * Stepping - instruction-accurate via the personality commit stream.
 *
 * The personality pushes a PC on its commit ring only once the writeback is
 * architecturally visible: sync instructions at retire, held ones (async
 * load, WFI) at commit-FIFO drain. One step() serves one commit: pop the
 * ring if it has a backlog, otherwise clock the engine until a commit
 * lands. A single clock can commit several instructions at once (a drained
 * load plus its followers); the backlog serves them one per call without
 * advancing the engine, so the sampled state never runs past the writeback
 * of the commit being reported by more than the same-clock burst.
 * ---------------------------------------------------------------------- */

static constexpr int STEP_MAX_CYCLES         = 2000; /* max stall cycles before timeout */

/* Commit one retire: record the retired PC, bump the counter, log the first
 * retires, feed the opt-in trace, reset the runaway detector. Shared by the
 * two retire paths of the step loop (PC-changed and branch-to-self). */
static void retire_commit(iss_reg_t retired_pc, const char *how, int cycles)
{
    g_retired_pc     = retired_pc;
    g_retired_opcode = 0;  /* no opcode captured in DPI mode */
    g_retire_count++;

    if (g_retire_count <= 25)
    {
        ENGINE_LOG("retire #%llu: PC 0x%08x %s (cycles=%d)",
                   (unsigned long long)g_retire_count,
                   (unsigned)retired_pc, how, cycles);
    }

    if (g_iss_trace_fp)
    {
        fprintf(g_iss_trace_fp, "0x%08x\n", (unsigned)retired_pc);
        if ((g_retire_count % 1000) == 0)
            fflush(g_iss_trace_fp);
    }

    /* Clean retire: the ISS is making forward progress, not a runaway. */
    g_runaway_count      = 0;
    g_runaway_last_valid = false;
}

/* Run the engine until at least one commit is queued in the ring.  Returns
 * true with the head left at commit_pop (NOT consumed); false on timeout or
 * finish.  Ring overflow marks the engine finished (commits were lost).
 * Callers wrap in try/catch: GVSOC step() may throw across the DPI boundary.
 * The trailing iteration serves a commit produced by the very last clock,
 * including the one that finished the simulation. */
static bool engine_advance_to_commit(int *cycles)
{
    for (int i = 0; i <= STEP_MAX_CYCLES; i++)
    {
        uint64_t backlog = g_iss->timing.commit_push - g_iss->timing.commit_pop;
        if (backlog > 0)
        {
            if (backlog > (uint64_t)Cv32e40pEvents::COMMIT_RING)
            {
                ENGINE_ERR("commit ring overflow (backlog=%llu) - "
                           "commits were lost, aborting",
                           (unsigned long long)backlog);
                g_finished = true;
                return false;
            }
            if (cycles)
                *cycles = i;
            return true;
        }

        if (g_finished || i == STEP_MAX_CYCLES)
            break;

        /* Re-assert skip_irq_check before every step.
         * The ISS slow handler clears it after one use. */
        if (g_dpi_skip_irq)
            g_iss->exec.skip_irq_check = true;

        g_gvsoc->step(g_clock_ps);
    }
    return false;
}

int gvsoc_engine_step(void)
{
    if (!g_running || !g_gvsoc || g_finished || !g_iss)
        return 0;

    try
    {
        int cycles = 0;
        if (engine_advance_to_commit(&cycles))
        {
            uint64_t idx = g_iss->timing.commit_pop % Cv32e40pEvents::COMMIT_RING;
            iss_reg_t pc = g_iss->timing.commit_pc[idx];
            g_popped_trap_seq = g_iss->timing.commit_trap_seq[idx];
            g_iss->timing.commit_pop++;
            retire_commit(pc, "committed", cycles);
            return 1;
        }
        if (g_finished)
            return 0;
    }
    catch (const std::exception &e)
    {
        ENGINE_ERR("step() threw: %s", e.what());
        g_finished = true;
        return 0;
    }
    catch (...)
    {
        ENGINE_ERR("step() threw unknown exception");
        g_finished = true;
        return 0;
    }

    /* Diagnostic for first timeouts */
    static uint64_t timeout_log_count = 0;
    timeout_log_count++;
    if (timeout_log_count <= 5)
    {
        ENGINE_ERR("step TIMEOUT: no retire after %d cycles "
                   "(retire #%llu, PC=0x%08x, retained=%lld, wfi=%d)",
                   STEP_MAX_CYCLES,
                   (unsigned long long)g_retire_count,
                   (unsigned)g_iss->exec.current_insn,
                   (long long)g_iss->exec.retained.get(),
                   (int)g_iss->exec.wfi.get());
        if (timeout_log_count == 5)
            ENGINE_ERR("(suppressing further timeout messages)");
    }

    /* Runaway detection. We reached here = the full STEP_MAX_CYCLES budget
     * was exhausted with NO clean retire (and the engine is not finished,
     * else the loop would have exited). Distinguish a genuine stuck/loop
     * (diverged ISS spinning at a fixed PC) from a legitimate WFI stall.
     *
     * - WFI stall: the bridge resyncs to the DUT (is_wfi_stuck path) and the
     *   wrap polls rvviRefIsFinished() for clean termination. Never a runaway. */
    if (!g_iss->exec.wfi.get())
    {
        iss_reg_t now_pc = g_iss->exec.current_insn;
        if (g_runaway_last_valid && now_pc == g_runaway_last_pc)
        {
            /* Same PC across consecutive cycle-budget-exhausting timeouts:
             * the ISS is stuck, not slow. */
            g_runaway_count++;
            if (g_runaway_count >= RUNAWAY_THRESHOLD && !g_runaway)
            {
                g_runaway = true;  /* sticky */
                ENGINE_ERR("RUNAWAY detected: %d consecutive stuck-PC timeouts "
                           "(PC=0x%08x, retire #%llu) - ISS diverged and stuck, "
                           "signalling bridge to abort",
                           g_runaway_count,
                           (unsigned)now_pc,
                           (unsigned long long)g_retire_count);
            }
        }
        else
        {
            /* PC moved (or first timeout): restart the consecutive count. */
            g_runaway_count = 1;
        }
        g_runaway_last_pc = now_pc;
        g_runaway_last_valid = true;
    }

    return 0;  /* Timeout or sim ended without retirement */
}

/* Returns true once the runaway detector has latched (sticky). */
bool gvsoc_engine_is_runaway(void)
{
    return g_runaway;
}

uint64_t gvsoc_engine_pending_commits(void)
{
    if (!g_iss)
        return 0;
    return g_iss->timing.commit_push - g_iss->timing.commit_pop;
}

int gvsoc_engine_commit_stream(void)
{
    return 1;
}

int gvsoc_engine_state_current(void)
{
    return g_iss && g_iss->timing.trap_seq == g_popped_trap_seq;
}

int gvsoc_engine_materialize_commit(uint32_t *pc)
{
    if (!g_running || !g_gvsoc || g_finished || !g_iss || !pc)
        return -1;

    try
    {
        if (!engine_advance_to_commit(NULL))
            return -1;
    }
    catch (const std::exception &e)
    {
        ENGINE_ERR("materialize_commit: step() threw: %s", e.what());
        g_finished = true;
        return -1;
    }
    catch (...)
    {
        ENGINE_ERR("materialize_commit: step() threw unknown exception");
        g_finished = true;
        return -1;
    }

    *pc = (uint32_t)g_iss->timing.commit_pc[
        g_iss->timing.commit_pop % Cv32e40pEvents::COMMIT_RING];
    return 0;
}

bool gvsoc_engine_finished(void)
{
    return g_finished;
}

/* --------------------------------------------------------------------------
 * State query - direct ISS struct access (public members and header-inline
 * accessors only, no model .so methods)
 * ---------------------------------------------------------------------- */

uint32_t gvsoc_engine_get_pc(void)
{
    if (!g_iss)
        return 0;
    /* PC of the instruction that just retired (captured pre-step). */
    return (uint32_t)g_retired_pc;
}

uint32_t gvsoc_engine_get_insn(void)
{
    /* Opcode is not captured in DPI mode; always 0. */
    return g_retired_opcode;
}

void gvsoc_engine_set_pc(uint32_t pc)
{
    if (!g_iss) return;
    g_iss->exec.current_insn = (iss_reg_t)pc;
    g_retired_pc = pc;
    /* Drop unconsumed commits: after a force-resync they belong to the
     * pre-resync stream. */
    g_iss->timing.commit_stream_flush();
    /* Ask the exec loop to flush prefetch + insn cache at the next slow
     * dispatch (the flush methods live in the model .so). Both are
     * addr-keyed, so a stale line cannot serve the redirected PC anyway;
     * the flush is defensive. */
    g_iss->exec.pending_flush = true;
    /* A WFI here means wire delivery diverged from the DUT: the v2 ISS can
     * only wake through check_interrupts() (the held WFI entry must be
     * terminated into the commit FIFO), so a manual clear would corrupt the
     * commit stream. Report it and let the step timeout surface the stall. */
    if (g_iss->exec.wfi.get()) {
        ENGINE_ERR("set_pc: ISS in WFI at force-resync (PC=0x%08x) - "
                   "cannot wake it externally", pc);
    }
}

void gvsoc_engine_skip_irq(bool skip)
{
    g_dpi_skip_irq = skip;
    if (g_iss)
        g_iss->exec.skip_irq_check = skip;
}

bool gvsoc_engine_is_wfi(void)
{
    if (!g_iss) return false;
    return g_iss->exec.wfi.get();
}

uint32_t gvsoc_engine_get_gpr(uint32_t index)
{
    if (!g_iss || index >= 32)
        return 0;
    if (index == 0)
        return 0;  /* x0 is hardwired to zero */
    return (uint32_t)g_iss->regfile.get_reg_untimed((int)index);
}

uint32_t gvsoc_engine_get_fpr(uint32_t index)
{
    if (!g_iss || index >= 32)
        return 0;
    /* get_reg_gid maps the architectural FPR index into the unified file
     * (ZFINX: same slot as the GPR, matching the shared register file). */
    return (uint32_t)g_iss->regfile.get_freg_untimed(
        g_iss->regfile.get_reg_gid((int)index));
}

int gvsoc_engine_get_csr(uint32_t csr_addr, uint32_t *value)
{
    if (!g_iss || !value)
        return 0;

    auto it = g_csr_value_map.find(csr_addr);
    if (it == g_csr_value_map.end())
    {
        *value = 0;
        return 0;  /* CSR not in our map */
    }

    iss_reg_t raw = *(it->second);

    /* mstatus (0x300): reading mstatus.value directly bypasses the read
     * fixups the ISS applies on a CSR read, so reproduce them here. */
    if (csr_addr == 0x300)
    {
        /* SD bit (bit 31) = (FS==3) || (XS==3) */
        if ((((raw >> 13) & 3) == 3) || (((raw >> 15) & 3) == 3))
        {
            raw |= (1ULL << 31);
        }

        /* MPP: CV32E40P is M-mode only, so MPP always reads as M (3).
         * A CSR write can leave MPP=0/1/2 in mstatus.value until the next
         * ISS read; force it to 3 to match RTL. */
        raw = (raw & ~(0x3ULL << 11)) | (0x3ULL << 11);
    }

    /* CSRs hardwired/read-only in CV32E40P M-mode. The ISS may retain stale
     * values (direct writes from exception handling, or writes that bypass
     * write_mask enforcement); force the RTL read value (0). */
    if (csr_addr == 0x343)  /* mtval - hardwired to 0 */
    {
        raw = 0;
    }
    if (csr_addr == 0x7A2)  /* tdata2 - writable only from Debug Mode */
    {
        raw = 0;
    }

    *value = (uint32_t)raw;
    return 1;
}

/* --------------------------------------------------------------------------
 * State injection - GPR and FPR write
 * ---------------------------------------------------------------------- */

void gvsoc_engine_set_gpr(uint32_t index, uint32_t value)
{
    if (!g_iss || index >= 32 || index == 0)
        return;  /* x0 is hardwired to zero */
    g_iss->regfile.set_reg((int)index, value);
}

void gvsoc_engine_set_fpr(uint32_t index, uint32_t value)
{
    if (!g_iss || index >= 32)
        return;
    g_iss->regfile.set_freg(g_iss->regfile.get_reg_gid((int)index), value);
}

/* --------------------------------------------------------------------------
 * State injection - CSR write
 * ---------------------------------------------------------------------- */

int gvsoc_engine_set_csr(uint32_t csr_addr, uint32_t value)
{
    if (!g_iss)
        return 0;

    auto it = g_csr_value_map.find(csr_addr);
    if (it == g_csr_value_map.end())
    {
        ENGINE_LOG("set_csr: addr 0x%03x not in map, ignored", csr_addr);
        return 0;
    }

    /* mstatus (0x300): a direct write bypasses the ISS write-mask callback,
     * so apply the CV32E40P write mask here. Spec-writable on CV32E40P is
     * FS[14:13] + MPIE[7] + MIE[3] (0x6088); MPP[12:11] is WARL hardwired to
     * M. We use the superset 0x7888 (incl. MPP) because get_csr() read-forces
     * MPP back to M anyway, and on non-FPU configs FS is always 0 in the
     * incoming value, so the mask is safe for all configurations. */
    if (csr_addr == 0x300)
    {
        const uint32_t MSTATUS_WR_MASK = 0x7888u;
        iss_reg_t cur = *(it->second);
        value = (cur & ~MSTATUS_WR_MASK) | (value & MSTATUS_WR_MASK);
    }

    iss_reg_t old_val = *(it->second);
    *(it->second) = (iss_reg_t)value;
    ENGINE_LOG("set_csr: 0x%03x = 0x%08x (was 0x%08x) ptr=%p",
               csr_addr, value, (unsigned)old_val, (void*)it->second);
    return 1;
}

/* --------------------------------------------------------------------------
 * State injection - IRQ via the injector wires
 *
 * Each RVVI net drives its wire on the platform irq injector (bound at
 * init, see engine_bind_irq_wires): IrqRiscv's sync method updates mip and
 * runs check_interrupts() inside the model .so, so the full-mode switch and
 * the WFI wake-up follow the same path as a hardware interrupt line.
 * ---------------------------------------------------------------------- */

void gvsoc_engine_set_irq(uint64_t net_index, int value)
{
    if (!g_iss)
        return;

    if (net_index >= IRQ_NB_WIRES)
    {
        /* haltreq (net_index=19): inject a debug request via direct struct
         * write only. The ISS debug_req() method is compiled into the model
         * .so and not available at DPI link time; its side effects (exit WFI,
         * switch to full exec mode) are therefore NOT triggered here. */
        if (net_index == 19 && value)
        {
            g_iss->irq.req_debug = true;
            /* Set dcsr.cause = 3 (haltreq) in bits [8:6]. Irq::check() sets
             * debug_mode and depc but does not update dcsr.cause. */
            g_iss->csr.dcsr = (g_iss->csr.dcsr & ~(0x7u << 6)) | (3u << 6);
            ENGINE_LOG("set_irq: haltreq asserted -> req_debug=true, dcsr.cause=3");
        }
        else if (net_index > 19)
        {
            ENGINE_ERR("set_irq: unmapped net index %llu (value=%d)",
                       (unsigned long long)net_index, value);
        }
        return;
    }

    iss_reg_t old_mip = g_iss->csr.mip.value;

    /* Engine-owned call: guarded like every other gv:: entry point (no
     * exception may cross the DPI boundary). On a throw mip is unchanged
     * and the settle logic below naturally no-ops. */
    engine_call_safe("wire_update",
                     [&](){ g_irq_wire[net_index]->update(value); });

    iss_reg_t new_mip = g_iss->csr.mip.value;

    if (old_mip != new_mip)
    {
        static uint64_t irq_log_count = 0;
        irq_log_count++;
        if (irq_log_count <= 20)
        {
            ENGINE_LOG("set_irq: net=%llu wire=%s val=%d -> mip 0x%08x->0x%08x",
                       (unsigned long long)net_index, g_irq_wire_name[net_index],
                       value, (unsigned)old_mip, (unsigned)new_mip);
        }
        if (irq_log_count == 20)
            ENGINE_LOG("(suppressing further IRQ log messages)");

        /* Mark settle needed only when asserting new bits (not on deassert).
         * Bitwise, not arithmetic: a simultaneous clear of a high bit and
         * assert of a low one lowers the numeric value but still needs the
         * settle. */
        if (new_mip & ~old_mip)
            g_irq_pending_settle = true;
    }
}

/* Drain cycles granted to the pre-fetch IRQ guard after an mip assertion,
 * before the next gvsoc_engine_step() loop. */
static constexpr int SETTLE_DRAIN_CYCLES = 4;

/* Run up to SETTLE_DRAIN_CYCLES after an mip assertion, letting the
 * pre-fetch IRQ guard fire before the next gvsoc_engine_step() loop.
 * Does NOT count as a retire (g_retire_count is unchanged). If the IRQ is
 * taken during the drain, g_retired_pc is set to the trapped instruction's
 * PC so the next step() returns the handler entry as the next retire.
 * Safe to call with no settle pending (returns immediately). */
void gvsoc_engine_settle_irq(void)
{
    if (!g_irq_pending_settle || !g_running || !g_gvsoc || !g_iss)
        return;

    g_irq_pending_settle = false;

    /* With skip_irq_check the ISS won't take interrupts, so draining would
     * only execute extra instructions and misalign the ISS with the DUT. */
    if (g_dpi_skip_irq)
        return;

    iss_reg_t settle_start_pc = g_iss->exec.current_insn;

    /* No exception may cross the DPI boundary: same guard as the step loop. */
    try
    {
        for (int i = 0; i < SETTLE_DRAIN_CYCLES; i++)
        {
            g_gvsoc->step(g_clock_ps);

            if (g_finished)
                break;

            iss_reg_t settle_pc = g_iss->exec.current_insn;
            if (settle_pc != settle_start_pc)
            {
                /* IRQ taken: record the trapped instruction as the last retired
                 * PC so gvsoc_engine_get_pc() matches the DUT. */
                g_retired_pc = settle_start_pc;
                ENGINE_LOG("settle_irq: IRQ taken on drain cycle %d, PC 0x%08x -> 0x%08x",
                           i, (unsigned)settle_start_pc, (unsigned)settle_pc);
                return;
            }
        }
    }
    catch (const std::exception &e)
    {
        ENGINE_ERR("settle_irq: step() threw: %s", e.what());
        g_finished = true;
        return;
    }
    catch (...)
    {
        ENGINE_ERR("settle_irq: step() threw unknown exception");
        g_finished = true;
        return;
    }

    /* No PC change: IRQ masked (mstatus.MIE=0) or pending for next fetch;
     * gvsoc_engine_step() will catch it at the pre-fetch guard. */
    ENGINE_LOG("settle_irq: %d drain cycles, no PC change (IRQ pending or masked)",
               SETTLE_DRAIN_CYCLES);
}

/* --------------------------------------------------------------------------
 * Informed IRQ injection (OVPSim-style "deferint"). Contract in the header.
 *
 * The engine keeps iss.exec.skip_irq_check asserted on every step, so the
 * ISS never takes interrupts on its own. Here we lower the guard, cycle the
 * engine until the take lands its first commit, then restore the guard.
 * The commit stays queued for the caller's step-and-compare.
 * ---------------------------------------------------------------------- */
int gvsoc_engine_take_irq_for_one_step(int mcause_irq_id)
{
    if (!g_running || !g_gvsoc || g_finished || !g_iss)
        return 0;

    if (mcause_irq_id < 0 || mcause_irq_id > 31)
    {
        ENGINE_ERR("take_irq: invalid mcause irq id %d", mcause_irq_id);
        return 0;
    }

    /* Present ONLY the DUT-selected cause to Irq::check() for this step. The
     * generic check() arbitrates by standard RISC-V priority (MEI > MSI > MTI >
     * ...), but CV32E40P ranks the fast local interrupts (16..31) ABOVE MEI.
     * With two lines pending (e.g. 31 and 11) the generic ladder would take 11
     * while the DUT took 31. Masking mip to exactly the DUT's cause forces
     * check() to take it and compute the matching vectored entry; the full
     * net-driven mip is restored after the step. (bit index == mcause exception
     * code, not the RVVI net index.) */
    iss_reg_t old_mip = g_iss->csr.mip.value;
    g_iss->csr.mip.value = (1u << mcause_irq_id);

    /* The DUT only takes locally-enabled interrupts and mie is lockstep
     * state, so the wire delivery (set_irq -> check_interrupts) has already
     * woken the ISS from any WFI. A sleep here means delivery diverged;
     * report it instead of forcing the wake - a manual wfi clear would
     * leave the held WFI entry parked in the commit FIFO. */
    if (g_iss->exec.wfi.get())
    {
        ENGINE_ERR("take_irq: ISS still in WFI at injection (irq id=%d) - "
                   "wire delivery diverged from the DUT", mcause_irq_id);
    }

    /* Remember the engine-level defense so we can restore it exactly. */
    bool saved_dpi_skip = g_dpi_skip_irq;

    /* Lower the defense for ONE step (see window guarantee above). */
    g_dpi_skip_irq = false;
    g_iss->exec.skip_irq_check = false;

    ENGINE_LOG("take_irq: arming irq id=%d (mip 0x%08x->0x%08x), single-step inject",
               mcause_irq_id, (unsigned)old_mip,
               (unsigned)g_iss->csr.mip.value);

    /* Commits already queued at the injection: the ISS ran past the DUT's
     * interrupt boundary inside a load-response burst and executed
     * instructions the DUT never did. No clean recovery from here; report
     * it and let the compare surface the divergence. */
    uint64_t stale = g_iss->timing.commit_push - g_iss->timing.commit_pop;
    if (stale > 0)
    {
        ENGINE_ERR("take_irq: %llu stale commits at injection - ISS ran "
                   "past the DUT's interrupt boundary",
                   (unsigned long long)stale);
    }

    /* Step until the take lands its first commit (Irq::check() redirects to
     * the vector slot and the same dispatch executes it), but LEAVE the
     * commit queued: the caller's normal step-and-compare must serve the
     * vector-slot retire against the DUT's entry row - popping it here
     * would shift the whole stream by one. */
    int rc = 0;
    try
    {
        for (int i = 0; i < STEP_MAX_CYCLES; i++)
        {
            if (g_iss->timing.commit_push - g_iss->timing.commit_pop > stale)
            {
                rc = 1;
                break;
            }
            if (g_finished)
                break;
            g_gvsoc->step(g_clock_ps);
        }
    }
    catch (const std::exception &e)
    {
        ENGINE_ERR("take_irq: step() threw: %s", e.what());
        g_finished = true;
    }
    catch (...)
    {
        ENGINE_ERR("take_irq: step() threw unknown exception");
        g_finished = true;
    }

    /* Restore mip to its pre-inject net-driven value: the mask above was a
     * transient single-bit arm for this one step. mip is MRO in CV32E40P (it
     * mirrors DUT irq_i via the net path), so a later `csrr mip` must read what
     * irq_i drives, not this step's transient arm. */
    g_iss->csr.mip.value = old_mip;

    /* Re-assert the defense: the next normal step() must not take IRQs.
     * g_iss stays valid across step(): only shutdown clears it, and no
     * shutdown runs on this call path. */
    g_dpi_skip_irq = saved_dpi_skip;
    g_iss->exec.skip_irq_check = saved_dpi_skip;

    ENGINE_LOG("take_irq: rc=%d, vector-slot commit queued, ISS at 0x%08x "
               "(mcause=0x%08x)",
               rc, (unsigned)g_iss->exec.current_insn,
               (unsigned)g_iss->csr.mcause.value);

    return rc;
}
