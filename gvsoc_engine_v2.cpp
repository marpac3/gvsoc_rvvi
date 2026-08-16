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

/* Hot-path logging (per-take / per-sync sites: set_csr, set_irq, take_irq,
 * take_debug, settle, set_pc drain): every line is a vpi_printf through the
 * simulator transcript, and an IRQ-heavy lane crosses these sites tens of
 * thousands of times. Gated on the same env knob as the bridge's hot logs
 * (CV_RVVI_BRIDGE_VERBOSE, cached at init); init/shutdown/error sites keep
 * the always-on macros. */
static bool g_engine_verbose = false;
#define ENGINE_LOG_HOT(fmt, ...) \
    do { if (__builtin_expect(g_engine_verbose, 0)) ENGINE_LOG(fmt, ##__VA_ARGS__); } while(0)

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
/* Sub-core-cycle step used while an asynchronous take is imminent (see
 * engine_advance_to_commit): the engine regains control between core
 * dispatches, so the commit stream can be stopped exactly at the boundary
 * the DUT will prove. Below the core cycle (~3 ns) by construction. */
static constexpr int64_t g_clock_ps_fine = 1000;

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

/* When set, re-assert exec.skip_irq_check before every step(): the flag is
 * one-shot, consumed inside Irq::check() (the model still evaluates its
 * synchronous execute-trigger ahead of the gate on every boundary). */
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

/* Wire bindings on the platform irq injector, indexed by RVVI net:
 * 0=MSWInterrupt->msi, 1=MTimerInterrupt->mti, 2=MExternalInterrupt->mei,
 * 3..18=LocalInterrupt0..15->external_irq_16..31, 19=haltreq (RTL
 * debug_req_i). haltreq travels the wire path like the interrupt lines:
 * Cv32e40pIrq::haltreq_sync arms req_debug AND wakes a WFI-parked hart
 * with the full release sequence, which a bridge-side struct write cannot
 * do (insn_terminate is not callable across the .so boundary). */
static constexpr int  IRQ_NB_WIRES = 21;
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
    "haltreq",
    "wfi_wake",
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

    {
        const char *v = getenv("CV_RVVI_BRIDGE_VERBOSE");
        g_engine_verbose = (v && v[0] != '\0' && strcmp(v, "0") != 0);
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
        /* Never tear the engine down from this DPI thread.
         *
         * A genuine engine finish runs has_ended(), which sets
         * g_running = false; that path already returned at the
         * "if (!g_running) return;" guard at the top of this function, so
         * control reaches here ONLY when termination was forced WITHOUT the
         * engine reaching its own end:
         *   - SV-side abort (consecutive-mismatch watchdog, runaway, UVM
         *     timeout): g_finished still false;
         *   - bridge ENGINE_ERR (commit-ring overflow, step() threw, ...):
         *     g_finished set true only to halt stepping.
         * In both cases the sync run loop still owns the non-recursive engine
         * mutex (taken in Controller::start()), so stop() -> engine_lock()
         * would relock it and hang forever (observed: multi-minute "hangs"
         * that were really this self-deadlock until an external kill).
         * join() is worse: its sync branch loops on run_sync() until
         * is_sim_finished, resuming the sim. The process is exiting anyway;
         * drop the pointer and let the OS reclaim the engine. */
        ENGINE_LOG("shutdown from DPI thread - skipping engine stop/join "
                   "(avoids Controller mutex self-deadlock)");
        g_gvsoc = nullptr;
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
    /* Stall budget is TIME, not iterations: with the sub-cycle quantum
     * active (take imminent) an iteration advances 1/20 of a platform
     * quantum, and an iteration-counted bound would shrink the budget by
     * the same factor - measured on the +reset_debug lane, whose 620-cycle
     * first-fetch stall overran the shrunken window and failed the initial
     * haltreq injection. */
    const int64_t budget_ps = (int64_t)STEP_MAX_CYCLES * (int64_t)g_clock_ps;
    int64_t       spent_ps  = 0;
    for (;;)
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
                *cycles = (int)(spent_ps / (int64_t)g_clock_ps);
            return true;
        }

        if (g_finished || spent_ps >= budget_ps)
            break;

        /* Re-assert skip_irq_check before every step (one-shot, consumed
         * inside Irq::check).  Synchronous debug conditions - the
         * execute-address trigger - are evaluated by the model AHEAD of
         * its internal gate, so a suppressed dispatch still enters debug
         * at an armed tdata2 boundary; no engine carve-out is needed.
         *
         * The one-shot alone is NOT airtight: one 20 ns platform quantum
         * runs several core dispatches (core cycle ~3 ns) and only the
         * first consumes the one-shot - the later dispatches used to take
         * pending wire IRQs / re-arm haltreq on their own, racing the
         * DUT's entry boundary (the async trap-entry seam family).  The
         * level hold dpi_async_hold closes that window inside
         * Cv32e40pIrq::check(); re-asserted here so the guard heals even
         * if a caller toggled it out of band. */
        if (g_dpi_skip_irq)
        {
            g_iss->exec.skip_irq_check = true;
            g_iss->irq.dpi_async_hold  = true;
        }

        /* While an asynchronous take is imminent - an enabled wired
         * interrupt pending with mstatus.MIE set, or a debug request
         * armed/held outside debug mode - step at sub-cycle granularity so
         * the loop regains control at the FIRST commit, before the next
         * dispatch runs.  With the async hold up the model cannot stop at
         * the take boundary by itself, and a full 20 ns quantum would
         * over-execute up to ~6 killed-path instructions (phantom
         * writebacks and stores) that the state repair then has to paper
         * over.  Fine stepping bounds the overrun to zero dispatches and
         * lets the DUT-boundary injection (take_irq/take_debug) certify
         * `current_insn == mepc` in the common case.  The window lasts
         * from the wire rise to the DUT's entry row (a few retires), so
         * the extra step calls are negligible. */
        bool take_imminent =
            ((g_iss->csr.mie.value & g_iss->csr.mip.value &
              Cv32e40pIrq::IRQ_MASK) != 0 && g_iss->csr.mstatus.mie) ||
            ((g_iss->irq.req_debug || g_iss->irq.haltreq_level) &&
             !g_iss->exec.debug_mode);

        int64_t quantum = take_imminent ? (int64_t)g_clock_ps_fine
                                        : (int64_t)g_clock_ps;
        g_gvsoc->step(quantum);
        spent_ps += quantum;
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

int gvsoc_engine_head_commit_trapped(void)
{
    if (!g_running || !g_gvsoc || g_finished || !g_iss)
        return 0;
    if (g_iss->timing.commit_pop >= g_iss->timing.commit_push)
        return 0;
    return g_iss->timing.commit_trapped[
        g_iss->timing.commit_pop % Cv32e40pEvents::COMMIT_RING] ? 1 : 0;
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

/* Injectable-boundary probe for the DUT-informed async-entry take: returns 1
 * with *pc = exec.current_insn when the ISS sits exactly on a dispatch
 * boundary with nothing in flight - no queued commits (every served retire
 * consumed), no held commit-FIFO entries, no pending LSU access, not parked
 * in WFI, not in debug mode. On such a boundary current_insn is the next
 * unexecuted instruction, i.e. the mepc a take at this boundary would
 * produce; the bridge certifies it against the DUT row's fresh mepc before
 * injecting. Returns 0 (pc untouched) whenever any of that is pending: the
 * ISS is mid-burst and only the repair fallback can serve the row. */
int gvsoc_engine_take_boundary(uint32_t *pc)
{
    if (!g_running || !g_gvsoc || g_finished || !g_iss || !pc)
        return 0;
    if (g_iss->timing.commit_push != g_iss->timing.commit_pop)
        return 0;
    if (g_iss->timing.inflight_pending())
        return 0;
    if (g_iss->lsu.get_nb_pending_accesses() > 0)
        return 0;
    if (g_iss->exec.wfi.get() || g_iss->exec.debug_mode)
        return 0;
    *pc = (uint32_t)g_iss->exec.current_insn;
    return 1;
}

uint32_t gvsoc_engine_get_insn(void)
{
    /* Raw encoding of the last popped commit (16-bit for RVC rows); 0 when
     * the last retire was served without a pop (pin, virtual consume,
     * deferral) - the INS compare skips those rows. */
    return g_retired_opcode;
}

void gvsoc_engine_set_pc(uint32_t pc)
{
    if (!g_iss) return;
    /* Let parked work complete before redirecting: flushing the commit
     * stream while an instruction sits in the commit FIFO (its scoreboard
     * bit set at issue) or an LSU response is in flight drops the
     * writeback, leaves the load-use scoreboard bit set forever, and the
     * next reader of that register deadlocks the exec loop. Bounded
     * because back-to-back issue can keep the pipeline busy; the
     * architectural state is forced from the DUT after the redirect either
     * way. A held WFI never drains, so skip (reported below). */
    if (g_gvsoc && !g_iss->exec.wfi.get() &&
        (g_iss->lsu.get_nb_pending_accesses() > 0 || g_iss->timing.inflight_pending()))
    {
        /* At most the single in-flight access can retire here: CV32E40P's
         * LsuV2 is built with the default nb_outstanding=1 (cv32e40p_v2.py).
         * Revisit the post-redirect force set if that ever changes. */
        int steps = 0;
        try
        {
            /* A WFI executed by the drain itself parks the ISS with the held
             * entry pending: it will never drain, so stop immediately instead
             * of spinning to the budget (the caller sees the WFI report below
             * and can wake through the wire path). */
            while ((g_iss->lsu.get_nb_pending_accesses() > 0 ||
                    g_iss->timing.inflight_pending()) &&
                   !g_iss->exec.wfi.get() && steps < STEP_MAX_CYCLES)
            {
                /* skip_irq_check is one-shot, consumed every dispatch cycle:
                 * re-assert per step or the ISS takes the pending wire-driven
                 * IRQ on its own mid-drain (same idiom as the other step
                 * loops in this file). The level hold covers the dispatches
                 * the one-shot cannot reach within a quantum. */
                if (g_dpi_skip_irq)
                {
                    g_iss->exec.skip_irq_check = true;
                    g_iss->irq.dpi_async_hold  = true;
                }
                g_gvsoc->step(g_clock_ps);
                steps++;
            }
        }
        catch (const std::exception &e)
        {
            ENGINE_ERR("set_pc: engine step threw during pipeline drain: %s", e.what());
        }
        ENGINE_LOG_HOT("set_pc: drained pipeline in %d steps before redirect (PC=0x%08x)",
                   steps, pc);
        if (!g_iss->exec.wfi.get() &&
            (g_iss->lsu.get_nb_pending_accesses() > 0 || g_iss->timing.inflight_pending()))
            ENGINE_ERR("set_pc: pipeline still busy after drain budget (PC=0x%08x)", pc);
    }
    g_iss->exec.current_insn = (iss_reg_t)pc;
    /* Drop a latched-but-unconsumed exception redirect: the next slow
     * dispatch would otherwise overwrite this redirect with exception_pc
     * (exec_inorder consumes has_exception BEFORE fetching current_insn).
     * Concrete case: a WFI wake lets the exec free-run one insn past the
     * boundary - an ebreak the DUT kills with an IRQ take - whose raise()
     * latches the redirect; without this clear the forced redirect lands
     * on the sync-trap vector (mtvec base) instead of pc. The insn's CSR
     * side effects are force-corrected by the resync caller either way. */
    g_iss->exec.has_exception = false;
    g_retired_pc = pc;
    /* Drop unconsumed commits: after a force-resync they belong to the
     * pre-resync stream (including any produced by the drain above). */
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
    {
        g_iss->exec.skip_irq_check = skip;
        /* Level companion of the one-shot: closes the mid-quantum window
         * where dispatches past the first ran unguarded (see
         * engine_advance_to_commit and Cv32e40pIrq::dpi_async_hold). */
        g_iss->irq.dpi_async_hold = skip;
    }
}

bool gvsoc_engine_is_wfi(void)
{
    if (!g_iss) return false;
    return g_iss->exec.wfi.get();
}

bool gvsoc_engine_is_debug_mode(void)
{
    if (!g_iss) return false;
    return g_iss->exec.debug_mode;
}

uint32_t gvsoc_engine_get_debug_handler(void)
{
    if (!g_iss) return 0;
    return (uint32_t)g_iss->irq.debug_handler;
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

    /* Hwloop CSRs (0xCC0-2 loop0, 0xCC4-6 loop1): the architectural values
     * live behind header-inline accessors, not in a raw store - lpstart and
     * lpcount in the hwloop module, lpend in the personality shadow (the
     * module keeps the loop-back point, LPEND - 4). */
    if (csr_addr >= 0xCC0 && csr_addr <= 0xCC6 && csr_addr != 0xCC3)
    {
        int loop = (csr_addr >= 0xCC4) ? 1 : 0;
        switch (csr_addr - (loop ? 0xCC4u : 0xCC0u))
        {
            case 0: *value = (uint32_t)g_iss->hwloop.get_start(loop); break;
            case 1: *value = (uint32_t)g_iss->csr.hwloop_lpend[loop]; break;
            case 2: *value = (uint32_t)g_iss->hwloop.get_count(loop); break;
        }
        return 1;
    }

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

int gvsoc_engine_fprs_aliased(void)
{
#ifdef CONFIG_GVSOC_ISS_ZFINX
    return 1;
#else
    return 0;
#endif
}

/* --------------------------------------------------------------------------
 * State injection - CSR write
 * ---------------------------------------------------------------------- */

int gvsoc_engine_set_csr(uint32_t csr_addr, uint32_t value)
{
    if (!g_iss)
        return 0;

    /* Hwloop CSRs: mirror of the get_csr() special case, routed through the
     * module setters (set_count maintains the active bitmap) with the same
     * semantics as the CSR/ISA write path: lpstart/lpend bits [1:0] hardwired
     * 0, module end = LPEND - 4 (loop-back point), architectural LPEND in the
     * personality shadow. Restoring these on an IRQ-redirect rollback undoes
     * the loop-count decrement of a cancelled-and-reexecuted loop-end
     * instruction (RTL hwlp_mask behavior, corev_hw_loop.rst) - without this
     * the counters were the only compared state not restored at a redirect,
     * so the off-by-one became permanent. */
    if (csr_addr >= 0xCC0 && csr_addr <= 0xCC6 && csr_addr != 0xCC3)
    {
        int loop = (csr_addr >= 0xCC4) ? 1 : 0;
        switch (csr_addr - (loop ? 0xCC4u : 0xCC0u))
        {
            case 0: g_iss->hwloop.set_start(loop, value & ~3u); break;
            case 1:
            {
                uint32_t end = value & ~3u;
                g_iss->csr.hwloop_lpend[loop] = end;
                g_iss->hwloop.set_end(loop, end - 4);
                break;
            }
            case 2: g_iss->hwloop.set_count(loop, value); break;
        }
        ENGINE_LOG_HOT("set_csr: hwloop 0x%03x = 0x%08x", csr_addr, value);
        return 1;
    }

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
    ENGINE_LOG_HOT("set_csr: 0x%03x = 0x%08x (was 0x%08x) ptr=%p",
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
        ENGINE_ERR("set_irq: unmapped net index %llu (value=%d)",
                   (unsigned long long)net_index, value);
        return;
    }

    /* haltreq (net 19) rides the wire path below like the interrupt lines;
     * Cv32e40pIrq::haltreq_sync arms req_debug (dcsr.cause is written by
     * check() atomically with the entry) and wakes a WFI-parked hart. It
     * never touches mip, so the settle logic naturally no-ops. */
    if (net_index == 19 && value)
        ENGINE_LOG_HOT("set_irq: haltreq asserted -> wire");

    /* wfi_wake (net 20): bridge-driven pulse releasing a WFI-parked hart
     * (Cv32e40pIrq::wfi_wake_sync), no architectural effect, mip untouched. */
    if (net_index == 20 && value)
        ENGINE_LOG_HOT("set_irq: wfi_wake pulse -> wire");

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
                ENGINE_LOG_HOT("settle_irq: IRQ taken on drain cycle %d, PC 0x%08x -> 0x%08x",
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
    ENGINE_LOG_HOT("settle_irq: %d drain cycles, no PC change (IRQ pending or masked)",
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

    /* Entry guard, symmetric to take_debug_for_one_step: in debug mode
     * Irq::check() never reaches the IRQ ladder, so the injection cannot
     * land - and a debug-ROM retire inside the window would be handed to
     * the caller as if it were the vector-slot commit. The DUT taking an
     * interrupt proves IT is not in debug mode (the spec masks interrupts
     * there), so this state is a divergence: fail the injection and let
     * the caller's fallback realign. */
    if (g_iss->exec.debug_mode)
    {
        ENGINE_ERR("take_irq: ISS in debug mode at injection (irq id=%d) - "
                   "the DUT's take proves it is not; failing the injection",
                   mcause_irq_id);
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

    /* Suppress a latched debug request for this one window: the debug branch
     * outranks the IRQ ladder in Irq::check(), so a haltreq armed by the
     * level-sensitive wire would hijack this injection into a debug entry
     * the DUT did not take at this boundary (the DUT's arbitration took the
     * IRQ). Restored after the step: the wire is still high on the DUT too,
     * and the genuine entry follows on the DUT's own entry row. Nets are
     * DPI-delivered between bridge calls, so no re-arm can land inside the
     * window. */
    bool saved_req_debug       = g_iss->irq.req_debug;
    int  saved_req_debug_cause = g_iss->irq.req_debug_cause;
    bool saved_haltreq_level   = g_iss->irq.haltreq_level;
    g_iss->irq.req_debug     = false;
    g_iss->irq.haltreq_level = false;  /* check() re-arms from a held-high level */

    /* Lower the defense for ONE step (see window guarantee above): the
     * one-shot, and the level hold that keeps the ladder parked between
     * injection windows. */
    g_dpi_skip_irq = false;
    g_iss->exec.skip_irq_check = false;
    g_iss->irq.dpi_async_hold  = false;

    ENGINE_LOG_HOT("take_irq: arming irq id=%d (mip 0x%08x->0x%08x), single-step inject",
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
        /* Sub-cycle quanta: keep the TIME budget of the historical
         * coarse-quantum loop, not its iteration count. */
        const int inject_iters =
            STEP_MAX_CYCLES * (int)(g_clock_ps / g_clock_ps_fine);
        for (int i = 0; i < inject_iters; i++)
        {
            if (g_iss->timing.commit_push - g_iss->timing.commit_pop > stale)
            {
                rc = 1;
                break;
            }
            if (g_finished)
                break;
            /* Sub-cycle stepping: the loop regains control right at the
             * vector-slot commit, before any follower dispatch runs. A
             * full platform quantum used to execute past the entry and
             * push follower commits, deferring the entry row's state
             * compare to the burst tail and moving the ISS off the clean
             * boundary the next row's injection may need. */
            g_gvsoc->step(g_clock_ps_fine);
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
    g_iss->irq.dpi_async_hold  = saved_dpi_skip;

    /* haltreq_level mirrors the wire, not a request: always restore it (the
     * next net sync only fires on a level CHANGE). */
    g_iss->irq.haltreq_level = saved_haltreq_level;

    /* Re-arm the suppressed debug request only if the window stayed out of
     * debug. The suppression covers req_debug and the haltreq level, but
     * check() can still enter through its non-suppressed paths (trigger
     * execute-match, single-step re-entry) - those entries consume
     * req_debug as part of taking, and re-latching the saved value on top
     * would arm a stale entry for the next dret. A dropped haltreq request
     * is not lost: check() re-arms it from the restored level. */
    if (!g_iss->exec.debug_mode)
    {
        g_iss->irq.req_debug       = saved_req_debug;
        g_iss->irq.req_debug_cause = saved_req_debug_cause;
    }
    else
    {
        ENGINE_ERR("take_irq: debug entry during the injection window "
                   "(irq id=%d) via a non-suppressed path (trigger or "
                   "single-step re-entry) - suppressed request dropped, "
                   "the commit handed to the caller is NOT the vector slot",
                   mcause_irq_id);
    }

    ENGINE_LOG_HOT("take_irq: rc=%d, vector-slot commit queued, ISS at 0x%08x "
               "(mcause=0x%08x)",
               rc, (unsigned)g_iss->exec.current_insn,
               (unsigned)g_iss->csr.mcause.value);

    return rc;
}

int gvsoc_engine_take_debug_for_one_step(int dcsr_cause, int collide_irq_id,
                                         uint32_t collide_mepc,
                                         int collide_certify,
                                         uint32_t expected_dpc,
                                         int expected_dpc_valid)
{
    if (!g_running || !g_gvsoc || g_finished || !g_iss)
        return 0;

    /* Spontaneous entry: the model's own execute-trigger match
     * (Cv32e40pIrq::check) can enter debug before the DUT edge reaches
     * this injection, with the first debug-ROM commit already queued.
     * Arming req_debug now would leave it pending across the session
     * (check() only consumes it outside debug mode) and fire a spurious
     * re-entry after dret: report success, the caller serves the queued
     * commit against the DUT's entry row. */
    if (g_iss->exec.debug_mode)
    {
        /* A collision id has nowhere to go on this branch: the spontaneous
         * entry already happened without the take. Loud, not silent - if
         * this ever fires, the mstatus/mcause divergence at the ROM rows
         * has THIS discard as its root cause. */
        if (collide_irq_id >= 0)
        {
            ENGINE_ERR("take_debug: spontaneous entry discards an informed "
                       "IRQ collision (id=%d) - the model entered without "
                       "the take", collide_irq_id);
        }
        ENGINE_LOG_HOT("take_debug: ISS already in debug mode (cause=%d), "
                   "spontaneous trigger entry - no injection", dcsr_cause);
        return 1;
    }

    /* Arm the debug request with the DUT-observed cause; Cv32e40pIrq::check()
     * writes dcsr.cause and dpc atomically with the entry and redirects to
     * the debug ROM. Because the entry runs BEFORE the fetch of the current
     * insn, depc = current_insn is correct for every cause: the ebreak the
     * DUT never retired (cause 1), the next unexecuted insn on haltreq
     * (cause 3) and the insn after the stepped one (cause 4). */
    g_iss->irq.req_debug_cause = dcsr_cause & 0x7;
    g_iss->irq.req_debug = true;

    /* Informed interrupt+debug collision: the DUT's entry row carried the
     * CSR writes of an interrupt take, so check() must take exactly that
     * line before the entry (dpc = the vectored handler entry, mstatus/
     * mepc/mcause from the take). -1 = no collision observed. With
     * collide_certify set (adjacent-row candidates), the model takes the
     * line only if its entry boundary equals collide_mepc - the
     * timing-exact stale-candidate rejection this function cannot do
     * up-front (the ISS may still be mid-batch, short of the boundary). */
    g_iss->irq.collide_irq_id = collide_irq_id;
    g_iss->irq.collide_expected_mepc = (iss_reg_t)collide_mepc;
    g_iss->irq.collide_certify = (collide_certify != 0);

    /* A WFI-parked ISS cannot spontaneously wake for an injected take (same
     * diagnostic as take_irq): report it, the step loop below will surface
     * the stall. */
    if (g_iss->exec.wfi.get())
    {
        ENGINE_ERR("take_debug: ISS in WFI at injection (cause=%d) - "
                   "delivery diverged from the DUT", dcsr_cause);
    }

    /* Remember the engine-level defense so we can restore it exactly. */
    bool saved_dpi_skip = g_dpi_skip_irq;

    /* Lower the defense for ONE step (same window guarantee as take_irq):
     * one-shot plus the level hold, which gates the cause-3 entry between
     * injection windows. */
    g_dpi_skip_irq = false;
    g_iss->exec.skip_irq_check = false;
    g_iss->irq.dpi_async_hold  = false;

    /* The debug entry only happens inside Cv32e40pIrq::check(), which the
     * fast dispatch handler never calls. No forcing is needed here: the
     * co-sim personality pins the slow handler (can_switch_to_fast_mode()
     * returns false while commit_stream_observed is set), so the next
     * dispatch always runs check(). Do NOT call switch_to_full_mode() from
     * the bridge: it inlines a reference to ExecInOrder::exec_instr_check_all,
     * which the ISS library does not export - vsim aborts with an undefined
     * symbol (vsim-12005) at the first call. The success check on debug_mode
     * below still guards against an unrelated commit slipping through. */

    ENGINE_LOG_HOT("take_debug: arming debug entry (cause=%d), single-step inject",
               dcsr_cause);

    /* The arm above lands dpc = the insn the DUT killed only if the ISS is
     * parked exactly at that boundary, with current_insn still pointing at
     * the unexecuted insn. Two independent lags can put it past the
     * boundary, and the commit ring only shows one of them:
     *
     *  - stale commits: retires already visible in the ring, not yet served
     *    by the step-and-compare.
     *
     *  - a non-empty exec commit FIFO: its head is a held insn (LSU async
     *    load) and every sync insn dispatched behind it has ALREADY executed
     *    - result in the regfile, current_insn advanced (the sync-follower
     *    branch of exec_instr_check_all) - while its retire is parked in the
     *    inflight ring by Cv32e40pEvents::event_retire_account instead of
     *    the commit ring. commit_push does not move for it, so this lag is
     *    invisible to the stale count even though the ISS is architecturally
     *    ahead of the boundary.
     *
     * In the second case the entry can no longer be placed before the killed
     * insn: that insn already retired inside the model, and its writeback
     * cannot be undone from here. Report it - the loop below refuses to
     * certify such an entry - and let the compare surface the divergence.
     * A WFI-parked ISS also holds a FIFO entry (its own parked WFI, from
     * sleep_enter) which is not an over-execution: the WFI diagnostic above
     * already covers that case. */
    bool at_boundary = true;
    uint64_t stale = g_iss->timing.commit_push - g_iss->timing.commit_pop;
    if (stale > 0)
    {
        at_boundary = false;
        ENGINE_ERR("take_debug: %llu stale commits at injection - ISS ran "
                   "past the DUT's debug-entry boundary",
                   (unsigned long long)stale);
    }
    if (g_iss->timing.inflight_pending() && !g_iss->exec.wfi.get())
    {
        at_boundary = false;
        ENGINE_ERR("take_debug: insn parked in the commit FIFO at injection "
                   "(ISS at 0x%08x) - the ISS already executed past the DUT's "
                   "debug-entry boundary, dpc cannot be placed before it",
                   (unsigned)g_iss->exec.current_insn);
    }
    /* DUT-informed boundary certification (C4 by-seed): for a non-ebreak,
     * non-collision entry the DUT's dpc IS the kill boundary, so an ISS
     * parked elsewhere must not certify even when its own boundary is clean
     * (no stale commits, nothing inflight) - "clean but WRONG" was exactly
     * the un-diagnosed shape (a phantom breakpoint exception parked the ISS
     * on mtvec, the entry certified there, and only the mepc compare at the
     * ROM rows surfaced it 50 rows deep). cause 1 is exempt (dpc = the
     * ebreak's own pc while current_insn sits past it - the structural
     * retire-vs-capture offset); a collide take is exempt too (pre-take
     * boundary == collide_mepc, certified separately); the bridge waives
     * the check (expected_dpc_valid=0) for unrecognized entry collisions,
     * where dpc is an un-executed entry target. A WFI-parked ISS is exempt
     * like everywhere else in the boundary bookkeeping (take_boundary
     * returns 0, the inflight check above skips wfi): its current_insn is
     * not a comparable kill boundary and the WFI diagnostic above already
     * covers the park. */
    if (expected_dpc_valid && collide_irq_id < 0 && (dcsr_cause & 7) != 1 &&
        !g_iss->exec.wfi.get() &&
        (uint32_t)g_iss->exec.current_insn != expected_dpc)
    {
        at_boundary = false;
        ENGINE_ERR("take_debug: ISS boundary 0x%08x != DUT dpc 0x%08x "
                   "(cause=%d) - entry not on the DUT's kill boundary",
                   (unsigned)g_iss->exec.current_insn, expected_dpc,
                   dcsr_cause);
    }

    /* Step until the entry lands its first debug-ROM commit, and LEAVE the
     * commit queued: the caller's step-and-compare serves it against the
     * DUT's first debug-ROM retire row.
     *
     * Success needs the entry to be observed BEFORE any new commit, not just
     * together with one. Testing debug_mode at the moment a commit appears
     * cannot tell "the pending insn retired, then the entry fired at the
     * next dispatch" (dpc one insn too high, the retire served against the
     * ROM row, the stream permanently slipped) from "the entry fired, then
     * the ROM row retired" (correct): both show a new commit with
     * debug_mode set. Watching the entry on its own dispatch separates
     * them, since Cv32e40pIrq::check() enters before the fetch of
     * current_insn and pushes no commit of its own.
     *
     * The queued commit is then proven to belong to the entry by its
     * trap_seq stamp: check() bumps timing.trap_seq atomically with the
     * entry, and each retire is stamped when it EXECUTES (in the inflight
     * ring for held/parked ones). A pre-entry insn draining after the entry
     * therefore still carries the older stamp and is rejected. */
    int rc = 0;
    bool entered = false;
    uint64_t entry_trap_seq = 0;
    uint64_t base_trap_seq  = g_iss->timing.trap_seq;
    try
    {
        /* Sub-cycle quanta: keep the TIME budget of the historical
         * coarse-quantum loop, not its iteration count (measured: the
         * +reset_debug initial haltreq entry sits behind a 620-cycle
         * first-fetch stall and overran the iteration-counted window). */
        const int inject_iters =
            STEP_MAX_CYCLES * (int)(g_clock_ps / g_clock_ps_fine);
        for (int i = 0; i < inject_iters; i++)
        {
            /* The entry is an EVENT; exec.debug_mode is a LEVEL. Polling the
             * level alone misses entries whose flag is not up at any poll
             * boundary of this loop (measured: 228/228 haltreq entries of
             * the pulp interrupt+debug lane broke here, reported as
             * "unrelated commit" at the halt address itself). The edge is
             * observable regardless: check() bumps timing.trap_seq
             * atomically with the entry and writes dcsr.cause; requiring
             * the ARMED cause on dcsr attributes the bump to this entry
             * rather than to an IRQ take in the same window (a stale
             * same-cause dcsr can in principle mis-attribute, but the
             * commit-stamp and PC compares downstream surface that
             * loudly). */
            if (!entered)
            {
                bool level = g_iss->exec.debug_mode;
                bool edge  = (g_iss->timing.trap_seq != base_trap_seq) &&
                             (((g_iss->csr.dcsr >> 6) & 0x7u) ==
                              (uint32_t)dcsr_cause);
                if (level || edge)
                {
                    entered = true;
                    entry_trap_seq = g_iss->timing.trap_seq;
                }
            }

            uint64_t queued = g_iss->timing.commit_push - g_iss->timing.commit_pop;
            if (queued > stale)
            {
                uint64_t idx = (g_iss->timing.commit_push - 1) %
                               Cv32e40pEvents::COMMIT_RING;
                if (!entered)
                {
                    ENGINE_ERR("take_debug: unrelated commit (PC=0x%08x) while "
                               "the debug request was pending - entry not taken",
                               (unsigned)g_iss->timing.commit_pc[idx]);
                }
                else if (queued != stale + 1)
                {
                    ENGINE_ERR("take_debug: %llu commits queued by the entry "
                               "window (expected 1) - a pre-entry retire is "
                               "queued ahead of the debug-ROM row",
                               (unsigned long long)(queued - stale));
                }
                else if (g_iss->timing.commit_trap_seq[idx] != entry_trap_seq)
                {
                    ENGINE_ERR("take_debug: queued commit (PC=0x%08x) executed "
                               "before the entry - it would be served against "
                               "the DUT's debug-ROM row, dpc is one insn past "
                               "the DUT's",
                               (unsigned)g_iss->timing.commit_pc[idx]);
                }
                else if (!at_boundary)
                {
                    /* The entry itself is well formed, but the ISS was not at
                     * the DUT's boundary when it was armed (reported above):
                     * the row served against the debug-ROM retire is the
                     * pre-boundary commit still ahead of it in the ring, and
                     * dpc belongs to an insn the DUT never reached. */
                    ENGINE_ERR("take_debug: entry taken off the DUT's boundary "
                               "(depc=0x%08x) - not certifying the injection",
                               (unsigned)g_iss->csr.depc);
                }
                else
                {
                    rc = 1;
                }
                break;
            }
            if (g_finished)
                break;
            /* Sub-cycle stepping, as in take_irq: the loop observes the
             * entry edge and the first ROM commit at dispatch granularity,
             * with no follower dispatched past them. */
            g_gvsoc->step(g_clock_ps_fine);
        }
    }
    catch (const std::exception &e)
    {
        ENGINE_ERR("take_debug: step() threw: %s", e.what());
        g_finished = true;
    }
    catch (...)
    {
        ENGINE_ERR("take_debug: step() threw unknown exception");
        g_finished = true;
    }

    /* Re-assert the defense: the next normal step() must not take IRQs. */
    g_dpi_skip_irq = saved_dpi_skip;
    g_iss->exec.skip_irq_check = saved_dpi_skip;
    g_iss->irq.dpi_async_hold  = saved_dpi_skip;

    /* req_debug is a LATCHED request only consumed by a successful entry in
     * Cv32e40pIrq::check(). Left armed after a failed injection, it would
     * hijack the next take_irq_for_one_step() window into a bogus debug
     * entry (the debug branch outranks the IRQ ladder). Disarm it
     * unconditionally, mirroring take_irq's unconditional mip restore: on
     * success check() already consumed it, so this is a no-op there. */
    g_iss->irq.req_debug = false;
    g_iss->irq.req_debug_cause = 3;
    /* Same latching hazard: an unconsumed collision id would make a later
     * entry take a stale interrupt. On success check() already reset it. */
    g_iss->irq.collide_irq_id = -1;

    /* depc is logged so a wrong entry point is visible here, without the
     * engine having to know the DUT's dpc (only the bridge holds it). */
    ENGINE_LOG_HOT("take_debug: rc=%d, entered=%d, debug-ROM commit queued, "
               "ISS at 0x%08x (dcsr=0x%08x, depc=0x%08x)",
               rc, (int)entered, (unsigned)g_iss->exec.current_insn,
               (unsigned)g_iss->csr.dcsr, (unsigned)g_iss->csr.depc);

    return rc;
}
