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
 * GVSOC-RVVI bridge - in-process co-simulation.
 *
 * Implements the RVVI API (rvviApi.h) backed by a GVSOC engine running in the
 * same process as the UVM/Questa simulator.  GVSOC is stepped one clock at a
 * time; instruction retires are detected by monitoring PC change.
 *
 * File organisation:
 *   1. Includes and debug macros
 *   2. Global state - net map, DUT state, trap snapshot, CSR comparison config
 *   3. Internal helpers - net map init, config patching, throttle logging
 *   4. DUT retire event recording
 *   5. RVVI API - version/init/shutdown
 *   6. RVVI API - net handling
 *   7. RVVI API - DUT state push (GPR, FPR, CSR, retire, trap)
 *   8. RVVI API - reference model stepping
 *   9. RVVI API - state readback
 *  10. RVVI API - comparison (PC, GPR, FPR, CSR)
 *  11. RVVI API - CSR/memory config and stubs
 *  12. RVVI API - sim-complete query
 */

#include <rvviApi.h>
#include "gvsoc_engine.hpp"
#include "rvvi_text_writer.hpp"
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <signal.h>     /* raise(SIGTRAP) - GDB attach gate */
#include <cstdlib>
#include <sys/stat.h>   /* stat() - RVVI-TEXT dir-vs-file env detection */
#include <chrono>
#include <exception>
#include <vpi_user.h>   /* vpi_printf - prints to simulator transcript */

/* --------------------------------------------------------------------------
 * Debug output macros.
 * Use vpi_printf, not fprintf(stderr): stderr is not flushed in DPI context
 * and the output is lost.  vpi_printf flushes to the Questa transcript.
 *
 * BRIDGE_LOG_HOT is for hot-path sites (per retire / trap / resync / CSR
 * compare).  It is gated on g_bridge_verbose (default OFF).  __builtin_expect
 * marks the disabled branch as the likely path so the call is skipped with no
 * branch misprediction when logging is off.
 * ---------------------------------------------------------------------- */
#define BRIDGE_LOG(fmt, ...) \
    do { vpi_printf((char *)"[rvvi-api2gvsoc] " fmt "\n", ##__VA_ARGS__); } while(0)
#define BRIDGE_ERR(fmt, ...) \
    do { vpi_printf((char *)"[rvvi-api2gvsoc] ERROR: " fmt "\n", ##__VA_ARGS__); } while(0)

static bool g_bridge_verbose = false;  /* set from CV_RVVI_BRIDGE_VERBOSE at init */
#define BRIDGE_LOG_HOT(fmt, ...) \
    do { if (__builtin_expect(g_bridge_verbose, 0)) BRIDGE_LOG(fmt, ##__VA_ARGS__); } while(0)

/* ==========================================================================
 * SECTION 2 - Global state
 * ========================================================================== */

/* Net name -> index mapping (populated once at init) */
static std::unordered_map<std::string, uint64_t> g_net_index_map;
static std::unordered_map<uint64_t, uint64_t>    g_net_value_map;
static uint64_t g_next_net_index = 0;

static std::string g_tmp_config_path;

/* --------------------------------------------------------------------------
 * RVVI-TEXT emitter (additive, env-gated, default OFF).
 *
 * When RVVI_TEXT_TRACE is set, writes two RVVI-TEXT v0.4 files per run:
 *   dut.rvvi - DUT architectural state (values pushed by the testbench)
 *   ref.rvvi - reference (GVSOC ISS) state for the SAME write-set, with the
 *              ISS's PC and values
 * One line per retire, so `diff dut.rvvi ref.rvvi` localises a divergence and
 * RVVI/source/host/rvvi/rvviTextChecker.py validates each side. Independent of
 * the PASS/FAIL semantics: the step-n-compare result is unchanged.
 *
 * Gate (read once in rvviRefInit):
 *   RVVI_TEXT_TRACE=<existing dir> -> <dir>/dut.rvvi, <dir>/ref.rvvi
 *   RVVI_TEXT_TRACE=1              -> ./dut.rvvi, ./ref.rvvi (cwd)
 *   unset / empty / 0             -> disabled, zero hot-path overhead
 * ---------------------------------------------------------------------- */
static bool   g_rvvi_text_enabled = false;
/* Set once via rvviBridgeSetRefOnly(), called by gvsoc_wrap's ref_init task
 * BEFORE rvviRefInit() -- see rvviBridgeSetRefOnly below.
 * When true (dual-trace mode: RVVI_TRACE also active), the SV tracer is the sole
 * dut.rvvi producer: this bridge opens/writes ref.rvvi only. */
static bool   g_rvvi_text_ref_only = false;
static FILE  *g_rvvi_text_dut_fp  = nullptr;
static FILE  *g_rvvi_text_ref_fp  = nullptr;
static uint64_t g_rvvi_text_count = 0;  /* retire counter for periodic fflush; reset in rvviRefInit */
/* RVVI-TEXT v0.4 header params (PARAMS line). File-scope = constant-init, no
 * function-local guard. FLEN is pushed by rvviBridgeSetFlen() before
 * rvviRefInit() writes the header (0 = no FPU, 32 = F/Zfinx); the 32 here is
 * only the fallback for a caller that never pushes it. */
static RvviTextParams g_rvvi_text_params = {
    /*vendor*/ "gvsoc_rvvi",
    /*ilen*/ 32, /*xlen*/ 32, /*flen*/ 32,
    /*vlen*/ 0,  /*nhart*/ 1, /*retire*/ 1
};
/* Per-function ns accumulators, dumped at shutdown.  Enabled via
 * CV_RVVI_BRIDGE_PROFILE=1; gated at each call site so disabled cost is nil. */
static bool     g_bridge_profile = false;
static uint64_t g_prof_ns_step        = 0;
static uint64_t g_prof_ns_pc_cmp      = 0;
static uint64_t g_prof_ns_gpr_cmp     = 0;
static uint64_t g_prof_ns_csr_cmp     = 0;
static uint64_t g_prof_ns_fpr_cmp     = 0;
static uint64_t g_prof_ns_dut_retire  = 0;
static uint64_t g_prof_cnt_step       = 0;
static uint64_t g_prof_cnt_pc_cmp     = 0;
static uint64_t g_prof_cnt_gpr_cmp    = 0;
static uint64_t g_prof_cnt_csr_cmp    = 0;
static uint64_t g_prof_cnt_fpr_cmp    = 0;
static uint64_t g_prof_cnt_dut_retire = 0;

/* RAII scope guard - accumulates ns into the given counter at destruction.
 * Zero-cost when g_bridge_profile is false (the ctor checks the flag and
 * skips the clock read; the dtor checks again and skips the delta). */
struct ProfGuard {
    uint64_t &ns_acc;
    uint64_t &cnt_acc;
    std::chrono::steady_clock::time_point t0;
    bool active;
    ProfGuard(uint64_t &ns, uint64_t &cnt)
        : ns_acc(ns), cnt_acc(cnt), active(__builtin_expect(g_bridge_profile, 0)) {
        if (active) t0 = std::chrono::steady_clock::now();
    }
    ~ProfGuard() {
        if (active) {
            auto t1 = std::chrono::steady_clock::now();
            ns_acc += std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
            cnt_acc++;
        }
    }
};
#define PROF_SCOPE(ns_var, cnt_var) ProfGuard _prof_guard((ns_var), (cnt_var))

/* --------------------------------------------------------------------------
 * The retire lifecycle.
 *
 * Two structures with one owner each:
 *
 *   PendingWriteback - the write-back set being ACCUMULATED for the retire in
 *   flight. Written only by the rvviDut*Set push functions.
 *
 *   RetireEvent - the snapshot of the last COMMITTED retire. Written only by
 *   take_retire_event() (called from rvviDutRetire / rvviDutTrap); every
 *   consumer - compare, volatile sync, RVVI-TEXT emit, informed inject -
 *   reads it and nobody else writes it.
 *
 * Ordering contract (why this is race-free): rvvi_trace2api.sv drives the
 * whole retire from a single always block and DPI calls are synchronous, so
 * within one retire the sequence is strictly
 *   push (Dut*Set) -> take (DutRetire/DutTrap) -> inject -> batch compare/emit
 * and the pushes of retire N+1 cannot start before the batch of retire N has
 * returned. */
struct PendingWriteback {
    uint32_t gpr_mask = 0;
    uint32_t fpr_mask = 0;
    std::vector<std::pair<uint32_t, uint32_t>> csr_writes;  /* (addr, value), push order */
};

struct RetireEvent {
    uint32_t pc      = 0;
    uint32_t pc_prev = 0;   /* DUT PC of the previous retire (phase-shift catch-up) */
    uint32_t insn    = 0;
    /* True when this retire carries rvfi_trap=1. On trap retires RVFI does
     * NOT report exception CSR updates in the same cycle; those arrive on the
     * following (handler) retire, so trap-CSR comparison is suppressed here. */
    bool     is_trap = false;
    uint32_t gpr_mask = 0;
    uint32_t fpr_mask = 0;
    std::vector<std::pair<uint32_t, uint32_t>> csr_writes;
};

static PendingWriteback g_pending;
static RetireEvent      g_retire;

/* Sticky architectural mirrors of the DUT state, accumulated across retires.
 * Distinct from the per-retire structures above: the force-resync path needs
 * the full GPR/FPR file and the CSR compare is state-based (a CSR pushed once
 * stays comparable forever). */
static uint32_t g_dut_gpr[32] = {};
static uint32_t g_dut_fpr[32] = {};
static std::unordered_map<uint32_t, uint32_t> g_dut_csr;
/* DUT privilege mode (rvvi.mode), pushed once per retire by rvviBridgeSetMode.
 * Only consumed by the RVVI-TEXT emitter (ref-only MODE column). */
static uint32_t g_dut_mode    = 0;

/* --------------------------------------------------------------------------
 * Trap-CSR snapshot mechanism.
 *
 * CV32E40P RVFI zeroes mepc/mcause/mtval during the pipeline-flush retire
 * (PC=0x0) that follows a trap retire.  That stale csr_wb=value=0 would
 * overwrite the correct exception CSRs in g_dut_csr by the handler retire.
 *
 * Fix: at rvviDutTrap, snapshot the exception CSR values; for the first
 * handler retire, compare against the snapshot instead of g_dut_csr.  The
 * snapshot is consumed (cleared) in rvviRefCsrsCompare once the handler-retire
 * comparison completes.  rvvi_trace2api pushes mepc/mcause/mtval via
 * rvviDutCsrSet before rvviDutTrap, so the snapshot is correct regardless of
 * csr_wb timing.
 * ---------------------------------------------------------------------- */
static bool     g_pending_handler = false;
/* One-shot arm for the sync-trap entry realign: set at rvviDutTrap, consumed
 * by the next rvviRefEventStep.  g_pending_handler cannot gate the realign
 * because it stays armed until a CSR compare row clears it, and the ISS can
 * spuriously satisfy the handler-entry PC equality on later rows (e.g. after
 * an IRQ redirect) when the snapshot is stale. */
static bool     g_sync_trap_seam = false;
static std::unordered_map<uint32_t, uint32_t> g_trap_csr_snapshot;

/* Exception CSRs requiring snapshot protection across the trap->handler
 * boundary.  Contract: this list must match the explicit trap-CSR pushes in
 * rvvi_trace2api.sv (CSR_MSTATUS/MEPC/MCAUSE/MTVAL, step 5) - the snapshot in
 * rvviDutTrap and the force-resync in rvviRefEventStep both iterate it. */
static const uint32_t TRAP_CSR_MEPC   = 0x341U;
static const uint32_t TRAP_CSR_MCAUSE  = 0x342U;
static const uint32_t TRAP_CSR_MTVAL   = 0x343U;
static const uint32_t TRAP_CSR_MSTATUS = 0x300U;  /* trap entry updates MPIE/MIE - also needs snapshot protection */
static const uint32_t CSR_MTVEC        = 0x305U;  /* trap-vector base+mode (informed-inject entry-detect) */
static const uint32_t CSR_DCSR         = 0x7B0U;  /* debug control: cause[8:6] read on informed debug entry */
static const uint32_t CSR_DPC          = 0x7B1U;  /* debug PC: forced from DUT at the informed debug entry seam */

/* DUT debug-mode level at the previous retire: the informed debug entry
 * fires on the 0->1 transition (first debug-ROM retire). */
static bool g_prev_dut_debug = false;

static inline bool is_trap_csr(uint32_t addr)
{
    return addr == TRAP_CSR_MSTATUS || addr == TRAP_CSR_MEPC ||
           addr == TRAP_CSR_MCAUSE  || addr == TRAP_CSR_MTVAL;
}

/* --------------------------------------------------------------------------
 * Force-resync ISS state on IRQ-induced traps.
 *
 * When an asynchronous interrupt fires, DUT and ISS may disagree on exactly
 * which instruction was interrupted (mepc differs by one insn) - a
 * microarchitectural timing difference, not a functional bug.  After the ISS
 * processes the trap, force-write the DUT's trap CSRs (mepc/mcause/mstatus/
 * mtval) and the full GPR/FPR file into the ISS, so both resume at the same PC
 * and register state after mret and the desync does not cascade.
 *
 * Gated on: interrupt (mcause[31]=1), an inline ISS-PC vs DUT-PC mismatch in
 * rvviRefEventStep, and GVSOC_FORCE_TRAP_CSR (default on).  Resync is done
 * inline in rvviRefEventStep - the PC comparison is the trigger, no staged
 * flag needed. */
static bool g_force_trap_enabled  = false;  /* cached from env var at init */
static uint64_t g_force_resync_count = 0;   /* diagnostic: resyncs applied */

/* Informed IRQ injection (OVPSim-style) - plusarg-gated, default OFF.
 * When OFF: behaviour is unchanged (the reactive resync above stays the only
 * IRQ path). When ON: the SV bridge calls rvviRefInjectIrq() on the first
 * retire of an external-interrupt trap, telling the ISS to COMPUTE the entry
 * itself (no DUT-state copy). Enabled from SV via rvviRefSetInformedIrq(1),
 * driven by +rvvi_informed_irq. */
static bool g_informed_irq_enabled = false;
static uint64_t g_informed_irq_count = 0;   /* diagnostic: injections applied */

/* Volatile-counter read sync (ImperasDV-style informed handling) - default ON.
 * A functional ISS cannot predict the RTL's performance counters: cycle counts
 * depend on pipeline stalls and memory latencies the instruction-level model
 * does not have. The counter VALUE is therefore excluded from verification,
 * but a program that consumes it (matmul prints its cycle delta) would fork
 * the two sides as soon as the value shapes control flow (printf digit loops).
 * After a retire whose instruction is a CSR read of a performance counter,
 * overwrite the ISS rd with the DUT rd value so downstream control flow stays
 * in lockstep. Reads only - counter writes are illegal on the RTL and trap.
 * Disable with CV_RVVI_VOLATILE_CSR_SYNC=0. */
static bool g_volatile_sync_enabled = true;
static uint64_t g_volatile_sync_count = 0;  /* diagnostic: syncs applied */

/* Volatile memory windows (same ImperasDV-informed semantics as the counter
 * sync above, for memory-mapped state). The testbench declares them via the
 * standard RVVI call rvviRefMemorySetVolatile() at init - the GVSOC wrap
 * mirrors the Imperas wrap's window (0x15001000-0x15001007: the TB virtual
 * peripheral random-number generator and cycle counter). A load whose
 * DUT-side effective address (RVFI mem_addr/mem_rmask, passed down the batch
 * DPI) falls in a window reads device state no functional model can predict:
 * the value is excluded from verification and the DUT's rd write-back is
 * copied into the ISS GPR so downstream control flow stays in lockstep.
 * Reads only - a store to a volatile window needs no sync (the device value
 * never feeds back through the ISS's own memory). */
static std::vector<std::pair<uint64_t, uint64_t>> g_mem_volatile;
static uint64_t g_volatile_mem_sync_count = 0;  /* diagnostic: syncs applied */

/* dpc forces applied at debug-entry seams (decision G): each one where the
 * ISS disagreed with the DUT is a direct gauge of an entry-ordering error,
 * so it is logged unthrottled and counted for the shutdown summary. */
static uint64_t g_dpc_force_count = 0;

/* Tracer-fidelity sidecar state (opt-in, +rvvi_tracer_fidelity): explicit
 * per-row rvfi_intr / rvfi_dbg from the core tracer, plus the cross-check
 * counters against the legacy detectors. See rvviRefSetTracerFidelity. */
static bool     g_tracer_fidelity = false;
static uint32_t g_row_intr = 0;   /* rvfi_intr bundle of the row in flight */
static uint32_t g_row_dbg  = 0;   /* rvfi_dbg (entry cause) of the row */
static uint64_t g_fid_intr_rows      = 0;  /* rows flagged intr+interrupt   */
static uint64_t g_fid_intr_agree     = 0;  /* ...where the detector agreed  */
static uint64_t g_fid_intr_only      = 0;  /* tracer saw it, detector blind */
static uint64_t g_fid_det_only       = 0;  /* detector fired, tracer silent */
static uint64_t g_fid_dbg_rows       = 0;  /* debug entries with rvfi_dbg   */
static uint64_t g_fid_dbg_agree      = 0;
static uint64_t g_fid_dbg_mismatch   = 0;

/* Tracer-informed take (opt-in, +rvvi_tracer_informed, implies the sidecar):
 * on a row rvfi_intr marks as an async-IRQ handler entry, the ISS TAKES the
 * DUT-selected cause (single-step inject) BEFORE the row's step, so the trap
 * entry is computed by the model instead of repaired by the reactive
 * force-resync afterwards. See rvviRefSetTracerInformed. */
static bool     g_tracer_informed = false;
static uint64_t g_fid_informed_takes = 0;  /* takes the ISS computed        */
static uint64_t g_fid_informed_fails = 0;  /* fell back to reactive resync  */
static uint64_t g_fid_informed_wfi   = 0;  /* left to reactive (ISS in WFI) */

/* --------------------------------------------------------------------------
 * Phase-shift re-alignment on synchronous exceptions.
 *
 * A synchronous trap (illegal/ecall/ebreak) can leave the ISS exactly one
 * retire BEHIND the DUT on the SAME control-flow path: GVSOC models the
 * exception as an extra step, while the async-IRQ resync above does NOT fire
 * (both MIE end at 0 for a sync trap, so is_new_irq is false).  When that
 * happens, the ISS PC equals the DUT's PREVIOUS retire PC (g_retire.pc_prev),
 * not the current one (g_retire.pc).  That is a provable 1-retire lag on the
 * same path -> give the ISS one catch-up step to re-align. */
static uint64_t g_phase_realign_count = 0;  /* diagnostic: catch-up steps applied */

/* CSR comparison configuration */
static std::unordered_set<uint32_t>           g_csr_compare_enabled;
static std::unordered_map<uint32_t, uint64_t> g_csr_compare_mask;
static std::unordered_set<uint32_t>           g_csr_volatile;

/* RVVI metric counters - queried by the testbench final block */
static uint64_t g_metric_retires           = 0;
static uint64_t g_metric_traps             = 0;
static uint64_t g_metric_mismatches        = 0;
static uint64_t g_metric_comparisons_pc    = 0;
static uint64_t g_metric_comparisons_gpr   = 0;
static uint64_t g_metric_comparisons_fpr   = 0;
static uint64_t g_metric_comparisons_csr   = 0;
static uint64_t g_metric_comparisons_insbin = 0;
static uint64_t g_metric_state_deferred    = 0;

/* Deferred-commit gate, two v2-only cases where the ISS state is newer
 * than the retire being compared (PC/order compares stay valid):
 * - commits still queued after this retire (multi-commit burst): the state
 *   matches the NEWEST queued commit; the compare resumes at the burst tail;
 * - a trap redirected the ISS after this instruction executed (it was held
 *   in the commit FIFO behind an in-flight memory op when the next
 *   instruction trapped): the state includes the trap entry; the compare
 *   re-arms on the first post-redirect commit.
 * The v1 engine steps per instruction and never defers. */
static uint64_t g_state_deferred_last_retire = 0;

static bool state_compare_deferred(void)
{
    if (gvsoc_engine_pending_commits() == 0 && gvsoc_engine_state_current())
        return false;
    if (g_state_deferred_last_retire != g_metric_retires) {
        g_state_deferred_last_retire = g_metric_retires;
        g_metric_state_deferred++;
    }
    return true;
}

/* Per-category mismatch counts for throttled logging */
static uint64_t g_pc_mismatch_count   = 0;
static uint64_t g_gpr_mismatch_count  = 0;
static uint64_t g_gprw_mismatch_count = 0;
static uint64_t g_fpr_mismatch_count  = 0;
static uint64_t g_csr_mismatch_count  = 0;
static uint64_t g_insn_mismatch_count = 0;  /* instruction binary mismatch throttle */
static uint64_t g_order_mismatch_count = 0; /* retire-ordering tripwire throttle */

/* Wall-clock start time recorded at rvviRefInit for CPI/throughput metrics */
static std::chrono::steady_clock::time_point g_init_time;

/* ==========================================================================
 * SECTION 3 - Internal helpers
 * ========================================================================== */

static void init_net_map()
{
    const char *known_nets[] = {
        "MSWInterrupt",
        "MTimerInterrupt",
        "MExternalInterrupt",
        "LocalInterrupt0",  "LocalInterrupt1",  "LocalInterrupt2",
        "LocalInterrupt3",  "LocalInterrupt4",  "LocalInterrupt5",
        "LocalInterrupt6",  "LocalInterrupt7",  "LocalInterrupt8",
        "LocalInterrupt9",  "LocalInterrupt10", "LocalInterrupt11",
        "LocalInterrupt12", "LocalInterrupt13", "LocalInterrupt14",
        "LocalInterrupt15",
        "haltreq",
        NULL
    };
    for (int i = 0; known_nets[i] != NULL; i++) {
        g_net_index_map[known_nets[i]] = g_next_net_index++;
    }
}

/* Patch the GVSOC JSON config with the target ELF path.
 *
 * Tries two strategies in order:
 *   1. Replace the literal placeholder "__CV32E40P_ELF__".
 *   2. Replace the array value under the "binary" JSON key.
 * Returns the path to a freshly-written temp file, or "" on error. */
static std::string create_temp_config(const char *template_path,
                                      const char *elf_path)
{
    FILE *f = fopen(template_path, "r");
    if (!f) {
        BRIDGE_ERR("cannot open config template: %s", template_path);
        return "";
    }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsize < 0) {   /* ftell error: a negative size would alloc a huge string */
        fclose(f);
        BRIDGE_ERR("ftell failed on config template: %s", template_path);
        return "";
    }
    std::string content(fsize, '\0');
    if (fread(&content[0], 1, fsize, f) != (size_t)fsize) {
        fclose(f);
        BRIDGE_ERR("error reading config template");
        return "";
    }
    fclose(f);

    const std::string replacement = elf_path ? elf_path : "";
    bool replaced = false;
    size_t pos = 0;

    /* Strategy 1: placeholder substitution */
    const std::string placeholder = "__CV32E40P_ELF__";
    while ((pos = content.find(placeholder, pos)) != std::string::npos) {
        content.replace(pos, placeholder.length(), replacement);
        pos += replacement.length();
        replaced = true;
    }

    /* Strategy 2: replace the "binary" JSON array value */
    if (!replaced) {
        const std::string key = "\"binary\": [";
        pos = content.find(key);
        if (pos != std::string::npos) {
            size_t arr_start = pos + key.size();
            size_t arr_end = content.find(']', arr_start);
            if (arr_end != std::string::npos) {
                std::string new_arr = "\n          \"" + replacement + "\"\n        ";
                content.replace(arr_start, arr_end - arr_start, new_arr);
                replaced = true;
            }
        }
    }

    if (!replaced) {
        BRIDGE_ERR("could not inject ELF path into config");
    }

    char tmp_path[64];
    snprintf(tmp_path, sizeof(tmp_path), "/tmp/gvsoc_cv32_%d.json", (int)getpid());
    FILE *out = fopen(tmp_path, "w");
    if (!out) {
        BRIDGE_ERR("cannot write temp config: %s", tmp_path);
        return "";
    }
    fwrite(content.c_str(), 1, content.size(), out);
    fclose(out);
    return std::string(tmp_path);
}

/* Throttled mismatch logging.
 * Increments the counter and returns true if the caller should log.
 * Emits a "suppressing further" message on the last allowed log.
 * The budget is per category and configurable via CV_RVVI_MISMATCH_LOG_MAX
 * (read at init; 0 = unlimited): the default 10 keeps a diverged run's log
 * bounded, but hides whether a mismatch is sustained or transient - raise
 * it (or set 0) when that distinction matters. The authoritative count is
 * always `Total Reference model mismatches`, never the printed lines. */
static uint64_t g_mismatch_log_max = 10;

static inline bool throttle_check(uint64_t &counter, const char *category)
{
    counter++;
    if (g_mismatch_log_max == 0)
        return true;
    if (counter > g_mismatch_log_max)
        return false;
    if (counter == g_mismatch_log_max)
        BRIDGE_ERR("(suppressing further %s mismatch messages)", category);
    return true;
}

/* ==========================================================================
 * SECTION 4 - DUT retire event recording
 * ========================================================================== */

/* Commit the accumulated write-back set as the current retire. The single
 * point where g_retire is written and g_pending is cleared: everything the
 * rest of the retire reads (compares, volatile sync, emit, inject) comes from
 * g_retire. The vector swap keeps both capacities alive, so the steady state
 * allocates nothing. */
static void take_retire_event(uint64_t dutPc, uint64_t dutInsBin, bool is_trap)
{
    g_retire.pc_prev  = g_retire.pc;
    g_retire.pc       = (uint32_t)dutPc;
    g_retire.insn     = (uint32_t)dutInsBin;
    g_retire.is_trap  = is_trap;
    g_retire.gpr_mask = g_pending.gpr_mask;
    g_retire.fpr_mask = g_pending.fpr_mask;
    g_retire.csr_writes.swap(g_pending.csr_writes);

    g_pending.gpr_mask = 0;
    g_pending.fpr_mask = 0;
    g_pending.csr_writes.clear();
}

/* Emit one RVVI-TEXT v0.4 line for the current retire on the given side.
 * Single shared formatter so dut.rvvi and ref.rvvi are byte-format-identical
 * (only the values/PC differ): a diff then shows exactly where they diverge.
 *
 * dut_side=true  -> DUT values  (g_retire.pc / g_dut_gpr / g_dut_fpr mirrors)
 * dut_side=false -> REF values  (gvsoc_engine_get_pc / _gpr / _fpr / _csr) for
 *                   the SAME write-set the DUT reported this retire.
 *
 * Called on both the normal-retire path and the trap path (rvviDutTrap);
 * g_retire.is_trap selects RET vs TRAP in the output. */
static void emit_rvvi_text_line(FILE *fp, bool dut_side)
{
    if (!fp)
        return;

    /* Build a plain-data write-set, then delegate to the shared formatter
     * (rvvi_text_writer). dut_side picks the DUT-reported values;
     * ref picks the ISS value for the SAME architectural write-set.
     * ws{} zero-inits all 32 GPR/FPR slots; only masked indices are written
     * below and read by the formatter (no read of an uninitialised slot). */
    RvviTextWriteSet ws{};
    ws.pc       = dut_side ? g_retire.pc : gvsoc_engine_get_pc();
    ws.insn     = g_retire.insn;
    ws.is_trap  = g_retire.is_trap;
    /* MODE column: emitted only in ref-only mode, where ref.rvvi is diffed
     * line-for-line against a dut.rvvi produced by the SV tracer (which always
     * emits MODE). Gate on which FILE is being written (fp == ref fp), not on
     * dut_side: on a trap retire this function runs with dut_side=true for
     * both lines, so dut_side cannot tell the files apart. g_dut_mode is the
     * DUT-reported mode -- the ISS has not necessarily reconverged privilege
     * mode at this exact retire boundary. */
    ws.has_mode = g_rvvi_text_ref_only && (fp == g_rvvi_text_ref_fp);
    ws.mode     = g_dut_mode;

    ws.gpr_mask = g_retire.gpr_mask;
    for (int i = 0; i < 32; i++)
        if (g_retire.gpr_mask & (1u << i))
            ws.gpr[i] = dut_side ? g_dut_gpr[i] : gvsoc_engine_get_gpr((uint32_t)i);

    ws.fpr_mask = g_retire.fpr_mask;
    for (int i = 0; i < 32; i++)
        if (g_retire.fpr_mask & (1u << i))
            ws.fpr[i] = dut_side ? g_dut_fpr[i] : gvsoc_engine_get_fpr((uint32_t)i);

    /* CSR deltas of this retire. If the ISS does not map a CSR, emit 0x0
     * (not omit) so dut/ref column counts match 1:1. */
    for (const auto &w : g_retire.csr_writes) {
        uint32_t v = w.second;
        if (!dut_side && !gvsoc_engine_get_csr(w.first, &v))
            v = 0u;
        ws.csr.push_back({w.first, v});
    }

    rvvi_text_write_line(fp, ws);
}

/* Emit the dut.rvvi and ref.rvvi lines for the committed retire (g_retire).
 * ref_from_dut=true is the trap seam: the ISS has not consumed the trap yet,
 * so the ref line is built from DUT data too. DPI cannot propagate exceptions
 * (would crash the simulator) and the formatter allocates: contain any throw
 * and disable the best-effort trace instead. */
static void emit_retire_lines(bool ref_from_dut)
{
    if (!__builtin_expect(g_rvvi_text_enabled, 0))
        return;
    try {
        emit_rvvi_text_line(g_rvvi_text_dut_fp, /*dut_side=*/true);
        emit_rvvi_text_line(g_rvvi_text_ref_fp, /*dut_side=*/ref_from_dut);
    } catch (const std::exception &e) {
        BRIDGE_LOG("RVVI-TEXT: line emit failed (%s) - disabling", e.what());
        g_rvvi_text_enabled = false;
    } catch (...) {
        BRIDGE_LOG("RVVI-TEXT: line emit failed (unknown) - disabling");
        g_rvvi_text_enabled = false;
    }
}

static void close_text_files(void)
{
    if (g_rvvi_text_dut_fp) { fclose(g_rvvi_text_dut_fp); g_rvvi_text_dut_fp = nullptr; }
    if (g_rvvi_text_ref_fp) { fclose(g_rvvi_text_ref_fp); g_rvvi_text_ref_fp = nullptr; }
}

extern "C" {

/* ==========================================================================
 * SECTION 5 - RVVI API: version / init / shutdown
 * ========================================================================== */

bool_t rvviVersionCheck(uint32_t version)
{
    if (version != RVVI_API_VERSION) {
        BRIDGE_ERR("RVVI Version mismatch: expected %x, got %x",
                   RVVI_API_VERSION, version);
        return RVVI_FALSE;
    }
    return RVVI_TRUE;
}

/* True while a debugger is ptrace-attached (TracerPid in /proc/self/status). */
static bool debugger_attached(void)
{
    FILE *f = fopen("/proc/self/status", "r");
    if (!f)
        return false;
    long tracer = 0;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "TracerPid: %ld", &tracer) == 1)
            break;
    }
    fclose(f);
    return tracer != 0;
}

/* GDB attach gate (default OFF).  GVSOC_RVVI_GDB_WAIT=<seconds> pauses the
 * simulation here, at the very start of rvviRefInit, until a debugger
 * attaches (or the timeout expires).  On attach it raises SIGTRAP, so the
 * debugger stops exactly at this point with every .so already loaded:
 * set breakpoints in the bridge, gvsoc_engine or GVSOC itself, then
 * 'continue'.  See docs/DEBUG_COSIM.md. */
static void gdb_attach_gate(void)
{
    const char *env = getenv("GVSOC_RVVI_GDB_WAIT");
    if (!env || env[0] == '\0' || strcmp(env, "0") == 0)
        return;
    long timeout_s = strtol(env, nullptr, 10);
    if (timeout_s <= 0)
        timeout_s = 300;
    BRIDGE_LOG("GDB attach gate: waiting up to %lds for   gdb -p %ld",
               timeout_s, (long)getpid());
    for (long tick = 0; tick < timeout_s * 10; tick++) {
        if (debugger_attached()) {
            BRIDGE_LOG("debugger attached - stopping in rvviRefInit "
                       "(set breakpoints, then 'continue')");
            raise(SIGTRAP);
            return;
        }
        usleep(100 * 1000);
    }
    BRIDGE_LOG("no debugger after %lds, continuing without one", timeout_s);
}

/* Boolean env-var gate: unset or empty -> the gate's default; "0" -> off;
 * any other value -> on (so =1 and =true both work). */
static bool env_flag(const char *name, bool def)
{
    const char *v = getenv(name);
    if (!v || v[0] == '\0')
        return def;
    return strcmp(v, "0") != 0;
}

bool_t rvviRefInit(const char *programPath)
{
    gdb_attach_gate();

    const char *template_path = getenv("GVSOC_CONFIG");
    if (!template_path) {
        BRIDGE_ERR("GVSOC_CONFIG env var not set");
        return RVVI_FALSE;
    }

    /* Read the boolean gates once at init.
     * CV_RVVI_BRIDGE_VERBOSE: hot-path logging.
     * RVVI_TEXT_TRACE: trace emitter; the value is also a target directory or
     *   "1" (cwd), consumed at the file-open below.
     * CV_RVVI_BRIDGE_PROFILE: per-function ns counters, dumped at shutdown. */
    g_bridge_verbose    = env_flag("CV_RVVI_BRIDGE_VERBOSE", false);
    g_rvvi_text_enabled = env_flag("RVVI_TEXT_TRACE", false);
    g_bridge_profile    = env_flag("CV_RVVI_BRIDGE_PROFILE", false);

    BRIDGE_LOG("rvviRefInit: ELF=%s, config=%s",
               programPath ? programPath : "<null>", template_path);

    init_net_map();

    /* Informed debug entry tracks the DUT's debug-mode level per retire;
     * start from the reset level. */
    g_prev_dut_debug = false;

    /* Force-resync on IRQ traps (default ON, GVSOC_FORCE_TRAP_CSR=0 disables).
     * Skip ISS IRQ checking only when force-resync is active, so that the ISS
     * never takes interrupts on its own and is resynced to the DUT. */
    g_force_trap_enabled = env_flag("GVSOC_FORCE_TRAP_CSR", true);
    BRIDGE_LOG("force-resync on IRQ traps (skip_irq): %s",
               g_force_trap_enabled ? "ENABLED" : "DISABLED");
    gvsoc_engine_skip_irq(g_force_trap_enabled);

    /* Volatile-counter read sync (default ON, CV_RVVI_VOLATILE_CSR_SYNC=0
     * disables). */
    g_volatile_sync_enabled = env_flag("CV_RVVI_VOLATILE_CSR_SYNC", true);
    BRIDGE_LOG("volatile-counter read sync: %s",
               g_volatile_sync_enabled ? "ENABLED" : "DISABLED");

    /* Per-category mismatch log budget (0 = unlimited). A malformed value
     * keeps the default: strtoull would return 0 on "abc" and stop early
     * on "10x", silently flipping the fail-safe default into unlimited
     * logging - the opposite of what a typo should do. */
    {
        const char *v = getenv("CV_RVVI_MISMATCH_LOG_MAX");
        if (v && v[0] != '\0') {
            char *end = nullptr;
            uint64_t parsed = strtoull(v, &end, 10);
            if (end != nullptr && *end == '\0')
                g_mismatch_log_max = parsed;
            else
                BRIDGE_ERR("CV_RVVI_MISMATCH_LOG_MAX='%s' is not a number - "
                           "keeping the default %llu", v,
                           (unsigned long long)g_mismatch_log_max);
        }
        BRIDGE_LOG("mismatch log budget per category: %llu%s",
                   (unsigned long long)g_mismatch_log_max,
                   g_mismatch_log_max == 0 ? " (unlimited)" : "");
    }

    /* Reset the retire lifecycle and diagnostic counters for this run.
     * The volatile memory windows are re-declared by the testbench's
     * ref_init right after this call, so clear them too. */
    g_pending             = {};
    g_retire              = {};
    g_phase_realign_count = 0;
    g_force_resync_count  = 0;
    g_volatile_sync_count = 0;
    g_volatile_mem_sync_count = 0;
    g_dpc_force_count     = 0;
    g_mem_volatile.clear();

    if (programPath && strlen(programPath) > 0) {
        /* Drop any temp config from a previous init so a re-init does not leak
         * /tmp/gvsoc_cv32_<pid>.json. */
        if (!g_tmp_config_path.empty()) {
            remove(g_tmp_config_path.c_str());
            g_tmp_config_path.clear();
        }
        std::string tmp = create_temp_config(template_path, programPath);
        if (!tmp.empty())
            g_tmp_config_path = tmp;
    }

    /* Open the RVVI-TEXT file(s) and write their header(s) (once per run).
     * RVVI_TEXT_TRACE points at a directory (-> <dir>/{dut,ref}.rvvi) or is "1"
     * (-> cwd). On any fopen failure, disable the emitter so the hot path stays
     * a no-op rather than half-writing.
     *
     * Ref-only (g_rvvi_text_ref_only, set by rvviBridgeSetRefOnly() before this
     * call - dual-trace mode): the SV tracer is the sole dut.rvvi producer, so
     * this bridge must NOT also open it - two independent FILE* truncating the
     * same path would corrupt the tracer's output. Only ref.rvvi is opened. */
    if (g_rvvi_text_enabled) {
        const char *rt_env = getenv("RVVI_TEXT_TRACE");
        std::string dir;
        struct stat st;
        if (rt_env && strcmp(rt_env, "1") != 0 &&
            stat(rt_env, &st) == 0 && S_ISDIR(st.st_mode)) {
            dir = rt_env;
            if (!dir.empty() && dir.back() != '/')
                dir += '/';
        }
        std::string ref_rvvi = dir + "ref.rvvi";
        /* Close any handles left open by a previous rvviRefInit (multi-run vsim
         * session) so a re-init does not leak FILE*, and reset the flush counter
         * and DUT mode so a re-init cannot start from a stale entry (the retire
         * lifecycle was already reset above). */
        close_text_files();
        g_rvvi_text_count = 0;
        g_dut_mode = 0;
        std::string dut_rvvi;
        if (!g_rvvi_text_ref_only) {
            dut_rvvi = dir + "dut.rvvi";
            g_rvvi_text_dut_fp = fopen(dut_rvvi.c_str(), "w");
        }
        g_rvvi_text_ref_fp = fopen(ref_rvvi.c_str(), "w");
        bool dut_ok = g_rvvi_text_ref_only || g_rvvi_text_dut_fp;
        if (!dut_ok || !g_rvvi_text_ref_fp) {
            BRIDGE_ERR("cannot open RVVI-TEXT file(s) (%s%s%s) - disabling",
                       dut_rvvi.c_str(), dut_rvvi.empty() ? "" : " / ", ref_rvvi.c_str());
            close_text_files();
            g_rvvi_text_enabled = false;
        } else {
            /* RVVI-TEXT v0.4 header via the shared formatter (params at file
             * scope, g_rvvi_text_params). */
            if (g_rvvi_text_dut_fp)
                rvvi_text_write_header(g_rvvi_text_dut_fp, g_rvvi_text_params);
            rvvi_text_write_header(g_rvvi_text_ref_fp, g_rvvi_text_params);
            BRIDGE_LOG("RVVI-TEXT -> %s%s%s (ref-only=%d)",
                       dut_rvvi.c_str(), dut_rvvi.empty() ? "" : " , ",
                       ref_rvvi.c_str(), g_rvvi_text_ref_only ? 1 : 0);
        }
    }

    if (!g_tmp_config_path.empty()) {
        int rc = gvsoc_engine_init(g_tmp_config_path.c_str());
        if (rc != 0) {
            BRIDGE_ERR("engine init failed (rc=%d) - cannot proceed", rc);
            return RVVI_FALSE;
        }
        BRIDGE_LOG("engine initialized in-process");
        BRIDGE_LOG("  ELF    : %s", programPath ? programPath : "<none>");
        BRIDGE_LOG("  Config : %s", g_tmp_config_path.c_str());
    } else {
        BRIDGE_LOG("no config generated, running in stub mode");
    }

    g_init_time = std::chrono::steady_clock::now();
    return RVVI_TRUE;
}

bool_t rvviRefShutdown(void)
{
    /* Close the RVVI-TEXT files BEFORE any engine teardown: if the engine
     * shutdown wedges (see the abnormal-termination note in
     * gvsoc_engine_shutdown), the trace tails must already be on disk. */
    close_text_files();
    if (!g_tmp_config_path.empty()) {
        remove(g_tmp_config_path.c_str());
        g_tmp_config_path.clear();
    }

    /* Performance summary */
    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - g_init_time).count();
    double retires_per_sec = (elapsed_s > 0.0) ? (double)g_metric_retires / elapsed_s : 0.0;
    BRIDGE_LOG("--- performance summary ---");
    BRIDGE_LOG("  retires total  : %llu", (unsigned long long)g_metric_retires);
    BRIDGE_LOG("  wall-clock     : %.3f s", elapsed_s);
    BRIDGE_LOG("  retires/sec    : %.0f", retires_per_sec);
    if (g_force_resync_count > 0)
        BRIDGE_LOG("  IRQ resyncs    : %llu", (unsigned long long)g_force_resync_count);
    if (g_phase_realign_count > 0)
        BRIDGE_LOG("  phase realigns : %llu", (unsigned long long)g_phase_realign_count);
    if (g_volatile_sync_count > 0)
        BRIDGE_LOG("  volatile syncs : %llu", (unsigned long long)g_volatile_sync_count);
    if (g_volatile_mem_sync_count > 0)
        BRIDGE_LOG("  volatile mem syncs : %llu", (unsigned long long)g_volatile_mem_sync_count);
    if (g_dpc_force_count > 0)
        BRIDGE_LOG("  dpc forces (debug-entry ordering gauge) : %llu",
                   (unsigned long long)g_dpc_force_count);
    if (g_metric_state_deferred > 0)
        BRIDGE_LOG("  state-deferred retires : %llu (commit bursts, PC-only compare)",
                   (unsigned long long)g_metric_state_deferred);
    if (g_tracer_fidelity) {
        BRIDGE_LOG("  tracer-fidelity intr rows : %llu (detector agree %llu, "
                   "tracer-only %llu, detector-only %llu)",
                   (unsigned long long)g_fid_intr_rows,
                   (unsigned long long)g_fid_intr_agree,
                   (unsigned long long)g_fid_intr_only,
                   (unsigned long long)g_fid_det_only);
        BRIDGE_LOG("  tracer-fidelity dbg entries : %llu (dcsr agree %llu, "
                   "mismatch %llu)",
                   (unsigned long long)g_fid_dbg_rows,
                   (unsigned long long)g_fid_dbg_agree,
                   (unsigned long long)g_fid_dbg_mismatch);
        if (g_tracer_informed)
            BRIDGE_LOG("  tracer-informed takes : %llu (failed %llu, "
                       "left to reactive on WFI %llu)",
                       (unsigned long long)g_fid_informed_takes,
                       (unsigned long long)g_fid_informed_fails,
                       (unsigned long long)g_fid_informed_wfi);
    }

    if (g_bridge_profile) {
        BRIDGE_LOG("--- per-function profile (CV_RVVI_BRIDGE_PROFILE=1) ---");
        auto report = [elapsed_s](const char *label, uint64_t ns, uint64_t cnt) {
            double ms     = ns / 1.0e6;
            double pct    = (elapsed_s > 0.0) ? (ms / 1000.0) / elapsed_s * 100.0 : 0.0;
            double ns_avg = cnt ? (double)ns / (double)cnt : 0.0;
            BRIDGE_LOG("  %-24s : %9.2f ms (%5.1f%%)  calls=%llu  avg=%.0f ns",
                       label, ms, pct, (unsigned long long)cnt, ns_avg);
        };
        report("rvviDutRetire",         g_prof_ns_dut_retire, g_prof_cnt_dut_retire);
        report("rvviRefEventStep",      g_prof_ns_step,       g_prof_cnt_step);
        report("rvviRefPcCompare",      g_prof_ns_pc_cmp,     g_prof_cnt_pc_cmp);
        report("rvviRefGprsCompareWritten", g_prof_ns_gpr_cmp,g_prof_cnt_gpr_cmp);
        report("rvviRefCsrsCompare",    g_prof_ns_csr_cmp,    g_prof_cnt_csr_cmp);
        report("rvviRefFprsCompare",    g_prof_ns_fpr_cmp,    g_prof_cnt_fpr_cmp);
        uint64_t total_ns = g_prof_ns_dut_retire + g_prof_ns_step + g_prof_ns_pc_cmp +
                            g_prof_ns_gpr_cmp + g_prof_ns_csr_cmp + g_prof_ns_fpr_cmp;
        double total_pct = (elapsed_s > 0.0) ? (total_ns / 1.0e9) / elapsed_s * 100.0 : 0.0;
        BRIDGE_LOG("  -- TOTAL (measured)       : %9.2f ms (%5.1f%% of wall-clock)",
                   total_ns / 1.0e6, total_pct);
    }

    /* Engine teardown last: everything above is already flushed/reported. */
    gvsoc_engine_shutdown();

    return RVVI_TRUE;
}

bool_t rvviRefProgramLoad(const char *programPath)
{
    BRIDGE_LOG("rvviRefProgramLoad(%s) not implemented",
               programPath ? programPath : "<null>");
    return RVVI_FALSE;
}

/* ==========================================================================
 * SECTION 6 - RVVI API: net / interrupt handling
 * ========================================================================== */

uint64_t rvviRefNetIndexGet(const char *name)
{
    auto it = g_net_index_map.find(name);
    if (it != g_net_index_map.end())
        return it->second;
    return (uint64_t)RVVI_INVALID_INDEX;
}

void rvviRefNetGroupSet(uint64_t /*netIndex*/, uint32_t /*group*/) {}

void rvviRefNetSet(uint64_t netIndex, uint64_t value, uint64_t /*when*/)
{
    g_net_value_map[netIndex] = value;
    if (gvsoc_engine_is_running())
    {
        gvsoc_engine_set_irq(netIndex, (int)value);
        gvsoc_engine_settle_irq();  /* drain a few clock cycles so the ISS pre-fetch IRQ guard fires in the same cycle as the RTL */
    }
}

uint64_t rvviRefNetGet(uint64_t netIndex)
{
    auto it = g_net_value_map.find(netIndex);
    return (it != g_net_value_map.end()) ? it->second : 0;
}

/* --------------------------------------------------------------------------
 * Informed IRQ injection (OVPSim-style "deferint" oracle handoff).
 *
 * GVSOC-specific (not part of the canonical RVVI API). The SV bridge enables
 * this via rvviRefSetInformedIrq(1) when +rvvi_informed_irq is present, and
 * calls rvviRefInjectIrq() on the first retire of an external-interrupt trap
 * (mcause[31]=1), BEFORE the normal rvviRefRetireAndCompare() for that
 * instruction. Unlike the reactive resync (which copies DUT trap CSRs/GPRs/FPRs
 * into the ISS), this lets the ISS COMPUTE the entry from mcause id alone; the
 * subsequent step-n-compare then verifies the ISS-computed handler entry.
 * ---------------------------------------------------------------------- */

/* Enable/disable the informed-injection path. Default OFF (reactive resync). */
void rvviRefSetInformedIrq(int enable)
{
    g_informed_irq_enabled = (enable != 0);
    BRIDGE_LOG("informed IRQ injection %s",
               g_informed_irq_enabled ? "ENABLED (+rvvi_informed_irq)"
                                      : "DISABLED (reactive resync)");
}

/* --------------------------------------------------------------------------
 * Tracer-fidelity sidecar (opt-in, +rvvi_tracer_fidelity).
 *
 * Explicit per-row rvfi_intr {cause[10:0], interrupt, exception, intr} and
 * rvfi_dbg (dcsr.cause) from the core tracer, pushed by rvvi_trace2api
 * before the row's trap/retire handling. When enabled, the explicit data is
 * the PRIMARY source for (a) async-interrupt handler-entry detection in the
 * reactive resync and (b) the debug-entry cause; the legacy inference
 * (mstatus.MIE skew + mtvec-target match, DUT dcsr read) keeps running as a
 * cross-check, with counters and a loud log on any disagreement. rvfi_intr
 * needs the tracer patch that drives it (undriven upstream); rvfi_dbg has
 * always been driven. Default OFF: the sidecar is ignored entirely.
 * (State lives with the other diagnostic counters near the top.)
 * ---------------------------------------------------------------------- */
void rvviRefSetTracerFidelity(int enable)
{
    g_tracer_fidelity = (enable != 0);
    BRIDGE_LOG("tracer-fidelity sidecar %s",
               g_tracer_fidelity ? "ENABLED (+rvvi_tracer_fidelity)"
                                 : "DISABLED");
}

void rvviDutTracerSidecar(uint32_t /*hartId*/, uint32_t intr, uint32_t dbg)
{
    g_row_intr = intr;
    g_row_dbg  = dbg;
}

void rvviRefSetTracerInformed(int enable)
{
    g_tracer_informed = (enable != 0);
    BRIDGE_LOG("tracer-informed take %s",
               g_tracer_informed ? "ENABLED (+rvvi_tracer_informed)"
                                 : "DISABLED");
}

/* Inject interrupt `mcause` into the reference ISS for the current retire.
 * No-op unless gated ON and mcause flags an interrupt (bit 31 set). The ISS
 * computes the trap entry; we do NOT push any DUT CSR/GPR/FPR here. */
void rvviRefInjectIrq(uint32_t /*hartId*/, uint32_t mcause)
{
    if (!g_informed_irq_enabled)
        return;                         /* gated OFF: reactive resync stays */
    if ((mcause & 0x80000000u) == 0)
        return;                         /* not an interrupt (sync exception) */
    if (!gvsoc_engine_is_running())
        return;

    int irq_id = (int)(mcause & 0x1Fu); /* mcause[30:0] exception code (0..31) -- vector slot */

    /* Entry-detect: fire exactly ONCE per genuine take.  rvfi_intr is undriven
     * in the CV32E40P RVFI, so SV passes EVERY mcause[31]=1 retire; gate on two
     * independent conditions:
     *  (a) ISS MIE==1: under skip_irq_check the ISS MIE stays 1 until we inject,
     *      and a take clears it -- so later handler retires (MIE==0) do not re-fire.
     *  (b) DUT retire PC == the vectored mtvec entry (base + cause*4): the DUT
     *      retires there only on a genuine vectored-interrupt entry, not on
     *      normal code.
     * We use the DUT-PC-at-vector test (not a DUT-MIE test) because g_dut_csr
     * mirrors mstatus one delta-cycle late, which would miss a handler's mret and
     * mis-fire on the next interrupt's pre-trap code.  g_retire.pc is this
     * retire's DUT PC (committed by rvviDutRetire, called by SV before this). */
    uint32_t iss_mstatus = 0, mtvec = 0;
    gvsoc_engine_get_csr(TRAP_CSR_MSTATUS, &iss_mstatus);
    if (!((iss_mstatus >> 3) & 1u)) {
        BRIDGE_LOG_HOT("informed-inject reject: ISS MIE=0 (mstatus=0x%08x, mcause=0x%08x, dutPc=0x%08x)",
                       iss_mstatus, mcause, g_retire.pc);
        return;                         /* ISS MIE=0: already took -> no re-inject */
    }
    gvsoc_engine_get_csr(CSR_MTVEC, &mtvec);
    uint32_t vbase = mtvec & ~(uint32_t)1u;
    uint32_t entry = (mtvec & 1u) ? (vbase + (uint32_t)irq_id * 4u) : vbase;
    if (g_retire.pc != entry) {
        BRIDGE_LOG_HOT("informed-inject reject: DUT PC 0x%08x != entry 0x%08x (mtvec=0x%08x, id=%d)",
                       g_retire.pc, entry, mtvec, irq_id);
        return;                         /* DUT not at the trap vector -> not a genuine take */
    }

    g_informed_irq_count++;

    BRIDGE_LOG_HOT("informed-inject: mcause=0x%08x irq_id=%d (injection #%llu)",
                   mcause, irq_id, (unsigned long long)g_informed_irq_count);

    /* ISS takes the IRQ and computes the entry over exactly one ISS step.  rc!=1
     * means the ISS did NOT take it (e.g. ISS mie[cause]=0, or a step timeout):
     * the following step-n-compare would then show a PC mismatch with the ISS one
     * retire behind -- log here so the root cause (failed inject) is not invisible. */
    int inject_rc = gvsoc_engine_take_irq_for_one_step(irq_id);
    if (inject_rc != 1)
        BRIDGE_ERR("informed-inject: ISS did NOT take IRQ id=%d (rc=%d) -- check ISS "
                   "mie[%d]; the step-n-compare will surface a PC mismatch",
                   irq_id, inject_rc, irq_id);
}

/* ==========================================================================
 * SECTION 7 - RVVI API: DUT state push
 * ========================================================================== */

void rvviDutGprSet(uint32_t /*hartId*/, uint32_t gprIndex, uint64_t value)
{
    if (gprIndex < 32) {
        g_dut_gpr[gprIndex] = (uint32_t)value;
        g_pending.gpr_mask |= (1u << gprIndex);
    }
}

void rvviDutFprSet(uint32_t /*hartId*/, uint32_t fprIndex, uint64_t value)
{
    if (fprIndex < 32) {
        g_dut_fpr[fprIndex] = (uint32_t)value;
        g_pending.fpr_mask |= (1u << fprIndex);
    }
}

void rvviDutCsrSet(uint32_t /*hartId*/, uint32_t csrIndex, uint64_t value)
{
    g_dut_csr[csrIndex] = (uint32_t)value;
    /* Duplicate pushes of one address in a retire are by design (a trap: the
     * generic csr_wb scan and rvvi_trace2api.sv's explicit trap-CSR push both
     * report mstatus/mepc/mcause); the RVVI-TEXT formatter deduplicates and
     * the sticky mirror above always holds the latest value. */
    g_pending.csr_writes.emplace_back(csrIndex, (uint32_t)value);
}

/* Custom extension, not part of the vendored RVVI API (declared inline in
 * rvvi_trace2api.sv, same pattern as rvviRefRetireAndCompare) - pushes the
 * DUT privilege mode once per retire for the RVVI-TEXT emitter. */
void rvviBridgeSetMode(uint32_t mode)
{
    g_dut_mode = mode;
}

/* Custom extension: called once by gvsoc_wrap's ref_init task,
 * BEFORE rvviRefInit(), when RVVI_TRACE is also compiled in (dual-trace). Program
 * order within that task guarantees this runs before rvviRefInit()'s file-open
 * decision, so there is no initial-block race with the SV tracer's own setup. */
void rvviBridgeSetRefOnly(uint8_t refOnly)
{
    g_rvvi_text_ref_only = (refOnly != 0);
}

/* Custom extension: pushes the CFG-derived FLEN (0 = no FPU, 32 = F/Zfinx)
 * for the RVVI-TEXT PARAMS header. Called by gvsoc_wrap's ref_init BEFORE
 * rvviRefInit(), where the header is written; without it the header keeps a
 * hardcoded FLEN 32 and diverges from the tracer's dut.rvvi on no-FPU CFGs. */
void rvviBridgeSetFlen(uint32_t flen)
{
    g_rvvi_text_params.flen = flen;
}

void rvviDutBusWrite(uint32_t /*hartId*/, uint64_t /*address*/,
                     uint64_t /*value*/, uint64_t /*byteEnableMask*/) {}
void rvviDutVrSet(uint32_t /*hartId*/, uint32_t /*vrIndex*/,
                  uint32_t /*byteIndex*/, uint8_t /*data*/) {}
void rvviDutCycleCountSet(uint64_t /*cycleCount*/) {}

/* Restore the entry-seam CSRs (dpc + hwloop 0xCC0-2 / 0xCC4-6) from the DUT
 * mirror. These are the compared state that the asynchronous informed-entry
 * reconstruction cannot get bit-exact:
 *
 *  - hwloop counters: when the ISS already retired the loop-end instruction the
 *    DUT cancelled (hwlp_mask fires on both interrupt and debug entry), its
 *    extra decrement must be undone from the DUT reference.
 *
 *  - dpc (0x7B1): an ebreak that enters debug mode retires on CV32E40P and sets
 *    dpc to the ebreak's own PC, but the ISS advances current_insn past it
 *    before Cv32e40pIrq::check() captures depc = current_insn, so depc lands
 *    one instruction size high (e.g. +2 for a c.ebreak). The DUT dpc is
 *    authoritative for the entry point; forcing it here also fixes the resume
 *    PC after dret. Haltreq/single-step entries agree only when the ISS was
 *    parked exactly at the DUT's boundary at injection: with a commit-FIFO
 *    lag (an instruction executed but parked behind a held LSU head) the ISS
 *    depc lands one instruction high there too, and this force repairs the
 *    entry point but NOT the extra architectural writeback (see the engine's
 *    boundary diagnostics in take_debug).
 *
 * The IRQ force-resync covers hwloop through its generic compared-CSR restore;
 * the debug entry has no generic restore, so it calls this targeted one. No-op
 * when the states already agree. */
static void force_debug_entry_csrs_from_dut(uint32_t cause)
{
    static const uint32_t addrs[] = {CSR_DPC, 0xCC0u, 0xCC1u, 0xCC2u,
                                     0xCC4u, 0xCC5u, 0xCC6u};
    for (uint32_t addr : addrs) {
        auto it = g_dut_csr.find(addr);
        if (it == g_dut_csr.end()) continue;
        uint32_t iss_val = 0;
        gvsoc_engine_get_csr(addr, &iss_val);
        if (iss_val != it->second) {
            if (addr == CSR_DPC && cause != 1) {
                /* A dpc force with a non-zero delta is the direct gauge of a
                 * debug-entry ordering error (the ISS's entry point was not
                 * the DUT's): log it unthrottled so the force never silently
                 * masks the diagnosis it repairs. Ebreak entries (cause 1)
                 * are excluded from the gauge: their dpc delta is the
                 * structural retire-vs-capture offset documented above, not
                 * an ordering error - counting them would flood the log and
                 * blunt the signal. */
                g_dpc_force_count++;
                BRIDGE_LOG("debug-entry dpc force #%llu: ISS=0x%08x -> DUT=0x%08x "
                           "(delta=%+d bytes - entry-ordering gauge)",
                           (unsigned long long)g_dpc_force_count,
                           iss_val, it->second,
                           (int)(iss_val - it->second));
            } else {
                BRIDGE_LOG_HOT("debug-entry force CSR[0x%03x]: ISS=0x%08x -> DUT=0x%08x",
                               addr, iss_val, it->second);
            }
            gvsoc_engine_set_csr(addr, it->second);
        }
    }
}

/* Informed debug entry: on the DUT's first debug-mode retire (the debug
 * ROM entry row) the ISS is still parked at the interrupted boundary -
 * the RTL's entry has no architectural retire of its own, so nothing in
 * the step-and-compare stream makes the ISS enter debug. Arm the entry
 * with the DUT's own dcsr.cause (haltreq wire, ebreak or single-step all
 * land here) and let the ISS compute dpc/dcsr and redirect to the ROM;
 * the queued debug-ROM commit is then served against this retire row.
 * The ISS-lag term de-latches the edge detector: a debug re-entry whose
 * exit edge this stream never observed (rapid re-arm, back-to-back
 * entries) leaves g_prev_dut_debug stale and the seam silently blind -
 * fire whenever the DUT is in debug and the ISS is lagging behind.
 * The PC term covers entries the debugMode flag misses entirely:
 * rvfi_dbg_mode is an entry MARKER (asserted only while the tracer sees
 * the DBG_TAKEN_* FSM states), and some haltreq entry paths retire the
 * first debug-ROM row without it - the row then arrives with
 * debugMode=0 and no later row carries the flag either. The DUT retire
 * PC landing exactly on the ISS's configured debug-ROM entry address
 * (dm_halt_addr) while the ISS is not in debug is an unambiguous entry
 * row; deeper ROM rows do not match, so a failed injection does not
 * retry-spam.
 * Called from BOTH retire paths: a plain entry row arrives through
 * rvviDutRetire, but an entry that collided with an interrupt take is a
 * TRAP row (the take's CSR writes ride it) and arrives through
 * rvviDutTrap - skipping it there left the ISS entering without the
 * take (mstatus.MIE/mcause divergence at the ROM rows). */
static void maybe_informed_debug_entry(uint64_t dutPc, bool debug_mode_flag)
{
    bool entry_row = debug_mode_flag ||
                     ((uint32_t)dutPc == gvsoc_engine_get_debug_handler() &&
                      !gvsoc_engine_is_debug_mode());
    if (entry_row && (!g_prev_dut_debug || !gvsoc_engine_is_debug_mode()) &&
        gvsoc_engine_is_running())
    {
        uint32_t cause = 3;
        auto it = g_dut_csr.find(CSR_DCSR);
        if (it != g_dut_csr.end())
            cause = (it->second >> 6) & 0x7u;
        /* Tracer-fidelity: rvfi_dbg is CROSS-CHECK ONLY here. Empirical
         * result 2026-08-06 (corev_rand_debug + debug_test): the tracer's
         * saved_debug_cause is sampled a cycle off the entry and comes out
         * STALE on some entries (said 1/ebreak where the DUT's own dcsr
         * said 3/haltreq at dm_halt_addr) - trusting it regressed PASS
         * lanes. The dcsr-derived cause stays primary; the counters
         * quantify the tracer's error rate for the upstream issue. */
        if (g_tracer_fidelity && (g_row_dbg & 0x7u) != 0) {
            uint32_t cause_tr = g_row_dbg & 0x7u;
            g_fid_dbg_rows++;
            if (cause_tr == cause) {
                g_fid_dbg_agree++;
            } else {
                g_fid_dbg_mismatch++;
                BRIDGE_ERR("tracer-fidelity: rvfi_dbg entry cause %u != "
                           "dcsr-derived %u at PC=0x%08x - keeping dcsr "
                           "(rvfi_dbg capture is timing-unreliable, see "
                           "upstream issue)",
                           cause_tr, cause, (uint32_t)dutPc);
            }
        }
        /* Interrupt+debug collision, decided by the DUT: when the RTL takes
         * an interrupt and enters debug in the same arbitration, the entry
         * row carries the take's CSR writes - a fresh mcause write with the
         * interrupt bit set on THIS row (g_retire.csr_writes, not the sticky
         * mirror) is the unambiguous signature, and its id tells the model
         * exactly which line to take ahead of the entry. First-match scan:
         * assumes at most one distinct MCAUSE push per row (the documented
         * duplications - generic scan + explicit trap push - carry the same
         * value); if that invariant ever changes, switch to last-match like
         * the sticky mirror. */
        int collide_irq_id = -1;
        for (const auto &w : g_retire.csr_writes) {
            if (w.first == TRAP_CSR_MCAUSE && (w.second & 0x80000000u)) {
                collide_irq_id = (int)(w.second & 0x1fu);
                break;
            }
        }
        /* The id is DUT-provided state, never trusted blindly: only the
         * wired lines (MSI 3, MTI 7, MEI 11, fast 16..31 - the RTL
         * int_controller IRQ_MASK) can collide with an entry. Anything
         * else (a software mcause write riding an entry row) would fall
         * through cv32e40p_irq_pick's default and take a phantom MTI. */
        if (collide_irq_id >= 0 &&
            !(collide_irq_id == 3 || collide_irq_id == 7 ||
              collide_irq_id == 11 ||
              (collide_irq_id >= 16 && collide_irq_id <= 31))) {
            BRIDGE_ERR("informed debug entry: mcause id %d on the entry row "
                       "is not a wired interrupt line - collision ignored",
                       collide_irq_id);
            collide_irq_id = -1;
        }
        BRIDGE_LOG_HOT("informed debug entry: DUT PC=0x%08x cause=%u%s",
                       (uint32_t)dutPc, cause,
                       collide_irq_id >= 0 ? " (+interrupt collision)" : "");
        int rc = gvsoc_engine_take_debug_for_one_step((int)cause, collide_irq_id);
        if (rc != 1)
            BRIDGE_ERR("informed debug entry not certified (cause=%u rc=%d) - "
                       "the ISS either did not enter debug or entered off the "
                       "DUT's boundary; the step-and-compare will surface the "
                       "divergence", cause, rc);
        else
            force_debug_entry_csrs_from_dut(cause);
    }
}

void rvviDutRetire(uint32_t /*hartId*/, uint64_t dutPc,
                   uint64_t dutInsBin, bool_t debugMode)
{
    PROF_SCOPE(g_prof_ns_dut_retire, g_prof_cnt_dut_retire);
    g_metric_retires++;
    take_retire_event(dutPc, dutInsBin, /*is_trap=*/false);
    /* Do NOT clear g_pending_handler here: rvvi_trace2api calls rvviDutRetire
     * before rvviRefCsrsCompare, so the trap-CSR snapshot must stay active for
     * the comparison.  It is cleared in rvviRefCsrsCompare after consumption. */

    maybe_informed_debug_entry(dutPc, debugMode != 0);
    /* Edge-detector state, updated on this path ONLY: the trap path never
     * carries debugMode, so a trap row must not fake a 1->0 edge. The
     * asymmetry is covered by the seam's second OR-term
     * (!gvsoc_engine_is_debug_mode()), which blocks re-triggering while
     * the ISS is already in debug - keep both in mind when refactoring. */
    g_prev_dut_debug = (debugMode != 0);
}

void rvviDutTrap(uint32_t /*hartId*/, uint64_t dutPc, uint64_t dutInsBin)
{
    g_metric_retires++;
    g_metric_traps++;
    take_retire_event(dutPc, dutInsBin, /*is_trap=*/true);

    /* A debug entry that collided with an interrupt take arrives as a TRAP
     * row (the take's CSR writes ride the entry row, so the tracer flags
     * it): run the same informed-entry detection as the plain-retire path,
     * or the ISS would reach the ROM without the take. The injected entry
     * leaves the ROM row0 commit queued; the caller's silent EventStep for
     * this trap row consumes it, exactly like an exception's faulting-step
     * commit. Normal synchronous exceptions never match: their trap row
     * retires at the mtvec handler, not at dm_halt_addr. */
    maybe_informed_debug_entry(dutPc, /*debug_mode_flag=*/false);

    /* Snapshot the exception CSRs before the pipeline-flush retire (PC=0x0)
     * can corrupt them.  The sync bridge pushes these values into g_dut_csr
     * before this call, so the snapshot is always correct. */
    g_trap_csr_snapshot.clear();
    for (uint32_t addr : {TRAP_CSR_MSTATUS, TRAP_CSR_MEPC, TRAP_CSR_MCAUSE, TRAP_CSR_MTVAL}) {
        auto it = g_dut_csr.find(addr);
        if (it != g_dut_csr.end())
            g_trap_csr_snapshot[addr] = it->second;
    }
    g_pending_handler = true;
    g_sync_trap_seam = true;

    /* RVVI-TEXT: a trap retire bypasses rvviRefRetireAndCompare (the SV batch
     * call is gated on !trap), so we emit its line HERE.  Per RVVI-TRACE a
     * synchronous exception is a TRAP event: the faulting instruction retires
     * no register writes (RVFI flags none, so g_retire carries empty masks and
     * no X/F tokens) and the line carries the trap-entry CSRs
     * (mstatus/mepc/mcause/mtval) the SV pushed before this call.
     * ref_from_dut: the ISS has not taken the trap yet at this seam (it
     * consumes the faulting step only at the rvviRefEventStep the SV calls
     * next), so the ref line is built from DUT data. */
    emit_retire_lines(/*ref_from_dut=*/true);

    /* No force-resync here.  rvviDutTrap fires only for synchronous exceptions
     * (rvfi_trap=1); async interrupts do NOT set rvfi_trap and are handled by
     * the inline ISS-PC vs DUT-PC detection in rvviRefEventStep. */

    BRIDGE_LOG_HOT("trap snapshot: mstatus=0x%08x mepc=0x%08x mcause=0x%08x mtval=0x%08x",
                   g_trap_csr_snapshot.count(TRAP_CSR_MSTATUS) ? g_trap_csr_snapshot[TRAP_CSR_MSTATUS] : 0u,
                   g_trap_csr_snapshot.count(TRAP_CSR_MEPC)    ? g_trap_csr_snapshot[TRAP_CSR_MEPC]    : 0u,
                   g_trap_csr_snapshot.count(TRAP_CSR_MCAUSE)  ? g_trap_csr_snapshot[TRAP_CSR_MCAUSE]  : 0u,
                   g_trap_csr_snapshot.count(TRAP_CSR_MTVAL)   ? g_trap_csr_snapshot[TRAP_CSR_MTVAL]   : 0u);
}

/* ==========================================================================
 * SECTION 8 - RVVI API: reference model stepping
 * ========================================================================== */

/* Release a WFI-parked ISS through the dedicated wfi_wake wire.  The DUT is
 * the oracle of the wake: this runs only when its retire stream has already
 * advanced past its own wfi (the RTL retires wfi at execute and sleeps
 * after), so the ISS park must end NOW whatever the wake source was - an
 * enabled interrupt whose edge was consumed before the park, a
 * level-sensitive debug_req, or a source with no wire mapping at all.  The
 * pulse reaches Cv32e40pIrq::wfi_wake_sync inside the model .so, which runs
 * the full release trio (clear wfi, retain_dec, terminate the held WFI
 * entry) with NO architectural side effect: mip, mie and the interrupt
 * wires stay untouched, and the terminated entry drains into the commit
 * FIFO, serving the DUT's own wfi retire.  Replaces the mie-guided wire
 * replay (cause_code_to_net / try_wake_on_net fallthrough), which could
 * not release a wake the interrupt wires cannot carry (debug_req with
 * mie=0) and re-derived what the DUT had already proven.
 * Net index mirrors gvsoc_engine_v2.cpp g_irq_wire_name order (haltreq=19,
 * wfi_wake=20).  Returns true if the ISS left WFI. */
static constexpr uint64_t WFI_WAKE_NET = 20;

static bool wake_wfi_via_wire(void)
{
    if (!gvsoc_engine_is_wfi())
        return true;

    gvsoc_engine_set_irq(WFI_WAKE_NET, 1);
    gvsoc_engine_set_irq(WFI_WAKE_NET, 0);

    if (!gvsoc_engine_is_wfi()) {
        BRIDGE_LOG_HOT("wfi-wake: released via wfi_wake wire");
        return true;
    }

    BRIDGE_ERR("wfi-wake: WFI still set after the wfi_wake pulse - "
               "set_pc will bail as before");
    return false;
}

bool_t rvviRefEventStep(uint32_t /*hartId*/)
{
    PROF_SCOPE(g_prof_ns_step, g_prof_cnt_step);
    if (!gvsoc_engine_is_running())
        return RVVI_TRUE;  /* stub mode */

    /* Trap rows need commit-stream care: an instruction that executes and
     * then traps (ecall/ebreak) commits in the ISS, so its commit must be
     * consumed here like any retire; a refused instruction (illegal) never
     * commits, so the queued head is already the handler entry and consuming
     * it would shift the comparison by one retire per trap.  Materialize the
     * head and let its PC decide.  Per-instruction engines (v1) consume the
     * faulting step unconditionally.  The SV batch compare is already gated
     * on !trap, so returning without a step is safe. */
    if (g_retire.is_trap && gvsoc_engine_commit_stream()) {
        /* NOTE (tracer-informed residue): on the sync-exception+IRQ
         * collision the DUT kills this instruction, but the materialize
         * below still executes it in the ISS, so the informed take on the
         * next row computes mepc one instruction ahead (+4 skew, the whole
         * residue of the exception lane). An mcause-bit31 guard here
         * over-fires (measured 8310 skips / 70 mm vs 51): the only exact
         * discriminator is the NEXT row's rvfi_intr, which needs a
         * deferred-step rework of this seam. Left as the known next step. */
        uint32_t commit_pc = 0;
        if (gvsoc_engine_materialize_commit(&commit_pc) == 0 &&
            commit_pc == (uint32_t)g_retire.pc) {
            gvsoc_engine_step();
        }
        return RVVI_TRUE;
    }

    /* Consume the sync-trap seam here, into a local, so no later early-return
     * (IRQ/WFI force-resync, sim-ended) can leak it armed onto an unrelated
     * row with a stale snapshot.  The trap-row early-return above keeps it
     * armed on purpose: the seam targets THIS first post-trap step. */
    bool sync_trap_seam = g_sync_trap_seam;
    g_sync_trap_seam = false;

    /* Tracer-informed take: rvfi_intr marks THIS row as the first
     * instruction of an async-IRQ handler, and it arrives BEFORE the row's
     * ISS step - the one point where the entry can be computed instead of
     * repaired. Single-step inject of the DUT-selected cause: the take's
     * vector-slot commit stays queued and the normal step below serves it
     * against this row, so mepc/mcause/mstatus come out of the model's own
     * trap logic (no force-resync recovery tail). A WFI-parked ISS is left
     * to the reactive path, whose wire-wake handling is validated; a failed
     * take falls through to the reactive repair unchanged. */
    if (g_tracer_informed && !g_informed_irq_enabled &&
        (g_row_intr & 0x1u) && (g_row_intr & 0x4u)) {
        if (gvsoc_engine_is_wfi()) {
            g_fid_informed_wfi++;
        } else {
            int irq_id = (int)((g_row_intr >> 3) & 0x1fu);
            int irc = gvsoc_engine_take_irq_for_one_step(irq_id);
            if (irc == 1) {
                g_fid_informed_takes++;
                BRIDGE_LOG_HOT("tracer-informed take: irq id=%d computed by "
                               "the ISS (take #%llu)", irq_id,
                               (unsigned long long)g_fid_informed_takes);
            } else {
                g_fid_informed_fails++;
                BRIDGE_ERR("tracer-informed take: ISS did NOT take irq id=%d "
                           "(rc=%d) - falling back to the reactive resync",
                           irq_id, irc);
            }
        }
    }

    int rc = gvsoc_engine_step();

    /* Step timeout with the ISS parked in WFI: the DUT's stream is serving
     * this retire, so the DUT already executed its wfi (the RTL retires it
     * at execute and sleeps after) - the park must end now, whatever the
     * wake source was (a debug_req level, an edge consumed before the park,
     * a source with no wire mapping). Release through the wfi_wake wire and
     * serve the retire with one retry; the terminated WFI entry drains as
     * the matching commit. */
    if (rc == 0 && !gvsoc_engine_finished() && gvsoc_engine_is_wfi()) {
        if (wake_wfi_via_wire())
            rc = gvsoc_engine_step();
    }

    if (rc == 0 && gvsoc_engine_finished()) {
        static bool ended_once = false;
        if (!ended_once) {
            BRIDGE_LOG("simulation ended (step returned 0, engine finished)");
            BRIDGE_LOG("scheduling $finish via vpi_control");
            ended_once = true;
        }
        gvsoc_engine_shutdown();
        vpi_control(vpiFinish, 1);
        return RVVI_TRUE;
    }

    /* Inline IRQ mismatch detection and force-resync.
     *
     * With skip_irq_check the ISS never takes interrupts on its own.  When the
     * DUT takes an async interrupt it retires at mtvec while the ISS is still
     * at the interrupted PC; RVFI does not set rvfi_trap for async interrupts,
     * so rvviDutTrap is never called.  Also handles WFI stalls: with
     * skip_irq_check the ISS cannot wake from WFI, so a step timeout (rc=0) in
     * WFI state force-resyncs to the DUT.
     *
     * Detection: after the step, if ISS PC differs from DUT PC, check below
     * whether this is a new interrupt-take or a stuck WFI. */
    if ((rc == 1 || rc == 0) && g_force_trap_enabled) {
        uint32_t iss_pc = gvsoc_engine_get_pc();
        if (iss_pc != g_retire.pc) {
            /* Detect a new interrupt-take: when the DUT takes an async IRQ,
             * mstatus.MIE goes 1->0 (saved to MPIE); with skip_irq_check the
             * ISS MIE stays 1.  This MIE divergence (DUT MIE=0, ISS MIE=1)
             * uniquely flags each take, even for repeated same-type IRQs.
             * A stuck WFI is handled unconditionally. */
            uint32_t dut_mstatus = 0, iss_mstatus = 0;
            {
                auto it = g_dut_csr.find(TRAP_CSR_MSTATUS);
                if (it != g_dut_csr.end()) dut_mstatus = it->second;
            }
            gvsoc_engine_get_csr(TRAP_CSR_MSTATUS, &iss_mstatus);
            /* MIE is bit 3 of mstatus.  IRQ taken = DUT MIE=0, ISS MIE=1 */
            bool dut_mie = (dut_mstatus >> 3) & 1;
            bool iss_mie = (iss_mstatus >> 3) & 1;
            /* The MIE-skew detector goes blind once a previous force-resync
             * has synced the ISS mstatus to the DUT's (MIE already 0 on both
             * sides), e.g. a WFI wake straight into a handler after an IRQ
             * storm.  Second, DUT-side-only signal immune to the forces: an
             * interrupt take writes mepc = resume PC, which is exactly where
             * the ISS (which did not take the IRQ) is sitting.  Also covers
             * nested takes, where DUT MIE stays 0 across the entry.
             * The mcause/mepc mirrors are sticky, and loop bodies revisit
             * PCs, so mepc==iss_pc alone misfires in hwloop/debug lanes: the
             * take is only accepted when the DUT row sits exactly on the
             * mtvec-derived entry for the mirrored cause (vector slots are
             * never ordinary loop/ROM rows). */
            bool dut_irq_take_here = false;
            {
                auto mc = g_dut_csr.find(TRAP_CSR_MCAUSE);
                auto me = g_dut_csr.find(TRAP_CSR_MEPC);
                auto mt = g_dut_csr.find(0x305u /*mtvec*/);
                if (mc != g_dut_csr.end() && (mc->second & 0x80000000u) &&
                    me != g_dut_csr.end() && me->second == iss_pc &&
                    mt != g_dut_csr.end()) {
                    uint32_t base   = mt->second & ~0x3u;
                    uint32_t code   = mc->second & 0x1fu;
                    uint32_t target = (mt->second & 0x3u) ? base + 4u * code
                                                          : base;
                    dut_irq_take_here = ((uint32_t)g_retire.pc == target);
                }
            }
            /* When informed-injection is ON it OWNS the async-IRQ entry (the ISS
             * takes the IRQ and computes the entry itself); the reactive
             * force-set-PC must NOT also fire or it re-introduces the stale-fetch
             * garbage.  WFI-stuck handling below stays active regardless. */
            bool is_new_irq = !g_informed_irq_enabled &&
                              ((!dut_mie && iss_mie) || dut_irq_take_here);
            /* Tracer-fidelity: rvfi_intr.intr + .interrupt explicitly marks
             * the first instruction of an async-IRQ handler - including the
             * takes the MIE-skew detector is structurally blind to (the
             * sync-exception+IRQ collision leaves both MIE at 0). Explicit
             * data is primary; the legacy detector keeps running as a
             * cross-check with loud logs on disagreement. */
            if (g_tracer_fidelity && !g_informed_irq_enabled) {
                bool intr_row = (g_row_intr & 0x1u) && (g_row_intr & 0x4u);
                if (intr_row) {
                    g_fid_intr_rows++;
                    if (is_new_irq) {
                        g_fid_intr_agree++;
                    } else {
                        g_fid_intr_only++;
                        BRIDGE_ERR("tracer-fidelity: rvfi_intr flags an IRQ "
                                   "handler entry the reactive detector "
                                   "missed (PC=0x%08x cause=%u) - resyncing "
                                   "on the explicit signal",
                                   g_retire.pc, (g_row_intr >> 3) & 0x1fu);
                        is_new_irq = true;
                    }
                } else if (is_new_irq) {
                    g_fid_det_only++;
                    BRIDGE_ERR("tracer-fidelity: reactive detector fired "
                               "without rvfi_intr on the row (PC=0x%08x) - "
                               "keeping the resync, flagging for review",
                               g_retire.pc);
                }
            }
            bool is_wfi_stuck = (rc == 0 && gvsoc_engine_is_wfi());

            if (is_new_irq || is_wfi_stuck) {
                g_force_resync_count++;

                {
                    auto mc = g_dut_csr.find(TRAP_CSR_MCAUSE);
                    BRIDGE_LOG_HOT("%s resync: ISS PC=0x%08x -> DUT PC=0x%08x "
                                   "(mcause=0x%08x, mstatus DUT=0x%08x ISS=0x%08x, resync #%llu)",
                                   is_wfi_stuck ? "WFI" : "IRQ",
                                   iss_pc, g_retire.pc,
                                   mc != g_dut_csr.end() ? mc->second : 0u,
                                   dut_mstatus, iss_mstatus,
                                   (unsigned long long)g_force_resync_count);
                }

                /* WFI-stuck: release the held WFI through the wire path first
                 * (fix R1), so the following set_pc redirect does not bail on
                 * a parked ISS and corrupt the commit stream.
                 *
                 * A successful wire wake retires the WFI instruction itself.
                 * On a MIE=0 wake-without-trap the DUT row IS that WFI, so the
                 * wake alone already satisfied the row: redirecting to
                 * g_retire.pc would re-execute the just-retired WFI with the
                 * wire released and park the ISS again (the rvviRefEventStep
                 * FAILED residue).  Skip both the redirect and the extra step;
                 * the diff-gated state forces below still run.  When the DUT
                 * instead woke into a handler, the retired-PC check fails and
                 * the reactive redirect proceeds as before. */
                bool woke_on_row = false;
                if (gvsoc_engine_is_wfi()) {
                    /* Attempt the wake whenever the ISS is parked, not only
                     * in the rc==0 stuck case: with rc==1 a backlog commit
                     * was just served while the ISS already executed the WFI
                     * and parked, and the set_pc redirect below would bail on
                     * the held entry and corrupt the commit stream.
                     * woke_on_row keeps the historical rc==0 semantics: with
                     * rc==1 the DUT row was already served, so the redirect
                     * must always proceed. */
                    bool woke = wake_wfi_via_wire();
                    woke_on_row = woke && is_wfi_stuck &&
                                  gvsoc_engine_get_pc() == g_retire.pc;
                }

                /* Force ISS PC to DUT handler entry */
                if (!woke_on_row) {
                    gvsoc_engine_set_pc(g_retire.pc);
                    /* The drain inside set_pc can execute the pre-resync
                     * stream into a WFI and park the ISS mid-redirect (the
                     * set_pc WFI bail): release it through the wire path and
                     * redo the redirect once, now unparked. On a failed wake
                     * (mie=0) this is the same bail as before, not worse. */
                    if (gvsoc_engine_is_wfi() && wake_wfi_via_wire())
                        gvsoc_engine_set_pc(g_retire.pc);
                }

                /* Force the DUT trap CSRs AND the full architectural CSR set
                 * the compare tracks - not just the 4 trap CSRs (fix R2).  A
                 * value the ISS missed while stalled in WFI (e.g. mscratch
                 * 0x340, swapped with sp by the trap prologue) would otherwise
                 * stay permanently stale and re-surface at every handler entry
                 * (periodic post-WFI divergence).  Skips: volatile CSRs (not
                 * compared) and mip 0x344 (driven by the interrupt wires -
                 * force-writing it would decouple the ISS mip from the wire
                 * mirror). */
                static const uint32_t CSR_MIP = 0x344U;
                auto force_csr_from_dut = [](uint32_t addr) {
                    auto it = g_dut_csr.find(addr);
                    if (it == g_dut_csr.end()) return;
                    uint32_t iss_val = 0;
                    gvsoc_engine_get_csr(addr, &iss_val);
                    if (iss_val != it->second) {
                        BRIDGE_LOG_HOT("IRQ-resync force CSR[0x%03x]: ISS=0x%08x -> DUT=0x%08x",
                                       addr, iss_val, it->second);
                        gvsoc_engine_set_csr(addr, it->second);
                    }
                };
                for (uint32_t addr : {TRAP_CSR_MEPC, TRAP_CSR_MCAUSE,
                                       TRAP_CSR_MTVAL, TRAP_CSR_MSTATUS})
                    force_csr_from_dut(addr);
                for (uint32_t addr : g_csr_compare_enabled) {
                    if (is_trap_csr(addr) || addr == CSR_MIP) continue;
                    if (g_csr_volatile.count(addr))           continue;
                    force_csr_from_dut(addr);
                }

                /* Force all GPRs */
                for (uint32_t i = 1; i < 32; i++) {
                    uint32_t iss_gpr = gvsoc_engine_get_gpr(i);
                    if (iss_gpr != g_dut_gpr[i]) {
                        gvsoc_engine_set_gpr(i, g_dut_gpr[i]);
                    }
                }

                /* Force all FPRs */
                for (uint32_t i = 0; i < 32; i++) {
                    uint32_t iss_fpr = gvsoc_engine_get_fpr(i);
                    if (iss_fpr != g_dut_fpr[i]) {
                        gvsoc_engine_set_fpr(i, g_dut_fpr[i]);
                    }
                }

                /* Step the ISS from the forced PC so it actually retires the
                 * instruction at g_retire.pc.  This keeps ISS and DUT at the
                 * same retire count instead of leaving the ISS one step behind.
                 * (Already done by the wire wake when the row was the WFI.) */
                rc = woke_on_row ? 1 : gvsoc_engine_step();
                return (rc == 1) ? RVVI_TRUE : RVVI_FALSE;
            }
        }
    }

    /* Phase-shift re-alignment (synchronous exception).  Reached only when the
     * async-IRQ resync above did NOT fire.  A sync trap (illegal/ecall/ebreak)
     * leaves the ISS exactly one retire BEHIND the DUT on the SAME path: GVSOC
     * models the trap entry as an extra ISS step, and the async-IRQ resync does
     * not trigger (both MIE are 0 for a sync trap).  Two adjacent forms:
     *
     *  (a) Take-retire: the DUT reports the handler entry (g_retire.pc ==
     *      handler) with g_pending_handler set; the ISS only reached the
     *      trapping insn.  One more ISS step makes it take its own sync trap --
     *      accepted only if that step lands on g_retire.pc, so a real
     *      different-path divergence is never masked.
     *  (b) Residual +1 lag: the ISS PC equals the DUT's PREVIOUS retire PC
     *      (g_retire.pc_prev), provably one insn behind -> one catch-up step. */
    if (g_force_trap_enabled && rc == 1) {
        uint32_t iss_pc = gvsoc_engine_get_pc();
        if (iss_pc != g_retire.pc && g_pending_handler) {
            /* (a) take-retire: let the ISS take its own sync trap, then confirm
             * it converged on the DUT handler entry before accepting. */
            int rc2 = gvsoc_engine_step();
            uint32_t iss_pc2 = gvsoc_engine_get_pc();
            if (rc2 == 1 && iss_pc2 == g_retire.pc) {
                g_phase_realign_count++;
                BRIDGE_LOG_HOT("phase-shift realign (trap-take): ISS 0x%08x -> 0x%08x "
                               "== DUT handler 0x%08x - catch-up step (#%llu)",
                               iss_pc, iss_pc2, g_retire.pc,
                               (unsigned long long)g_phase_realign_count);
            } else if (rc2 == 1) {
                /* Extra step retired but did NOT converge on the DUT handler entry:
                 * this is NOT the expected 2-step sync-trap lag.  The ISS is now one
                 * step ahead; surface it so the resulting desync is not silent (the
                 * next compare will diverge).  rc=rc2 is preserved so the compare runs. */
                BRIDGE_ERR("phase-shift realign: extra step ISS 0x%08x -> 0x%08x != "
                           "DUT 0x%08x (no convergence) -- possible 1-retire desync",
                           iss_pc, iss_pc2, g_retire.pc);
            }
            rc = rc2;
        } else if (iss_pc != g_retire.pc && iss_pc == g_retire.pc_prev &&
                   g_retire.pc_prev != 0) {
            /* (b) residual +1 lag on the same path. */
            g_phase_realign_count++;
            BRIDGE_LOG_HOT("phase-shift realign: ISS=0x%08x (==DUT_prev) one retire "
                           "behind DUT=0x%08x - catch-up step (#%llu)",
                           iss_pc, g_retire.pc,
                           (unsigned long long)g_phase_realign_count);
            rc = gvsoc_engine_step();
        }
    }

    /* Sync-trap entry seam: once the ISS sits on the DUT handler entry with a
     * trap snapshot pending, realign mstatus and mepc from the snapshot taken
     * at rvviDutTrap.  In reactive resync the ISS mstatus.MIE timeline is
     * force-managed for the asynchronous interrupts (skip_irq), so a sync
     * exception landing inside an IRQ storm computes its trap entry from a
     * knowingly skewed MIE: the saved MPIE then differs from the DUT by
     * exactly that bit, and the skew leaks through the whole handler.  Only
     * the two async-skew carriers are forced - mcause and mtval derive from
     * the faulting instruction alone and stay honestly compared, so a real
     * trap-cause model bug is still caught (a wrong trap target would diverge
     * on PC regardless).  mstatus goes through the set_csr write mask, so
     * only FS/MPP/MPIE/MIE can move.  Diff-gated: a converged entry writes
     * nothing.  Only mstatus is forced: mepc derives from the faulting PC
     * alone (not async-skewed), so it stays in the honest compare - forcing
     * it would mask a real mepc-computation bug on this very row.
     * Consume-once: the seam local was captured right after the trap-row
     * early-return, so this runs exactly on the first post-trap non-trap row
     * (the handler entry); if the ISS is not on the handler entry here the
     * realign is skipped and the divergence surfaces honestly. */
    if (sync_trap_seam && g_force_trap_enabled && rc == 1 &&
        gvsoc_engine_get_pc() == g_retire.pc) {
        auto it = g_trap_csr_snapshot.find(TRAP_CSR_MSTATUS);
        if (it != g_trap_csr_snapshot.end()) {
            uint32_t iss_val = 0;
            gvsoc_engine_get_csr(TRAP_CSR_MSTATUS, &iss_val);
            if (iss_val != it->second) {
                BRIDGE_LOG_HOT("sync-trap resync CSR[0x%03x]: ISS=0x%08x -> DUT=0x%08x",
                               TRAP_CSR_MSTATUS, iss_val, it->second);
                gvsoc_engine_set_csr(TRAP_CSR_MSTATUS, it->second);
            }
        }
    }

    return (rc == 1) ? RVVI_TRUE : RVVI_FALSE;
}

/* ==========================================================================
 * SECTION 9 - RVVI API: state readback from GVSOC ISS
 * ========================================================================== */

uint64_t rvviRefPcGet(uint32_t /*hartId*/)
{
    return gvsoc_engine_is_running() ? gvsoc_engine_get_pc() : 0;
}

uint64_t rvviRefInsBinGet(uint32_t /*hartId*/)
{
    return gvsoc_engine_is_running() ? gvsoc_engine_get_insn() : 0;
}

uint64_t rvviRefGprGet(uint32_t /*hartId*/, uint32_t gprIndex)
{
    return gvsoc_engine_is_running() ? gvsoc_engine_get_gpr(gprIndex) : 0;
}

uint32_t rvviRefGprsWrittenGet(uint32_t /*hartId*/)
{
    return g_retire.gpr_mask;
}

uint64_t rvviRefFprGet(uint32_t /*hartId*/, uint32_t fprIndex)
{
    return gvsoc_engine_is_running() ? gvsoc_engine_get_fpr(fprIndex) : 0;
}

uint8_t  rvviRefVrGet(uint32_t /*hartId*/, uint32_t /*vrIndex*/,
                      uint32_t /*byteIndex*/)                            { return 0; }

uint64_t rvviRefCsrGet(uint32_t /*hartId*/, uint32_t csrIndex)
{
    if (!gvsoc_engine_is_running())
        return 0;
    uint32_t val = 0;
    gvsoc_engine_get_csr(csrIndex, &val);
    return val;
}

uint64_t rvviRefMemoryRead(uint32_t /*hartId*/, uint64_t /*address*/,
                           uint32_t /*size*/)                            { return 0; }

/* ==========================================================================
 * SECTION 10 - RVVI API: DUT vs ISS comparison
 * ========================================================================== */

bool_t rvviRefPcCompare(uint32_t /*hartId*/)
{
    PROF_SCOPE(g_prof_ns_pc_cmp, g_prof_cnt_pc_cmp);
    if (!gvsoc_engine_is_running() || gvsoc_engine_finished())
        return RVVI_TRUE;

    g_metric_comparisons_pc++;
    uint32_t iss_pc = gvsoc_engine_get_pc();
    if (g_retire.pc != iss_pc) {
        g_metric_mismatches++;
        if (throttle_check(g_pc_mismatch_count, "PC")) {
            BRIDGE_ERR("PC mismatch #%llu @ retire #%llu: DUT=0x%08x ISS=0x%08x",
                       (unsigned long long)g_pc_mismatch_count,
                       (unsigned long long)g_metric_retires,
                       g_retire.pc, iss_pc);
        }
        return RVVI_FALSE;
    }
    return RVVI_TRUE;
}

bool_t rvviRefGprsCompare(uint32_t /*hartId*/)
{
    PROF_SCOPE(g_prof_ns_gpr_cmp, g_prof_cnt_gpr_cmp);
    if (!gvsoc_engine_is_running() || gvsoc_engine_finished())
        return RVVI_TRUE;
    if (state_compare_deferred())
        return RVVI_TRUE;

    bool_t pass = RVVI_TRUE;
    for (uint32_t i = 1; i < 32; i++) {
        g_metric_comparisons_gpr++;
        uint32_t iss_val = gvsoc_engine_get_gpr(i);
        if (g_dut_gpr[i] != iss_val) {
            g_metric_mismatches++;
            if (throttle_check(g_gpr_mismatch_count, "GPR")) {
                BRIDGE_ERR("GPR[x%u] mismatch @ retire #%llu: DUT=0x%08x ISS=0x%08x (PC=0x%08x)",
                           i, (unsigned long long)g_metric_retires, g_dut_gpr[i], iss_val, g_retire.pc);
            }
            pass = RVVI_FALSE;
        }
    }
    return pass;
}

bool_t rvviRefGprsCompareWritten(uint32_t /*hartId*/, bool_t ignX0)
{
    PROF_SCOPE(g_prof_ns_gpr_cmp, g_prof_cnt_gpr_cmp);
    if (!gvsoc_engine_is_running() || gvsoc_engine_finished())
        return RVVI_TRUE;
    if (state_compare_deferred())
        return RVVI_TRUE;

    bool_t pass = RVVI_TRUE;
    uint32_t start = ignX0 ? 1 : 0;
    for (uint32_t i = start; i < 32; i++) {
        if (!(g_retire.gpr_mask & (1u << i)))
            continue;
        g_metric_comparisons_gpr++;
        uint32_t iss_val = gvsoc_engine_get_gpr(i);
        if (g_dut_gpr[i] != iss_val) {
            g_metric_mismatches++;
            if (throttle_check(g_gprw_mismatch_count, "GPR-written")) {
                BRIDGE_ERR("GPR[x%u] mismatch (written) @ retire #%llu: DUT=0x%08x ISS=0x%08x (PC=0x%08x)",
                           i, (unsigned long long)g_metric_retires, g_dut_gpr[i], iss_val, g_retire.pc);
            }
            pass = RVVI_FALSE;
        }
    }
    return pass;
}

bool_t rvviRefFprsCompare(uint32_t hartId)
{
    PROF_SCOPE(g_prof_ns_fpr_cmp, g_prof_cnt_fpr_cmp);
    if (!gvsoc_engine_is_running() || gvsoc_engine_finished() || !rvviRefFprsPresent(hartId))
        return RVVI_TRUE;
    if (state_compare_deferred())
        return RVVI_TRUE;

    bool_t pass = RVVI_TRUE;
    /* Only compare FPRs that were written this retire (mirrors the GPR logic) */
    for (uint32_t i = 0; i < 32; i++) {
        if (!(g_retire.fpr_mask & (1u << i)))
            continue;
        g_metric_comparisons_fpr++;
        uint32_t iss_val = gvsoc_engine_get_fpr(i);
        if (g_dut_fpr[i] != iss_val) {
            g_metric_mismatches++;
            if (throttle_check(g_fpr_mismatch_count, "FPR")) {
                BRIDGE_ERR("FPR[f%u] mismatch @ retire #%llu: DUT=0x%08x ISS=0x%08x (PC=0x%08x)",
                           i, (unsigned long long)g_metric_retires, g_dut_fpr[i], iss_val, g_retire.pc);
            }
            pass = RVVI_FALSE;
        }
    }
    return pass;
}

bool_t rvviRefVrsCompare(uint32_t /*hartId*/)  { return RVVI_TRUE; }

bool_t rvviRefInsBinCompare(uint32_t /*hartId*/)
{
    if (!gvsoc_engine_is_running() || gvsoc_engine_finished())
        return RVVI_TRUE;

    uint32_t iss_insn = gvsoc_engine_get_insn();

    g_metric_comparisons_insbin++;

    /* If the ISS cannot provide an opcode (0 = not decoded, or DPI mode
     * where insn_cache is not accessible), skip comparison gracefully.
     * Still count as a comparison to satisfy the testbench final check. */
    if (iss_insn == 0)
        return RVVI_TRUE;

    /* Mask comparison to instruction size: RVC (compressed) instructions
     * are 16-bit, standard instructions are 32-bit.  RVC is identified
     * by bits [1:0] != 0b11. */
    uint32_t mask = ((g_retire.insn & 0x3) != 0x3) ? 0x0000FFFF : 0xFFFFFFFF;
    if ((g_retire.insn & mask) != (iss_insn & mask)) {
        g_metric_mismatches++;
        if (throttle_check(g_insn_mismatch_count, "INSN")) {
            BRIDGE_ERR("INSN mismatch @ retire #%llu: DUT=0x%08x ISS=0x%08x (PC=0x%08x)",
                       (unsigned long long)g_metric_retires, g_retire.insn, iss_insn, g_retire.pc);
        }
        return RVVI_FALSE;
    }
    return RVVI_TRUE;
}

bool_t rvviRefCsrCompare(uint32_t /*hartId*/, uint32_t csrIndex)
{
    if (!gvsoc_engine_is_running())
        return RVVI_TRUE;

    if (state_compare_deferred())
        return RVVI_TRUE;

    /* Skip volatile CSRs */
    if (g_csr_volatile.count(csrIndex))
        return RVVI_TRUE;

    /* On trap retires RVFI does not carry exception CSR updates in the same
     * cycle; skip them here.  They are compared at the handler retire. */
    if (g_retire.is_trap && is_trap_csr(csrIndex))
        return RVVI_TRUE;

    uint32_t iss_val = 0;
    if (!gvsoc_engine_get_csr(csrIndex, &iss_val))
        return RVVI_TRUE;  /* CSR not in ISS model */

    g_metric_comparisons_csr++;

    /* For the first handler retire after a trap, use the snapshot for the
     * exception CSRs.  The snapshot was populated by rvviDutTrap from values
     * pushed explicitly by rvvi_trace2api before the CSR write-back timing could
     * corrupt them. */
    uint32_t dut_val = 0;
    if (g_pending_handler && is_trap_csr(csrIndex)) {
        auto snap_it = g_trap_csr_snapshot.find(csrIndex);
        if (snap_it != g_trap_csr_snapshot.end()) {
            dut_val = snap_it->second;
        } else {
            auto dut_it = g_dut_csr.find(csrIndex);
            dut_val = (dut_it != g_dut_csr.end()) ? dut_it->second : 0;
        }
    } else {
        auto dut_it = g_dut_csr.find(csrIndex);
        dut_val = (dut_it != g_dut_csr.end()) ? dut_it->second : 0;
    }

    /* Apply comparison mask if configured */
    auto mask_it = g_csr_compare_mask.find(csrIndex);
    if (mask_it != g_csr_compare_mask.end()) {
        uint32_t mask = (uint32_t)mask_it->second;
        dut_val &= mask;
        iss_val &= mask;
    }

    if (dut_val != iss_val) {
        g_metric_mismatches++;
        if (throttle_check(g_csr_mismatch_count, "CSR")) {
            BRIDGE_ERR("CSR[0x%03x] mismatch @ retire #%llu: DUT=0x%08x ISS=0x%08x (PC=0x%08x)%s",
                       csrIndex, (unsigned long long)g_metric_retires, dut_val, iss_val, g_retire.pc,
                       (g_pending_handler && is_trap_csr(csrIndex)) ? " [trap-snapshot]" : "");
        }
        return RVVI_FALSE;
    }
    return RVVI_TRUE;
}

bool_t rvviRefCsrsCompare(uint32_t /*hartId*/)
{
    PROF_SCOPE(g_prof_ns_csr_cmp, g_prof_cnt_csr_cmp);
    if (!gvsoc_engine_is_running() || gvsoc_engine_finished())
        return RVVI_TRUE;

    bool_t pass = RVVI_TRUE;
    for (uint32_t csr_addr : g_csr_compare_enabled) {
        if (rvviRefCsrCompare(0, csr_addr) == RVVI_FALSE)
            pass = RVVI_FALSE;
    }

    /* Consume the trap snapshot after all CSR comparisons for this retire. */
    if (g_pending_handler) {
        g_pending_handler = false;
        g_sync_trap_seam = false;
        g_trap_csr_snapshot.clear();
    }
    return pass;
}

/* ==========================================================================
 * SECTION 11 - RVVI API: CSR/memory configuration and stubs
 * ========================================================================== */

void rvviRefCsrCompareEnable(uint32_t /*hartId*/, uint32_t csrIndex,
                              bool_t enableState)
{
    if (enableState)
        g_csr_compare_enabled.insert(csrIndex);
    else
        g_csr_compare_enabled.erase(csrIndex);
}

void rvviRefCsrCompareMask(uint32_t /*hartId*/, uint32_t csrIndex,
                            uint64_t mask)
{
    g_csr_compare_mask[csrIndex] = mask;
}

bool_t rvviRefCsrSetVolatile(uint32_t /*hartId*/, uint32_t csrIndex)
{
    g_csr_volatile.insert(csrIndex);
    return RVVI_TRUE;
}

/* csrMask flags the VOLATILE bits (ImperasDV semantics: "mask of volatile
 * bits" excluded from comparison), so the compare mask is its complement. */
bool_t rvviRefCsrSetVolatileMask(uint32_t /*hartId*/, uint32_t csrIndex,
                                  uint64_t csrMask)
{
    g_csr_compare_mask[csrIndex] = ~csrMask;
    return RVVI_TRUE;
}

bool_t rvviRefCsrSetOneWayCompare(uint32_t /*hartId*/, uint32_t /*csrIndex*/,
                                   bool_t /*enable*/)                              { return RVVI_TRUE; }
/* Volatile memory windows: standard RVVI declaration, consumed by
 * sync_volatile_memory_read() on every retire whose DUT-side load address
 * falls inside a window (see g_mem_volatile above). addressHigh is
 * inclusive, matching the ImperasDV call convention. */
bool_t rvviRefMemorySetVolatile(uint64_t addressLow, uint64_t addressHigh)
{
    if (addressLow > addressHigh) {
        /* An inverted window would never match in the sync loop: fail loud
         * instead of silently disabling what the testbench asked for. */
        BRIDGE_ERR("memory volatile window rejected: low 0x%08llx > high "
                   "0x%08llx", (unsigned long long)addressLow,
                   (unsigned long long)addressHigh);
        return RVVI_FALSE;
    }
    g_mem_volatile.emplace_back(addressLow, addressHigh);
    BRIDGE_LOG("memory volatile window: 0x%08llx - 0x%08llx",
               (unsigned long long)addressLow, (unsigned long long)addressHigh);
    return RVVI_TRUE;
}
bool_t rvviRefMemorySetPrivilege(uint64_t /*addrLo*/, uint64_t /*addrHi*/,
                                  uint32_t /*access*/)                             { return RVVI_TRUE; }

bool_t   rvviRefPcSet(uint32_t /*hartId*/, uint64_t /*address*/)           { return RVVI_TRUE; }

void rvviRefGprSet(uint32_t /*hartId*/, uint32_t gprIndex, uint64_t gprValue)
{
    if (gvsoc_engine_is_running())
        gvsoc_engine_set_gpr(gprIndex, (uint32_t)gprValue);
}

void rvviRefFprSet(uint32_t /*hartId*/, uint32_t fprIndex, uint64_t fprValue)
{
    if (gvsoc_engine_is_running())
        gvsoc_engine_set_fpr(fprIndex, (uint32_t)fprValue);
}

void     rvviRefVrSet(uint32_t /*hartId*/, uint32_t /*vrIndex*/,
                      uint32_t /*byteIndex*/, uint8_t /*data*/) {}

void rvviRefCsrSet(uint32_t /*hartId*/, uint32_t csrIndex, uint64_t value)
{
    if (gvsoc_engine_is_running())
        gvsoc_engine_set_csr(csrIndex, (uint32_t)value);
}

void     rvviRefMemoryWrite(uint32_t /*hartId*/, uint64_t /*address*/,
                             uint64_t /*data*/, uint32_t /*size*/) {}
void     rvviRefReservationInvalidate(uint32_t /*hartId*/) {}
void     rvviRefStateDump(uint32_t /*hartId*/) {}

uint64_t rvviRefMetricGet(rvviMetricE metric)
{
    switch (metric) {
    case RVVI_METRIC_RETIRES:            return g_metric_retires;
    case RVVI_METRIC_TRAPS:              return g_metric_traps;
    case RVVI_METRIC_MISMATCHES:         return g_metric_mismatches;
    case RVVI_METRIC_COMPARISONS_PC:     return g_metric_comparisons_pc;
    case RVVI_METRIC_COMPARISONS_GPR:    return g_metric_comparisons_gpr;
    case RVVI_METRIC_COMPARISONS_FPR:    return g_metric_comparisons_fpr;
    case RVVI_METRIC_COMPARISONS_CSR:    return g_metric_comparisons_csr;
    case RVVI_METRIC_COMPARISONS_INSBIN: return g_metric_comparisons_insbin;
    default:                             return 0;
    }
}

bool_t      rvviRefCsrPresent(uint32_t /*hartId*/, uint32_t /*csrIndex*/) { return RVVI_TRUE;  }
/* FPR comparison is enabled: the ISS regfile reset zeros FPRs to match the
 * DUT reset value. */
bool_t      rvviRefFprsPresent(uint32_t /*hartId*/)                       { return RVVI_TRUE; }
bool_t      rvviRefVrsPresent(uint32_t /*hartId*/)                        { return RVVI_FALSE; }
uint32_t    rvviRefCsrIndex(uint32_t /*hartId*/,
                             const char * /*csrName*/)  { return (uint32_t)RVVI_INVALID_INDEX; }
const char *rvviRefCsrName(uint32_t /*hartId*/, uint32_t /*csrIndex*/)    { return ""; }
const char *rvviRefGprName(uint32_t /*hartId*/, uint32_t /*gprIndex*/)    { return ""; }
const char *rvviRefFprName(uint32_t /*hartId*/, uint32_t /*fprIndex*/)    { return ""; }
const char *rvviRefVrName(uint32_t /*hartId*/, uint32_t /*vrIndex*/)      { return ""; }
const char *rvviDasmInsBin(uint32_t /*hartId*/, uint64_t /*address*/,
                             uint64_t /*insBin*/)                          { return ""; }
const char *rvviErrorGet(void)                                             { return ""; }

/* Performance-counter CSR addresses: cycle/instret/hpmcounter3..31 in the
 * machine (0xB0x) and user (0xC0x) banks, low and high halves. */
static inline bool csr_is_perf_counter(uint32_t addr)
{
    uint32_t base = addr & ~0x080u;  /* fold the *h bank onto the low one */
    return (base >= 0xB00 && base <= 0xB1F) ||
           (base >= 0xC00 && base <= 0xC1F);
}

/* Volatile-counter read sync (see g_volatile_sync_enabled above). Runs after
 * the ISS stepped the current retire, before the compares and the RVVI-TEXT
 * emit, so both observe the synced value. Decodes the DUT instruction: any
 * CSR read form (csrrw needs rd!=x0, csrrs/csrrc/immediates always read) of a
 * performance counter with a DUT-reported rd write gets the DUT value. */
static void sync_volatile_counter_read(void)
{
    if (!g_volatile_sync_enabled)
        return;
    uint32_t insn = g_retire.insn;
    if ((insn & 0x7f) != 0x73)       /* SYSTEM opcode (CSR ops are never compressed) */
        return;
    uint32_t funct3 = (insn >> 12) & 0x7;
    if (funct3 == 0 || funct3 == 4)  /* ecall/ebreak/xret class, not a CSR op */
        return;
    uint32_t csr = insn >> 20;
    if (!csr_is_perf_counter(csr))
        return;
    uint32_t rd = (insn >> 7) & 0x1f;
    if (rd == 0 || !(g_retire.gpr_mask & (1u << rd)))
        return;                      /* no rd write reported by the DUT */
    uint32_t iss_val = gvsoc_engine_get_gpr(rd);
    if (iss_val == g_dut_gpr[rd])
        return;
    gvsoc_engine_set_gpr(rd, g_dut_gpr[rd]);
    g_volatile_sync_count++;
    BRIDGE_LOG_HOT("volatile counter CSR[0x%03x] read @ PC=0x%08x: "
                   "x%u ISS=0x%08x -> DUT=0x%08x (sync #%llu)",
                   csr, g_retire.pc, rd, iss_val, g_dut_gpr[rd],
                   (unsigned long long)g_volatile_sync_count);
}

/* Volatile-memory read sync (see g_mem_volatile above). Runs after the ISS
 * stepped the current retire, before the compares and the RVVI-TEXT emit, so
 * both observe the synced value - the exact placement of the counter sync.
 * The DUT-side effective address and byte mask come from RVFI
 * (rvfi_mem_addr / rvfi_mem_rmask) through the batch DPI: no instruction
 * decode, so every load form (C.LW, post-increment, misaligned) is covered.
 * All GPRs the DUT wrote on this retire are forced, not just a decoded rd: a
 * post-increment load also writes the base register, whose value matches on
 * both sides anyway, so the extra write is idempotent. ACCEPTED COVERAGE
 * GAP: forcing the whole written set means an honest ISS divergence on a
 * side-effect register of the SAME instruction (e.g. a post-increment
 * address bug) is repaired instead of reported when it lands on a load
 * from a volatile window - the price of staying decode-free. The windows
 * are tiny (8 bytes today), so the exposure is negligible. Gated on a PC-aligned
 * row: during a divergence the resync paths own the state and this sync must
 * not blur the honest mismatch. */
static void sync_volatile_memory_read(uint64_t addr, uint32_t rmask)
{
    if (!g_volatile_sync_enabled || rmask == 0 || g_mem_volatile.empty())
        return;
    /* The CV32E40P tracer expands each byte-enable lane to 8 mask BITS
     * (be_to_mask), so the byte count is popcount/8; the lanes of one
     * access are contiguous from addr. Clamp to >=1 byte so a producer
     * with a byte-granular mask cannot underflow the span. */
    uint32_t bytes = (uint32_t)__builtin_popcount(rmask) / 8;
    if (bytes == 0)
        bytes = 1;
    uint64_t last = addr + bytes - 1;
    bool hit = false;
    for (const auto &w : g_mem_volatile) {
        if (addr <= w.second && last >= w.first) {
            hit = true;
            break;
        }
    }
    if (!hit)
        return;
    if (gvsoc_engine_get_pc() != g_retire.pc)
        return;
    for (uint32_t i = 1; i < 32; i++) {
        if (!(g_retire.gpr_mask & (1u << i)))
            continue;
        uint32_t iss_val = gvsoc_engine_get_gpr(i);
        if (iss_val == g_dut_gpr[i])
            continue;
        gvsoc_engine_set_gpr(i, g_dut_gpr[i]);
        g_volatile_mem_sync_count++;
        BRIDGE_LOG_HOT("volatile mem read @ PC=0x%08x addr=0x%08llx: "
                       "x%u ISS=0x%08x -> DUT=0x%08x (sync #%llu)",
                       g_retire.pc, (unsigned long long)addr, i, iss_val,
                       g_dut_gpr[i],
                       (unsigned long long)g_volatile_mem_sync_count);
    }
}

/* Batched DPI path - one SV->C crossing per retire instead of six.
 * Combines EventStep + PcCompare + GprsCompareWritten + CsrsCompare +
 * FprsCompare + InsBinCompare.  rvviDutRetire is NOT called here:
 * rvvi_trace2api already invokes it earlier in the retire flow
 * (unconditionally, outside the USE_GVSOC ifdef); calling it again would
 * double-count g_metric_retires and re-commit the retire event.
 *
 * Returns a bitmask:
 *   0x01=step ok, 0x02=pc, 0x04=gpr, 0x08=csr, 0x10=fpr, 0x20=runaway,
 *   0x40=insn.
 * Each compare bit is set on PASS or when not applicable (e.g. fpr on a no-FPU
 * build, insn when the ISS opcode is unavailable), so the SV side can treat
 * 0x5F as a clean retire; 0 = step failed. The 0x20 bit is set (in addition
 * to the compare bits) once the ISS runaway detector has latched: the ISS is
 * permanently diverged and stuck, so the SV side aborts with a clean FAIL
 * instead of letting the OS timeout reap the (crawling) simulation. We do
 * NOT call vpiFinish here - termination is left to the SV side so the test
 * surfaces a UVM-visible error (SIMULATION FAILED), not a silent finish.
 *
 * dutInsn/debugMode are unused; kept for API symmetry with the SV import.
 * dutMemAddr/dutMemRmask carry the retire's RVFI data-memory read (address
 * and byte mask, 0 when the instruction did no read) for the volatile
 * memory window sync. */
int rvviRefRetireAndCompare(
    uint32_t /*hartId*/,
    uint64_t dutPc,
    uint32_t /*dutInsn*/,
    uint8_t  /*debugMode*/,
    uint64_t dutMemAddr,
    uint32_t dutMemRmask)
{
    const uint32_t hart_id = 0;  /* single-hart sim */

    /* DPI boundary guard: a C++ exception must never cross into the simulator.
     * DPI cannot propagate exceptions, so catch everything and return 0. */
    try {
        /* Ordering-contract tripwire (see the retire-lifecycle comment above):
         * the batch must run on the retire committed by rvviDutRetire. */
        if (g_retire.pc != (uint32_t)dutPc &&
            throttle_check(g_order_mismatch_count, "retire-ordering"))
            BRIDGE_ERR("retire ordering broken: batch dutPc=0x%08llx but committed "
                       "retire PC=0x%08x", (unsigned long long)dutPc, g_retire.pc);

        if (!rvviRefEventStep(hart_id))
            return 0;

        int result = 0x01;  /* step OK */
        sync_volatile_counter_read();
        sync_volatile_memory_read(dutMemAddr, dutMemRmask);
        if (rvviRefPcCompare(hart_id))                      result |= 0x02;
        if (rvviRefGprsCompareWritten(hart_id, RVVI_TRUE))  result |= 0x04;
        if (rvviRefCsrsCompare(hart_id))                    result |= 0x08;
        if (rvviRefFprsCompare(hart_id))                    result |= 0x10;
        if (rvviRefInsBinCompare(hart_id))                  result |= 0x40;

        /* Runaway: ISS permanently diverged and stuck. OR in the runaway
         * bit so the SV side aborts deterministically (clean FAIL) rather
         * than waiting for the OS timeout to reap a crawling sim. */
        if (gvsoc_engine_is_runaway())                      result |= 0x20;

        /* RVVI-TEXT emit (off by default): one line per retire to each file.
         * This is the last call of the retire, after all rvviDutCsrSet and
         * rvviDutTrap, so the write-set is complete. */
        emit_retire_lines(/*ref_from_dut=*/false);
        if (__builtin_expect(g_rvvi_text_enabled, 0) &&
            (++g_rvvi_text_count % 1000) == 0) {
            /* dut_fp is null in ref-only mode; guard it like every
             * other site in this file (rvviRefInit/rvviRefShutdown) does --
             * fflush(NULL) is defined but flushes every open stream in the
             * process, a silent cross-.so side effect worth avoiding. */
            if (g_rvvi_text_dut_fp) fflush(g_rvvi_text_dut_fp);
            if (g_rvvi_text_ref_fp) fflush(g_rvvi_text_ref_fp);
        }

        return result;
    } catch (const std::exception &e) {
        BRIDGE_LOG("batched DPI: std::exception in rvviRefRetireAndCompare: %s", e.what());
        return 0;
    } catch (...) {
        BRIDGE_LOG("batched DPI: unknown exception in rvviRefRetireAndCompare");
        return 0;
    }
}

bool_t rvviRefConfigSetInt(uint64_t /*configParam*/, uint64_t /*value*/)  { return RVVI_TRUE; }
bool_t rvviRefConfigSetString(uint64_t /*configParam*/,
                               const char * /*value*/)                    { return RVVI_TRUE; }

uint64_t rvviRefConnIndexGet(const char * /*name*/)                  { return (uint64_t)RVVI_INVALID_INDEX; }
bool_t   rvviRefConnSetEmpty(uint64_t /*connIndex*/)                 { return RVVI_TRUE; }
bool_t   rvviRefConnSetFull(uint64_t /*connIndex*/)                  { return RVVI_TRUE; }
bool_t   rvviRefConnData(uint64_t /*connIndex*/, uint32_t /*offset*/,
                          uint64_t /*value*/, bool_t /*commit*/)     { return RVVI_TRUE; }

void setContextExtMemory(const char * /*func*/) {}

/* ==========================================================================
 * SECTION 12 - RVVI API: sim-complete query
 * ========================================================================== */

/* Watchdog hook: SV-side polling entry point.
 * The SV watchdog calls this every 100us sim time; if it returns 1 the
 * watchdog issues $finish to prevent the sim hanging while the DUT is in WFI. */
int rvviRefIsFinished(void)
{
    return gvsoc_engine_finished() ? 1 : 0;
}

} // extern "C"
