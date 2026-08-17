#!/bin/bash
# Quick validation sweep for the GVSOC RVVI co-simulation: one run of every
# test type in each testbench configuration, instead of a full regression
# (~100 runs vs 651+; same breadth of stimulus types, a fraction of the wall
# clock). Generated tests run with SEED=1 so two sweeps on the same build
# are directly comparable.
#
# Test lists mirror cv32e40p_full_covg_no_pulp.yaml (CFG=default) and the
# cv32e40pv2_{xpulp,fpu,fpu_zfinx}_instr yamls (PULP/FPU configs); the
# interrupt lanes come from the same no_pulp yaml (CFG=default) and from
# cv32e40pv2_interrupt_debug_short.yaml (CFG=pulp, gen_rand_int aliases).
# The debug axis covers every (TEST, test_cfg) combination of
# cv32e40p_debug.yaml and cv32e40pv2_interrupt_debug.yaml (one lane each,
# characterized 2026-08-04).
#
# Usage:   test/quick_val.sh [output-dir] [cfg ...]
#            cfg list optional (default: default pulp pulp_fpu pulp_fpu_zfinx)
# Expects: simulator environment already set up (see docs/TESTING.md) and a
#          compiled toolchain; run from anywhere inside the core-v-verif tree.
#
# Exit code: 0 only when no lane reports FAIL/TIMEOUT/NO_SIM. Known-fail
# lanes (kind xfail) and skips do not affect it.
#
# Known-open expectations:
#   - corev_rand_fp_instr_* run the shared corev-dv rand FP stream with the
#     per-config test_cfg (floating_pt_instr_en / floating_pt_zfinx_instr_en);
#     without it riscv-dv aborts generation with a constraint contradiction.
#     FULL FP PARITY since 2026-08-05: all rand FP lanes are kind gen. The
#     FMA double-rounding is fixed structurally with a round-to-odd binary64
#     intermediate (plus the exact-zero sign under RDN), and three latent
#     flexfloat defects it exposed are fixed with it: the grid-step rounding
#     add in sanitize is now exact (integer truncation first), underflow
#     follows FPnew's tininess-after-rounding-unbounded formula
#     (fpnew_fma.sv:616), and the sticky bit is kept for deeply tiny values
#     (shift >= backend width). Historical: the fcvt negative-saturation
#     bug was fixed 2026-07-24.
#   - corev_rand_pulp_{,hwloop_}illegal_instr_test run the base rand TEST
#     with TEST_CFG_FILE=insert_illegal_instr (regress-yaml alias).
#   - DEBUG-MODE SWEEP 2026-08-10/11 (nine lanes flipped expected-pass,
#     evidence /tmp/tracer_triage_20260810 + /tmp/final_gate_20260811):
#     (1) interrupt_debug 231mm: the take_irq injection window let a
#     latched haltreq hijack the entry (Irq::check debug branch outranks
#     the IRQ ladder) - the engine now saves/suppresses req_debug +
#     haltreq_level for the one step and restores after (the level
#     re-arms post-dret like the RTL wire).
#     (2) debug_test tracer phantom: on a haltreq DBG_TAKEN_ID the RVFI
#     tracer emitted a retire for the instruction killed in ID -
#     bhv/cv32e40p_rvfi.sv now drops that trace_id row (order released,
#     send_rvfi contiguity kept). Fix is in the tracer, not the bridge.
#     (3) exceptions/ecall in debug mode -> dm_exception_addr with NO CSR
#     update (debug.rst: Cv32e40pException::raise debug-mode branch; new
#     config key debug_exception_handler=0x1A111600 in the v2 JSONs).
#     (4) mret in debug mode -> dm_exception_addr, no side effects
#     (debug.rst "without affecting status registers";
#     Cv32e40pCore::mret_handle).
#     (5) execute-trigger vs the standing irq-check suppression: check()
#     never saw the tdata2 boundary, the ISS ran one insn past the DUT's
#     trigger entry - the async gate is consumed inside check() and the
#     personality evaluates the trigger ahead of it on every dispatch
#     boundary, so the entry certifies model-side (debug_test_trigger).
#     Cause priority on a shared boundary follows the RTL, per entry
#     path: TRIGGER overrides an armed haltreq (DBG_TAKEN_ID table) but
#     YIELDS to a closing single-step window (DBG_TAKEN_IF's cause mux
#     never looks at trigger_match - step wins, the trigger fires on the
#     next session).
#     (6) self-modifying code (debug_hwloop_test writes the hwloop body):
#     a hart store into a decoded insn-cache page queues a deferred flush
#     (LsuV2::data_req_virtual + InsnCache::covers) - CV32E40P fetches
#     straight from memory, the model must stay coherent without fence.i.
#     (7) hwloop count on a trapping loop-end insn (ebreak at lpend): the
#     RTL kills the insn before the loop update - the dispatch loop skips
#     hwloop.check when the handler raised (hwloop-ebreak lanes, cc6 skew).
#     debug_test_boot_set stays xfail: the TEST PROGRAM itself flags
#     failure at reset (77 retires, exit_value=1, no co-sim mismatch) -
#     debug-req-at-reset TB config path, separate triage.
#     FINAL GATE 2026-08-11 (/tmp/final_gate_20260811, 18 lanes on the
#     full fix stack): 16 fast lanes PASS + mem_stress PASS (TMO=6000,
#     3165s); nested fails at its self-corruption point as expected (see
#     the dedicated interrupt_nested note - unpassable by construction).
#     CONSOLIDATION 2026-08-11 morning (review-driven): the async gate is
#     consumed inside Irq::check() and the execute trigger is evaluated
#     model-side ahead of it - the engine's trigger carve-out is GONE;
#     SMC hook on physical addresses with a page-crossing short-circuit;
#     tracer halt-kill arming guarded against re-arm (review HIGH).
#     FULL GATE on the consolidated build (155 PASS / 0 FAIL /
#     3 KNOWN_FAIL / 2 SKIP, 08:36-13:08, archived
#     /data2/marco.paci/validation-evidence/quickval_gate_20260811_c2):
#     zero regressions, the three KNOWN_FAIL are exactly the documented
#     xfails (interrupt_nested, debug_test_boot_set, coremark).
#   - PARITY AUDIT 2026-08-12: all_csr_por and load_store_rs1_zero added
#     (CFG=pulp) - the only two tests of the official CV32E40Pv2 sign-off
#     matrix (CV32E40Pv2_test_list.xlsx) that had no quick_val lane. See
#     cv32e40p/docs/GVSOC_vplan_parity_report.md for the full audit.
#     load_store_rs1_zero PASS (28s). all_csr_por diverged at pmpcfg0
#     (retire #1158407: the RTL has no PMP and raises illegal, the model
#     declared the bank even with the PmpEmpty variant) - FIXED by
#     undeclaring pmpcfg0..15/pmpaddr0..63 in Cv32e40pCsr; the full CSR
#     sweep now PASSES (2212s, hence TMO=3600) with csr_access/modeled/
#     readonly non-regression PASS. Evidence
#     /data2/.../{parity_lanes,pmp_fix_val}_20260812.
#   - debug-mode tests (debug_hwloop_test, pulp_hardware_loop_debug_test):
#     the reference model now follows the DUT into debug the informed way
#     (engine take-debug on the rvvi.debug_mode edge, dpc/hwloop CSRs
#     forced from the DUT at the entry seam, dret outside debug raises
#     illegal, dcsr.prv WARL-pinned to M). The residual is a
#     one-instruction retire misalignment inside the debug ROM when debug
#     entries re-arm in rapid succession - RESOLVED by the 2026-08-10/11
#     debug-mode sweep above (items 3/4/6/7); lanes now expected-pass.
#   - debug axis (characterized 2026-08-04, evidence
#     /data2/.../debug_axis_char_20260804; RESOLVED in bulk 2026-08-05
#     night, gate quickval_gate_20260805_night: 139/160 PASS, 33 debug
#     lanes flipped KNOWN_FAIL->PASS in one sweep). Signatures:
#     (a) hwloop counters (0xcc2 AND 0xcc6, both directions, debug-ROM and
#     application PCs) - reviewed 2026-08-05: NOT an hwloop-model gap (the
#     spec mandates the counters keep updating in debug mode, the RTL has
#     no debug term in the decrement path, and hwloop_in_debug_rom PASSes);
#     it was the hwloop-visible projection of class (b) and fell with it -
#     every hwloop-debug lane now expected-pass. Do NOT gate the model's
#     decrement on debug mode.
#     (b) async debug-entry, decomposed 2026-08-05 into five sub-signatures:
#     B1/B2/B3 root cause ELIMINATED 2026-08-05 evening: the over-execution
#     came from soc/mem latency=1 holding every load (sync followers parked
#     in the inflight ring, ISS architecturally ahead of the boundary) -
#     latency=0 on every port (cv32e40p_v2_standalone.py) plus the native
#     debug-entry model (haltreq wire + level, ebreakm, dret single-step
#     window, wfi-as-nop on pending debug, wfi_wake release wire) turned
#     the WHOLE class around: 33 debug lanes (corev_rand_debug and
#     _single_step first, then every hwloop-debug / random-debug-req /
#     int+debug-trigger combination) went KNOWN_FAIL->PASS in the
#     2026-08-05 night gate and are expected-pass now.
#     The residual on debug_test is a DUT-side TRACER artifact: on a
#     haltreq entry through DBG_TAKEN_ID the RVFI tracer emits a retire for
#     the instruction killed in ID, register write included (proof: the
#     debugger's save/restore reads the OLD value on the DUT; dpc-force
#     gauge shows a systematic +4) - FIXED 2026-08-10 in the tracer
#     itself (sweep item 2 above), debug_test now expected-pass.
#     B4 FIXED 2026-08-05 by three stacked fixes:
#     entry rows whose rvfi_dbg_mode marker the tracer never asserts are
#     recognized by PC (== the model's dm_halt_addr) in the bridge; wfi
#     executes as nop in debug/single-step mode (RTL controller behavior);
#     c.ebreak in debug mode re-enters the ROM like the 32-bit form
#     (pulp_instr_ebreak_debug_test revalidated PASS, now gen). B5 (WFI +
#     haltreq with mie=0) FIXED 2026-08-05 evening: haltreq is a
#     first-class injector wire handled inside the model
#     (Cv32e40pIrq::haltreq_sync arms req_debug and runs the full WFI
#     release - insn_terminate is only callable model-side), plus the
#     dedicated wfi_wake wire for wakes the interrupt wires cannot carry.
#     (c) tdata1 (0x7a1) bit 2 (execute match enable) reads back set on the
#     DUT and clear in the model on trigger tests - FIXED 2026-08-05
#     (tdata1/tdata2 debug-gated writes modelled, bridge read clamp
#     dropped; pulp hwloop_debug_trigger revalidated PASS). The directed
#     debug_test_trigger keeps a narrower residual (coverage-gap note).
#   - coverage-gap lanes (second pass 2026-08-04, evidence
#     /data2/.../gap_char_20260804): the last test types the regress yamls
#     run and quick_val did not. Six pass and are plain lanes (matmul int/
#     float, the directed hwloop/interrupt covg tests - the interrupt one
#     generates only under pulp_fpu with floating_pt_instr_en, the rand FP
#     registry constraint again - and the pulp_instr_test debug_ebreak /
#     single_step aliases). Of the four known-open lanes of that pass:
#     corev_rand_illegal_instr FIXED 2026-08-05 (the "mepc skew" was the
#     ISS executing reserved RVC code-points, e.g. c.addi4spn nzuimm=0,
#     that the RTL correctly rejects - guards in rv32c.hpp, lane now gen);
#     debug_test_trigger FIXED 2026-08-11 (sweep item 5 above: the
#     engine opens the irq-check for the armed-trigger boundary dispatch
#     and the trigger outranks a same-boundary haltreq like the RTL); interrupt_test on Zfinx (the
#     default sibling); and coremark, which carries TWO independent
#     causes: a GPR read of the TB vp_cycle_counter at 0x15001004
#     (cycles-since-reset, unstable across seeds with random OBI stalls -
#     not modellable instruction-accurate) and a UVM phase timeout at
#     20 ms sim time (the benchmark never completes its iterations;
#     ~40 min wall, hence the dedicated TMO).
#   - fpu_bugs_test: PASS on both FP configs since 2026-08-05 (kind run) -
#     the underflow accrual difference was FPnew's
#     tininess-after-rounding-unbounded semantics, now modelled in
#     flexfloat_sanitize; the FMA artefact fell with the round-to-odd fix.
#   - interrupt lanes: corev_rand_interrupt, corev_rand_interrupt_wfi and
#     interrupt_bootstrap PASS. The known-open residual (reviewed
#     2026-08-05) is NOT an ISS arbitration gap - the model's priority
#     ladder matches the RTL int_controller bit for bit, and with the
#     skip_irq_check defense the ladder is never even reached in co-sim.
#     It is a synchronous-exception + enabled-IRQ collision on the same
#     retire boundary, resolved in opposite order (the DUT takes the IRQ
#     and kills the excepting instruction; the ISS takes the exception):
#     recognizable as DUT_PC - ISS_PC == 4*(mcause&0x1f) with the ISS at
#     mtvec base. Both terms of the reactive resync detector go blind
#     there (ISS MIE already cleared by its trap; DUT mepc != ISS PC), so
#     the divergence cascades. The informed-IRQ seam (+rvvi_informed_irq)
#     was re-probed 2026-08-05 evening with latency=0: the control lanes
#     now PASS informed (corev_rand_interrupt, interrupt_test,
#     interrupt_wfi - the wfi_wake retry unblocks the parked-WFI takes)
#     but the collision lanes stay worse than reactive (exception 51 mm,
#     nested 122, interrupt_debug 215 vs ~10): reactive stays the default,
#     informed stays opt-in. Residual informed signature: mepc one insn
#     high on WFI-wake takes ([trap-snapshot], delta +4). An
#     interrupt+debug collision on a debug ENTRY is now solved informed
#     (the entry row's fresh mcause write carries the taken cause id ->
#     collide_irq_id -> the model takes that line ahead of the entry).
#     Lanes: corev_rand_interrupt_exception,
#     corev_rand_pulp_with_priv_instr_test, corev_rand_interrupt_nested
#     (nested root cause is NOT this collision class - see the dedicated
#     note below, root-caused 2026-08-11); interrupt_test carries a
#     DIFFERENT root cause, FIXED 2026-08-05: the test reads the TB
#     virtual-peripheral RNG at 0x15001000 (*(volatile int*)...), which no
#     functional model can predict - the same volatile-device class as the
#     vp_cycle_counter read in coremark. Handled the ImperasDV way: the
#     wrap declares rvviRefMemorySetVolatile('h15001000,'h15001007) and the
#     bridge copies the DUT rd into the ISS on loads from the window
#     (RVFI mem_addr/mem_rmask through the batch DPI). _wfi_mem_stress
#     since 2026-08-05 compares CLEAN (a WFI parked mid-redirect is now
#     woken and redirected: drain break + wake retry); its TMO=600 cap was
#     lifted 2026-08-05 (decision E) but the ~78k reactive resyncs do not
#     fit the default per-lane budget - VALIDATED PASS 2026-08-11 with a
#     dedicated TMO=6000 (final gate: rc=0 at 3165s), lane is gen now.
#   - corev_rand_interrupt_nested: NOT a co-sim gap - the generated test
#     self-destructs (root-caused 2026-08-11). The handler re-enables
#     mstatus.MIE 10 insns into its prologue (csrsi mstatus,8 at the
#     window 0x1ea9e..0x1eaaa) and the TB irq agent fires continuously,
#     so entries nest without ever reaching the epilogue pop: the kernel
#     stack descends 16B per level and after ~7884 levels (~83400
#     retires, 459701 ns, deterministic at SEED=1) the context save
#     overwrites the handler's own code (x2 reaches 0x1ea8x - the c.swsp
#     at 0x1ea88 stores over itself). The DUT rides the stale prefetch a
#     couple of insns, then storms illegal-instruction forever (103k+
#     illegals, PC walking off-program): the historic rc=124 "timeout"
#     was this runaway in DUT/ISS lockstep, NOT a clean-but-slow run.
#     With the 2026-08-11 SMC insn-cache coherence fix the ISS honestly
#     redecodes the clobbered bytes and diverges AT the corruption point
#     (fail-fast ~74s wall instead of hours of lockstep derailment).
#     Unpassable at ANY timeout by construction; a fix needs a TB
#     irq-rate constraint or a generator handler change upstream. Keep
#     gen_xfail, TMO=900 (fail-fast).
#
# Seed: generated lanes default to SEED=1 (deterministic snapshot, sweeps
# directly comparable). Set QV_SEED=<n> to run the SAME lane list at a
# different seed - the multi-seed depth axis (a low-probability timing race
# was only ever caught by a non-1 seed).

set -o pipefail

SELF_DIR=$(cd "$(dirname "$0")" && pwd)
SUB=$(dirname "$SELF_DIR")
CVV=$(cd "$SUB/../.." && pwd)
UVMT="$CVV/cv32e40p/sim/uvmt"
OUT=/tmp/gvsoc_rvvi_quickval_$(date +%Y%m%d_%H%M%S)
if [ $# -gt 0 ]; then OUT=$1; shift; fi
CFGS=${*:-default pulp pulp_fpu pulp_fpu_zfinx}
mkdir -p "$OUT" || { echo "cannot create output dir $OUT" >&2; exit 1; }

if ! command -v vsim > /dev/null 2>&1; then
    echo "vsim not in PATH - set up the simulator environment first (docs/TESTING.md)" >&2
    exit 1
fi

cd "$UVMT" || exit 1
PASS=0; FAIL=0; XFAIL=0; SKIP=0
: > "$OUT/SUMMARY.txt"
echo "quick_val start: $(date -Iseconds)" >> "$OUT/SUMMARY.txt"
echo "reference core: iss_v2" >> "$OUT/SUMMARY.txt"
echo "configs: $CFGS" >> "$OUT/SUMMARY.txt"
echo "seed: ${QV_SEED:-1}" >> "$OUT/SUMMARY.txt"

# Entry format: kind|label|TEST|extra make args
#   kind = gen (gen_corev-dv + test) / run (test only) / xfail (run, failure
#          expected and reported as KNOWN_FAIL) / skip (report only)
#   label names the log and report line (differs from TEST only when the
#   same TEST runs twice with different flags).
#   Multi-word values in the extra field MUST be double-quoted (e.g.
#   VSIM_USER_FLAGS="+a +b"): the field is re-parsed by eval and unquoted
#   words would reach make as separate arguments.

TESTS_default='
gen|corev_rand_arithmetic_base_test|corev_rand_arithmetic_base_test|
gen|corev_rand_instr_test|corev_rand_instr_test|
gen|corev_rand_jump_stress_test|corev_rand_jump_stress_test|
gen|corev_rand_instr_long_stall|corev_rand_instr_long_stall|
gen|corev_rand_interrupt|corev_rand_interrupt|
gen|corev_rand_interrupt_wfi|corev_rand_interrupt_wfi|
gen|corev_rand_interrupt_wfi_mem_stress|corev_rand_interrupt_wfi_mem_stress|TMO=6000
gen|corev_rand_interrupt_exception|corev_rand_interrupt_exception|
gen_xfail|corev_rand_interrupt_nested|corev_rand_interrupt_nested|TMO=900
run|hello-world|hello-world|
run|branch_zero|branch_zero|
run|csr_instr_asm|csr_instr_asm|
run|csr_instructions|csr_instructions|
run|cv32e40p_csr_access_test|cv32e40p_csr_access_test|
run|cv32e40p_readonly_csr_access_test|cv32e40p_readonly_csr_access_test|
run|dhrystone|dhrystone|
run|fibonacci|fibonacci|
run|generic_exception_test|generic_exception_test|
run|hpmcounter_basic_test|hpmcounter_basic_test|
run|hpmcounter_hazard_test|hpmcounter_hazard_test|
run|illegal|illegal|
run|illegal_instr_test|illegal_instr_test|
run|interrupt_bootstrap|interrupt_bootstrap|
run|interrupt_test|interrupt_test|
run|isa_fcov_holes|isa_fcov_holes|
run|mhpmcounter29_csr_access_test_1|mhpmcounter29_csr_access_test_1|
run|misalign|misalign|
run|modeled_csr_por|modeled_csr_por|
run|perf_counters_instructions|perf_counters_instructions|
run|requested_csr_por|requested_csr_por|
run|riscv_arithmetic_basic_test_0|riscv_arithmetic_basic_test_0|
run|riscv_arithmetic_basic_test_1|riscv_arithmetic_basic_test_1|
gen|corev_rand_debug|corev_rand_debug|
gen|corev_rand_debug_ebreak|corev_rand_debug_ebreak|
gen|corev_rand_debug_single_step|corev_rand_debug_single_step|
gen|corev_rand_interrupt_debug|corev_rand_interrupt_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
run|debug_test|debug_test|
xfail|debug_test_boot_set|debug_test_boot_set|
run|debug_test_known_miscompares|debug_test_known_miscompares|CFG_PLUSARGS="+UVM_TIMEOUT=20000000"
run|debug_test_reset|debug_test_reset|
run|riscv_ebreak_test_0|riscv_ebreak_test_0|CFG_PLUSARGS="+UVM_TIMEOUT=20000000"
gen|corev_rand_illegal_instr_test|corev_rand_illegal_instr_test|
run|debug_test_trigger|debug_test_trigger|
run|matmul_32b_int|matmul_32b_int|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
xfail|coremark|coremark|CFG_PLUSARGS="+UVM_TIMEOUT=20000000" TMO=3600
'

TESTS_pulp='
gen|corev_rand_pulp_hwloop_test|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_instr_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000"
gen|corev_rand_pulp_simd_instr_test|corev_rand_pulp_simd_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
gen|corev_rand_pulp_mac_instr_test|corev_rand_pulp_mac_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
gen|corev_rand_pulp_hwloop_exception|corev_rand_pulp_hwloop_exception|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_illegal_instr_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=insert_illegal_instr
gen|corev_rand_pulp_hwloop_illegal_instr_test|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=25000000" TEST_CFG_FILE=insert_illegal_instr
gen_xfail|corev_rand_pulp_with_priv_instr_test|corev_rand_pulp_with_priv_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_instr_interrupt_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int
gen|corev_rand_pulp_hwloop_interrupt_test|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int
gen|corev_directed_pulp_hwloop_test_with_interrupt|corev_directed_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int
gen|corev_rand_pulp_hwloop_count_range_test|corev_rand_pulp_hwloop_count_range_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" VSIM_USER_FLAGS=+skip_sampling_uvme_rv32x_hwloop_covg
gen|tb_hack_obi_gnt_stalls|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" VSIM_USER_FLAGS="+random_instr_stall +random_data_stall +tb_hack_1_obi_gnt_signal"
run|pulp_hardware_loop|pulp_hardware_loop|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" VSIM_USER_FLAGS="+skip_sampling_uvme_rv32x_hwloop_covg +fixed_data_gnt_stall=3"
run|pulp_hardware_loop_interrupt_test|pulp_hardware_loop_interrupt_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_hardware_loop_debug_test|pulp_hardware_loop_debug_test|
run|debug_hwloop_test|debug_hwloop_test|
run|custom_opcode_illegal_test|custom_opcode_illegal_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|cv32e40pv2_illegal_ro_csr_access_test|cv32e40pv2_illegal_ro_csr_access_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|cv32e40p_csr_access_test|cv32e40p_csr_access_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|all_csr_por|all_csr_por|CFG_PLUSARGS="+UVM_TIMEOUT=300000000" TMO=3600
run|load_store_rs1_zero|load_store_rs1_zero|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
run|jalr_test|jalr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_bit_manipulation|pulp_bit_manipulation|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_general_alu|pulp_general_alu|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_immediate_branching|pulp_immediate_branching|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_multiply_accumulate|pulp_multiply_accumulate|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_post_increment_load_store|pulp_post_increment_load_store|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_add_sub|pulp_vectorial_add_sub|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_avg|pulp_vectorial_avg|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_bit_manip|pulp_vectorial_bit_manip|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_bitwise|pulp_vectorial_bitwise|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_comparison_1|pulp_vectorial_comparison_1|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_comparison_2|pulp_vectorial_comparison_2|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_comparison_3|pulp_vectorial_comparison_3|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_complex|pulp_vectorial_complex|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_dot_product_1|pulp_vectorial_dot_product_1|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_dot_product_2|pulp_vectorial_dot_product_2|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_max|pulp_vectorial_max|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_min|pulp_vectorial_min|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_shift|pulp_vectorial_shift|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|pulp_vectorial_shuffle_pack|pulp_vectorial_shuffle_pack|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
gen|corev_directed_pulp_hwloop_debug|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_directed_pulp_hwloop_debug_ebreak|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_ebreak
gen|corev_directed_pulp_hwloop_debug_single_step|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_single_step_en
gen|corev_directed_pulp_hwloop_debug_trigger|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_trigger_basic
gen|corev_directed_pulp_hwloop_debug_trigger_with_ebreak|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_trigger_basic,debug_ebreak
gen|corev_directed_pulp_hwloop_debug_trigger_with_single_step|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_trigger_basic,debug_single_step_en
gen|corev_directed_pulp_hwloop_debug_with_int_debug_ebreak|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_ebreak
gen|corev_directed_pulp_hwloop_debug_with_int_debug_trigger|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic
gen|corev_directed_pulp_hwloop_debug_with_int_debug_trigger_and_ebreak|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_ebreak
gen|corev_directed_pulp_hwloop_debug_with_int_debug_trigger_single_step|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_single_step_en
gen|corev_directed_pulp_hwloop_debug_with_interrupt|corev_directed_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int
gen|corev_directed_pulp_hwloop_test_with_random_debug|corev_directed_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_debug_req
gen|corev_rand_debug_ebreak_xpulp|corev_rand_debug_ebreak_xpulp|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_debug_single_step_xpulp|corev_rand_debug_single_step_xpulp|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_hwloop_debug|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_hwloop_debug_ebreak|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_ebreak
gen|corev_rand_pulp_hwloop_debug_single_step|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_single_step_en
gen|corev_rand_pulp_hwloop_debug_trigger|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_trigger_basic
gen|corev_rand_pulp_hwloop_debug_trigger_with_ebreak|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_trigger_basic,debug_ebreak
gen|corev_rand_pulp_hwloop_debug_trigger_with_single_step|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_trigger_basic,debug_single_step_en
gen|corev_rand_pulp_hwloop_debug_with_int_debug_ebreak|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_ebreak
gen|corev_rand_pulp_hwloop_debug_with_int_debug_trigger|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic
gen|corev_rand_pulp_hwloop_debug_with_int_debug_trigger_and_ebreak|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_ebreak
gen|corev_rand_pulp_hwloop_debug_with_int_debug_trigger_single_step|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_single_step_en
gen|corev_rand_pulp_hwloop_debug_with_interrupt|corev_rand_pulp_hwloop_debug|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int
gen|corev_rand_pulp_hwloop_exception_debug_trigger|corev_rand_pulp_hwloop_exception|CFG_PLUSARGS="+UVM_TIMEOUT=20000000" TEST_CFG_FILE=debug_trigger_basic,gen_limit_debug_req
gen|corev_rand_pulp_hwloop_exception_single_step_debug|corev_rand_pulp_hwloop_exception|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=debug_single_step_en
gen|corev_rand_pulp_hwloop_exception_with_int_debug_trigger|corev_rand_pulp_hwloop_exception|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic
gen|corev_rand_pulp_hwloop_in_debug_rom|corev_rand_pulp_hwloop_in_debug_rom|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_hwloop_test_with_random_debug|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_debug_req
gen|corev_rand_pulp_instr_debug_ebreak_with_random_debug_req|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_ebreak,gen_rand_debug_req
gen|corev_rand_pulp_instr_debug_single_step_with_random_debug_req|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_single_step_en,gen_rand_debug_req
gen_xfail|corev_rand_pulp_instr_debug_test_with_int_and_debug_ebreak|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_ebreak
gen|corev_rand_pulp_instr_debug_test_with_int_and_debug_single_step|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_single_step_en
gen|corev_rand_pulp_instr_debug_test_with_int_and_debug_trigger|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic
gen|corev_rand_pulp_instr_debug_test_with_int_debug_trigger_and_ebreak|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_ebreak
gen|corev_rand_pulp_instr_debug_test_with_int_debug_trigger_and_single_step|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_single_step_en
gen|corev_rand_pulp_instr_debug_trigger_test|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_trigger_basic
gen|corev_rand_pulp_instr_debug_trigger_with_ebreak|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_trigger_basic,debug_ebreak
gen|corev_rand_pulp_instr_debug_trigger_with_random_debug_req|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_trigger_basic,gen_rand_debug_req
gen|corev_rand_pulp_instr_debug_trigger_with_single_step|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_trigger_basic,debug_single_step_en
gen|corev_rand_pulp_instr_ebreak_debug_test|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_ebreak
gen|corev_rand_pulp_instr_interrupt_debug_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,gen_rand_debug_req
gen|corev_rand_pulp_instr_random_debug_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_debug_req
gen|corev_rand_pulp_instr_single_step_debug_test|corev_rand_pulp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_single_step_en
gen|corev_directed_for_hwloop_covg_test|corev_directed_for_hwloop_covg_test|CFG_PLUSARGS="+UVM_TIMEOUT=60000000"
gen|corev_rand_pulp_instr_test_debug_ebreak|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_ebreak
gen|corev_rand_pulp_instr_test_debug_single_step|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=debug_single_step_en
'

# Shared FPU lanes; the func_cov programs are ISA-specific (F uses f-registers,
# Zfinx uses x-registers) and only assemble on their own config, so each config
# list below runs its own and skips the other. corev_fp_mstatus_fs_test is also
# per-config: the generator needs +enable_floating_point (F) or
# +enable_fp_in_x_regs (Zfinx) via TEST_CFG_FILE, otherwise the FP instruction
# registry is empty and the stream degrades to an unconstrained random pick.
TESTS_fpu_common='
run|fpu_bugs_test|fpu_bugs_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|illegal_fp_instr_test|illegal_fp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=100000000"
'

TESTS_pulp_fpu="$TESTS_fpu_common"'gen|corev_rand_fp_instr_test|corev_rand_fp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_instr_en
gen|corev_rand_fp_instr_sanity_test|corev_rand_fp_instr_sanity_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_instr_en
gen|corev_rand_fp_instr_data_fwd_test|corev_rand_fp_instr_data_fwd_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_instr_en
gen|corev_rand_fp_instr_mlt_cyc_test|corev_rand_fp_instr_mlt_cyc_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_instr_en
gen|corev_rand_fp_instr_w_special_ops_test|corev_rand_fp_instr_w_special_ops_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_instr_en
gen|corev_fp_mstatus_fs_test|corev_fp_mstatus_fs_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" TEST_CFG_FILE=floating_pt_instr_en
run|fpu_func_cov_improve_test|fpu_func_cov_improve_test|CFG_PLUSARGS="+UVM_TIMEOUT=100000000"
run|matmul_32b_float|matmul_32b_float|CFG_PLUSARGS="+UVM_TIMEOUT=2000000"
gen|corev_directed_for_interrupt_covg_test|corev_directed_for_interrupt_covg_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=floating_pt_instr_en
skip|zfinx_func_cov_improve_test|zfinx_func_cov_improve_test|Zfinx-only program (F-config assembler rejects x-register FP operands)
gen_xfail|corev_rand_fp_instr_debug_test_with_int_and_debug|corev_rand_fp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int
gen_xfail|corev_rand_fp_instr_debug_test_with_int_and_debug_single_step|corev_rand_fp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_single_step_en
gen_xfail|corev_rand_fp_instr_debug_test_with_int_and_debug_trigger|corev_rand_fp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic
gen_xfail|corev_rand_fp_instr_debug_test_with_int_debug_trigger_and_single_step|corev_rand_fp_instr_debug|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int,debug_trigger_basic,debug_single_step_en
'

TESTS_pulp_fpu_zfinx="$TESTS_fpu_common"'gen|corev_rand_fp_instr_test|corev_rand_fp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
gen|corev_rand_fp_instr_sanity_test|corev_rand_fp_instr_sanity_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
gen|corev_rand_fp_instr_data_fwd_test|corev_rand_fp_instr_data_fwd_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
gen|corev_rand_fp_instr_mlt_cyc_test|corev_rand_fp_instr_mlt_cyc_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
gen|corev_rand_fp_instr_w_special_ops_test|corev_rand_fp_instr_w_special_ops_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
gen|corev_fp_mstatus_fs_test|corev_fp_mstatus_fs_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
skip|fpu_func_cov_improve_test|fpu_func_cov_improve_test|F-only program (Zfinx config has no F register file)
run|zfinx_func_cov_improve_test|zfinx_func_cov_improve_test|CFG_PLUSARGS="+UVM_TIMEOUT=100000000"
xfail|interrupt_test_zfinx_cfg|interrupt_test|TEST_CFG_FILE=floating_pt_zfinx_instr_en
'

run_one() {
    local cfg=$1 kind=$2 label=$3 tc=$4 extra=$5
    local gen=""
    case $kind in gen|gen_xfail) gen="gen_corev-dv";; esac
    # Optional per-lane wall-clock cap: a TMO=<seconds> token in the extra
    # field overrides the 2400s default (used by lanes known to run to the
    # timeout - same TIMEOUT verdict, less gate wall clock).
    local tmo=2400
    case "$extra" in
        *TMO=*) tmo=${extra##*TMO=}; tmo=${tmo%% *}
                extra=${extra//TMO=$tmo/};;
    esac
    local t0=$(date +%s)
    eval timeout $tmo make $gen test COREV=YES TEST=$tc CV_CORE=cv32e40p \
        CFG=$cfg COREV=1 SIMULATOR=vsim COMP=0 USE_ISS=YES ISS=GVSOC COV=NO \
        SEED=${QV_SEED:-1} GEN_START_INDEX=0 RUN_INDEX=0 TEST_CFG_FILE= ENABLE_TRACE_LOG=NO \
        $extra \
        > "$OUT/$cfg/$label.log" 2>&1
    local rc=$? t1=$(date +%s)
    local verdict
    if [ $rc -eq 0 ] && grep -q "SIMULATION PASSED" "$OUT/$cfg/$label.log"; then
        verdict=PASS; PASS=$((PASS+1))
    elif [ "$kind" = xfail ] || [ "$kind" = gen_xfail ]; then
        verdict=KNOWN_FAIL; XFAIL=$((XFAIL+1))
    elif [ $rc -eq 124 ]; then
        verdict=TIMEOUT; FAIL=$((FAIL+1))
    elif ! grep -q "SIMULATION PASSED\|SIMULATION FAILED" "$OUT/$cfg/$label.log"; then
        verdict=NO_SIM; FAIL=$((FAIL+1))
    else
        verdict=FAIL; FAIL=$((FAIL+1))
    fi
    local mm=$(grep -o 'Total Reference model mismatches *= *[0-9]*' "$OUT/$cfg/$label.log" | grep -o '[0-9]*$' | tail -1)
    echo "$cfg/$label $verdict rc=$rc wall=$((t1-t0))s mismatches=${mm:-na}" >> "$OUT/SUMMARY.txt"
}

sweep_cfg() {
    local cfg=$1 list=$2
    mkdir -p "$OUT/$cfg" || { echo "cannot create $OUT/$cfg" >> "$OUT/SUMMARY.txt"; FAIL=$((FAIL+1)); return 1; }
    echo "=== CFG=$cfg compile ===" >> "$OUT/SUMMARY.txt"
    local t0=$(date +%s)
    make comp comp_corev-dv CV_CORE=cv32e40p CFG=$cfg SIMULATOR=vsim \
        USE_ISS=YES ISS=GVSOC COV=NO \
        > "$OUT/$cfg/comp.log" 2>&1
    local rc=$?
    echo "comp rc=$rc wall=$(( $(date +%s) - t0 ))s" >> "$OUT/SUMMARY.txt"
    if [ $rc -ne 0 ]; then
        echo "COMP_FAIL $cfg - lane skipped (see $cfg/comp.log)" >> "$OUT/SUMMARY.txt"
        FAIL=$((FAIL+1))
        return 1
    fi
    while IFS='|' read -r kind label tc extra; do
        [ -z "$kind" ] && continue
        case $kind in
            skip)
                echo "$cfg/$label SKIP ($extra)" >> "$OUT/SUMMARY.txt"
                SKIP=$((SKIP+1)) ;;
            gen|gen_xfail|run|xfail)
                run_one "$cfg" "$kind" "$label" "$tc" "$extra" ;;
            *)
                echo "$cfg: malformed entry kind '$kind' (label '$label')" >> "$OUT/SUMMARY.txt"
                FAIL=$((FAIL+1)) ;;
        esac
    done <<< "$list"
}

for cfg in $CFGS; do
    case $cfg in
        default)        sweep_cfg default        "$TESTS_default" ;;
        pulp)           sweep_cfg pulp           "$TESTS_pulp" ;;
        pulp_fpu)       sweep_cfg pulp_fpu       "$TESTS_pulp_fpu" ;;
        pulp_fpu_zfinx) sweep_cfg pulp_fpu_zfinx "$TESTS_pulp_fpu_zfinx" ;;
        *)
            echo "unknown cfg: $cfg" >> "$OUT/SUMMARY.txt"
            FAIL=$((FAIL+1)) ;;
    esac
done

echo "=== TOTAL: $PASS PASS / $FAIL FAIL / $XFAIL KNOWN_FAIL / $SKIP SKIP ===" >> "$OUT/SUMMARY.txt"
RECAP=$(grep -E "^[a-z_]+/.* (FAIL|TIMEOUT|NO_SIM) |^COMP_FAIL |malformed entry|^unknown cfg" "$OUT/SUMMARY.txt")
[ -n "$RECAP" ] && { echo "--- non-PASS (known-fail and skips excluded):"; echo "$RECAP"; } >> "$OUT/SUMMARY.txt"
echo "quick_val end: $(date -Iseconds)" >> "$OUT/SUMMARY.txt"
echo "results in $OUT"
cat "$OUT/SUMMARY.txt"
[ $FAIL -eq 0 ]
