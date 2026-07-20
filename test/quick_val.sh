#!/bin/bash
# Quick validation sweep for the GVSOC RVVI co-simulation: one run of every
# test type in each testbench configuration, instead of a full regression
# (~82 runs vs 651+; same breadth of stimulus types, a fraction of the wall
# clock). Generated tests run with SEED=1 so two sweeps on the same build
# are directly comparable.
#
# Test lists mirror cv32e40p_full_covg_no_pulp.yaml (CFG=default) and the
# cv32e40pv2_{xpulp,fpu,fpu_zfinx}_instr yamls (PULP/FPU configs); the
# interrupt lanes come from the same no_pulp yaml (CFG=default) and from
# cv32e40pv2_interrupt_debug_short.yaml (CFG=pulp, gen_rand_int aliases).
#
# Usage:   test/quick_val.sh [output-dir] [cfg ...]
#            cfg list optional (default: default pulp pulp_fpu pulp_fpu_zfinx)
# Expects: simulator environment already set up (see docs/TESTING.md) and a
#          compiled toolchain; run from anywhere inside the core-v-verif tree.
# With GVSOC_ISS_V2=YES in the environment every run uses the iss_v2
# reference core.
#
# Exit code: 0 only when no lane reports FAIL/TIMEOUT/NO_SIM. Known-fail
# lanes (kind xfail) and skips do not affect it.
#
# Known-open expectations:
#   - corev_rand_fp_instr_* are SKIPPED: riscv-dv aborts generation with a
#     constraint contradiction on pulp_fpu/pulp_fpu_zfinx (before any sim).
#   - corev_rand_pulp_{,hwloop_}illegal_instr_test run the base rand TEST with
#     TEST_CFG_FILE=insert_illegal_instr (regress-yaml alias); they currently
#     share the base lanes' open FAIL.
#   - debug-mode tests (debug_hwloop_test, pulp_hardware_loop_debug_test)
#     exercise a debug flow the reference model does not implement
#     (ebreak-enters-debug and single-step); they are marked xfail and
#     reported as KNOWN_FAIL.
#   - fpu_bugs_test (xfail): the remaining mismatches are the on-hold
#     underflow accrual difference (fflags read back into GPRs), one
#     subnormal-boundary value and the control-flow knock-on of both; the
#     FMA lanes also carry a double-rounding artefact (flexfloat computes
#     fma in double, the RTL fuses at single precision).
#   - corev_rand_pulp_hwloop_count_range_test (gen_xfail): random hwloop
#     body with IRQ noise deadlocks the v2 exec loop - a load-use scoreboard
#     bit is orphaned around an IRQ redirect and the next reader stalls
#     forever. The clean fix (pipeline squash on redirect) lives in the
#     common iss_v2 exec/LSU layer and is on hold.
#   - interrupt lanes: corev_rand_interrupt and interrupt_bootstrap PASS;
#     the other default-config IRQ lanes and the pulp gen_rand_int aliases
#     share open reference-model gaps, all confirmed on the v1 bridge too
#     (never measured before: the fast2 regression list has its interrupt
#     entries commented out). Signatures: IRQ taken at a different retire
#     than the RTL (interrupt_test, _exception, _nested), WFI wake-up missed
#     (_wfi, _wfi_mem_stress), hwloop+IRQ commit starvation (pulp aliases).

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
echo "reference core: $([ -n "$GVSOC_ISS_V2" ] && echo iss_v2 || echo v1)" >> "$OUT/SUMMARY.txt"
echo "configs: $CFGS" >> "$OUT/SUMMARY.txt"

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
gen|corev_rand_interrupt_wfi_mem_stress|corev_rand_interrupt_wfi_mem_stress|
gen|corev_rand_interrupt_exception|corev_rand_interrupt_exception|
gen|corev_rand_interrupt_nested|corev_rand_interrupt_nested|
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
'

TESTS_pulp='
gen|corev_rand_pulp_hwloop_test|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_instr_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000"
gen|corev_rand_pulp_simd_instr_test|corev_rand_pulp_simd_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
gen|corev_rand_pulp_mac_instr_test|corev_rand_pulp_mac_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
gen|corev_rand_pulp_hwloop_exception|corev_rand_pulp_hwloop_exception|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_illegal_instr_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=5000000" TEST_CFG_FILE=insert_illegal_instr
gen|corev_rand_pulp_hwloop_illegal_instr_test|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=25000000" TEST_CFG_FILE=insert_illegal_instr
gen|corev_rand_pulp_with_priv_instr_test|corev_rand_pulp_with_priv_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000"
gen|corev_rand_pulp_instr_interrupt_test|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=10000000" TEST_CFG_FILE=gen_rand_int
gen|corev_rand_pulp_hwloop_interrupt_test|corev_rand_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int
gen|corev_directed_pulp_hwloop_test_with_interrupt|corev_directed_pulp_hwloop_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" TEST_CFG_FILE=gen_rand_int
gen_xfail|corev_rand_pulp_hwloop_count_range_test|corev_rand_pulp_hwloop_count_range_test|CFG_PLUSARGS="+UVM_TIMEOUT=30000000" VSIM_USER_FLAGS=+skip_sampling_uvme_rv32x_hwloop_covg
gen|tb_hack_obi_gnt_stalls|corev_rand_pulp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" VSIM_USER_FLAGS="+random_instr_stall +random_data_stall +tb_hack_1_obi_gnt_signal"
run|pulp_hardware_loop|pulp_hardware_loop|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" VSIM_USER_FLAGS="+skip_sampling_uvme_rv32x_hwloop_covg +fixed_data_gnt_stall=3"
run|pulp_hardware_loop_interrupt_test|pulp_hardware_loop_interrupt_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
xfail|pulp_hardware_loop_debug_test|pulp_hardware_loop_debug_test|
xfail|debug_hwloop_test|debug_hwloop_test|
run|custom_opcode_illegal_test|custom_opcode_illegal_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|cv32e40pv2_illegal_ro_csr_access_test|cv32e40pv2_illegal_ro_csr_access_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|cv32e40p_csr_access_test|cv32e40p_csr_access_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
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
'

# Shared FPU lanes; the func_cov programs are ISA-specific (F uses f-registers,
# Zfinx uses x-registers) and only assemble on their own config, so each config
# list below runs its own and skips the other. corev_fp_mstatus_fs_test is also
# per-config: the generator needs +enable_floating_point (F) or
# +enable_fp_in_x_regs (Zfinx) via TEST_CFG_FILE, otherwise the FP instruction
# registry is empty and the stream degrades to an unconstrained random pick.
TESTS_fpu_common='
skip|corev_rand_fp_instr_test|corev_rand_fp_instr_test|riscv-dv constraint contradiction (generation aborts)
skip|corev_rand_fp_instr_sanity_test|corev_rand_fp_instr_sanity_test|riscv-dv constraint contradiction (generation aborts)
skip|corev_rand_fp_instr_data_fwd_test|corev_rand_fp_instr_data_fwd_test|riscv-dv constraint contradiction (generation aborts)
skip|corev_rand_fp_instr_mlt_cyc_test|corev_rand_fp_instr_mlt_cyc_test|riscv-dv constraint contradiction (generation aborts)
skip|corev_rand_fp_instr_w_special_ops_test|corev_rand_fp_instr_w_special_ops_test|riscv-dv constraint contradiction (generation aborts)
xfail|fpu_bugs_test|fpu_bugs_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000"
run|illegal_fp_instr_test|illegal_fp_instr_test|CFG_PLUSARGS="+UVM_TIMEOUT=100000000"
'

TESTS_pulp_fpu="$TESTS_fpu_common"'gen|corev_fp_mstatus_fs_test|corev_fp_mstatus_fs_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" TEST_CFG_FILE=floating_pt_instr_en
run|fpu_func_cov_improve_test|fpu_func_cov_improve_test|CFG_PLUSARGS="+UVM_TIMEOUT=100000000"
skip|zfinx_func_cov_improve_test|zfinx_func_cov_improve_test|Zfinx-only program (F-config assembler rejects x-register FP operands)
'

TESTS_pulp_fpu_zfinx="$TESTS_fpu_common"'gen|corev_fp_mstatus_fs_test|corev_fp_mstatus_fs_test|CFG_PLUSARGS="+UVM_TIMEOUT=1000000" TEST_CFG_FILE=floating_pt_zfinx_instr_en
skip|fpu_func_cov_improve_test|fpu_func_cov_improve_test|F-only program (Zfinx config has no F register file)
run|zfinx_func_cov_improve_test|zfinx_func_cov_improve_test|CFG_PLUSARGS="+UVM_TIMEOUT=100000000"
'

run_one() {
    local cfg=$1 kind=$2 label=$3 tc=$4 extra=$5
    local gen=""
    case $kind in gen|gen_xfail) gen="gen_corev-dv";; esac
    local t0=$(date +%s)
    eval timeout 2400 make $gen test COREV=YES TEST=$tc CV_CORE=cv32e40p \
        CFG=$cfg COREV=1 SIMULATOR=vsim COMP=0 USE_ISS=YES ISS=GVSOC COV=NO \
        SEED=1 GEN_START_INDEX=0 RUN_INDEX=0 TEST_CFG_FILE= ENABLE_TRACE_LOG=NO \
        ${GVSOC_ISS_V2:+GVSOC_ISS_V2=$GVSOC_ISS_V2} $extra \
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
        ${GVSOC_ISS_V2:+GVSOC_ISS_V2=$GVSOC_ISS_V2} > "$OUT/$cfg/comp.log" 2>&1
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
