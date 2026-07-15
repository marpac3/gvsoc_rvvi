#!/bin/bash
# Step-and-compare validation matrix for the GVSOC RVVI bridge.
#
# Runs the formatter unit test plus one UVM co-simulation lane per coverage
# axis (zfinx build/ABI, FP traps, FP arithmetic, hardware loops, baseline,
# interrupts) and the RVVI-TEXT conformance check. Each lane records
# PASS/FAIL, the per-category mismatch counts, the layout-canary and
# retire-ordering tripwire hits, and the trace line counts in SUMMARY.txt.
#
# Usage:   test/validation_matrix.sh [output-dir]
# Expects: simulator environment already set up (see docs/TESTING.md) and a
#          compiled toolchain; run from anywhere inside the core-v-verif tree.
#
# Known-open expectations (do not read every FAIL as a regression):
#   - matmul_32b_float FAILs on an ISS trap-redirect desync after an illegal
#     CSR read (open ISS issue; PC/GPR/CSR mismatches at retire ~780).
#   - interrupt_test FAILs when the program reads the testbench random-number
#     device (0x15001000): the reference model has no entropy source
#     (upstream gvsoc issue #18198, wontfix). Informed injection itself is
#     exercised and must show inject lines with rc=1.

set -o pipefail

SELF_DIR=$(cd "$(dirname "$0")" && pwd)
SUB=$(dirname "$SELF_DIR")
CVV=$(cd "$SUB/../.." && pwd)
UVMT="$CVV/cv32e40p/sim/uvmt"
OUT=${1:-/tmp/gvsoc_rvvi_matrix_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"

if ! command -v vsim > /dev/null 2>&1; then
    echo "vsim not in PATH - set up the simulator environment first (docs/TESTING.md)" >&2
    exit 1
fi

cd "$UVMT" || exit 1
: > "$OUT/SUMMARY.txt"
echo "matrix start: $(date -Iseconds)" >> "$OUT/SUMMARY.txt"

echo "=== unit: formatter grammar (no simulator license) ===" >> "$OUT/SUMMARY.txt"
make -C "$SUB" test > "$OUT/unit.log" 2>&1
echo "unit rc=$? ($(grep -c PASS "$OUT/unit.log") PASS lines)" >> "$OUT/SUMMARY.txt"

run() {
    local name=$1 test=$2; shift 2
    local t0=$(date +%s)
    timeout 2400 make test TEST="$test" USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES \
        RVVI_TEXT_TRACE="$OUT/${name}_trace" "$@" > "$OUT/$name.log" 2>&1
    local rc=$? t1=$(date +%s)
    echo "=== $name rc=$rc wall=$((t1-t0))s end=$(date -Iseconds) $(grep -o 'SIMULATION PASSED\|SIMULATION FAILED' "$OUT/$name.log" | head -1)" >> "$OUT/SUMMARY.txt"
    grep -E 'Total Reference model mismatches|phase realigns|retires total|volatile syncs|UVM_ERROR :' "$OUT/$name.log" | tail -5 >> "$OUT/SUMMARY.txt"
    echo "$name canary: $(grep -c 'layout canary OK' "$OUT/$name.log") OK / $(grep -c 'canary FAILED' "$OUT/$name.log") FAILED" >> "$OUT/SUMMARY.txt"
    echo "$name ordering-tripwire: $(grep -c 'retire ordering broken' "$OUT/$name.log")" >> "$OUT/SUMMARY.txt"
    echo "$name mismatches PC/GPR/FPR/CSR/INSN: $(grep -c 'RVVI Mismatch: PC' "$OUT/$name.log")/$(grep -c 'RVVI Mismatch: GPR' "$OUT/$name.log")/$(grep -c 'RVVI Mismatch: FPR' "$OUT/$name.log")/$(grep -c 'RVVI Mismatch: CSR' "$OUT/$name.log")/$(grep -c 'RVVI Mismatch: INSN' "$OUT/$name.log")" >> "$OUT/SUMMARY.txt"
    echo "$name dropped-flush-rows: $(grep -c 'dropped trap-redirect flush row' "$OUT/$name.log")" >> "$OUT/SUMMARY.txt"
    [ -d "$OUT/${name}_trace" ] && wc -l "$OUT/${name}_trace"/*.rvvi 2>/dev/null | head -3 >> "$OUT/SUMMARY.txt"
}

echo "=== zfinx lane (second .so, ABI canary) ===" >> "$OUT/SUMMARY.txt"
run zfx zfinx_func_cov_improve_test CFG=pulp_fpu_zfinx
diff <(awk '{print $1,$2,$3}' "$OUT/zfx_trace/dut.rvvi") \
     <(awk '{print $1,$2,$3}' "$OUT/zfx_trace/ref.rvvi") > /dev/null 2>&1 \
  && echo "zfx dut-vs-ref (pc/insn cols): identical" >> "$OUT/SUMMARY.txt" \
  || echo "zfx dut-vs-ref (pc/insn cols): DIFFER" >> "$OUT/SUMMARY.txt"

echo "=== zfinx lane, volatile-counter sync off ===" >> "$OUT/SUMMARY.txt"
CV_RVVI_VOLATILE_CSR_SYNC=0 run zfx_nvs zfinx_func_cov_improve_test CFG=pulp_fpu_zfinx COMP=NO

echo "=== FPU lanes ===" >> "$OUT/SUMMARY.txt"
run ifp illegal_fp_instr_test CFG=pulp_fpu
run mm matmul_32b_float CFG=pulp_fpu COMP=NO

echo "=== pulp + default lanes ===" >> "$OUT/SUMMARY.txt"
run hwl pulp_hardware_loop CFG=pulp
run par pulp_hardware_loop_interrupt_test CFG=pulp COMP=NO USER_RUN_FLAGS=+rvvi_informed_irq
grep -c "informed-inject" "$OUT/par.log" | sed 's/^/par informed-inject lines: /' >> "$OUT/SUMMARY.txt"
run hello hello-world

echo "=== hello lane, volatile-counter sync off ===" >> "$OUT/SUMMARY.txt"
CV_RVVI_VOLATILE_CSR_SYNC=0 run hello_nvs hello-world COMP=NO

echo "=== interrupt lane (informed injection) ===" >> "$OUT/SUMMARY.txt"
run irq interrupt_test COMP=NO USER_RUN_FLAGS=+rvvi_informed_irq

echo "=== RVVI-TEXT conformance ===" >> "$OUT/SUMMARY.txt"
for d in zfx ifp hello; do
    make check-rvvi RVVI_TRACE_DIR="$OUT/${d}_trace" > "$OUT/check_$d.log" 2>&1
    echo "check-rvvi $d: rc=$?" >> "$OUT/SUMMARY.txt"
done

echo "matrix end: $(date -Iseconds)" >> "$OUT/SUMMARY.txt"
echo "MATRIX_DONE" >> "$OUT/SUMMARY.txt"
echo "results in $OUT"
