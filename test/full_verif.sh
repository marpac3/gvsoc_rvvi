#!/bin/bash
# Full CV32E40Pv2 regression with Questa code coverage and GVSOC co-simulation.
#
# Runs the v2 regression perimeter (374 lanes over the three base TB configs,
# plus ~36 FPU-instruction lanes over the four raised-latency configs when
# those are selected) as a parallel pool instead of the sequential script
# cv_regress emits, collects one UCDB per lane, and merges the passing ones
# into per-config and cross-config coverage databases plus HTML / module /
# covergroup reports.
#
# Perimeter (cv32e40p/regress/README.md):
#   pulp                    xpulp_instr + interrupt_debug                   116
#   pulp_fpu                xpulp_instr + interrupt_debug + fpu_instr       129
#   pulp_fpu_zfinx          xpulp_instr + interrupt_debug + fpu_instr_zfinx 129
#   pulp_fpu_1cyclat        fpu_instr (FPU_*_LAT=1 APU write-back timing)     9
#   pulp_fpu_2cyclat        fpu_instr (FPU_*_LAT=2)                           9
#   pulp_fpu_zfinx_1cyclat  fpu_instr_zfinx (LAT=1)                           9
#   pulp_fpu_zfinx_2cyclat  fpu_instr_zfinx (LAT=2)                           9
# The latency configs are opt-in via FV_CFGS: the default config list stays
# the three base ones.
#
# Usage:   test/full_verif.sh [output-dir]
#
# Environment knobs (all optional):
#   FV_JOBS=16              parallel lanes (16 default, 24 is the sane ceiling
#                           on a 128-core box shared with other users)
#   FV_CFGS="pulp ..."      config list (default: the three above)
#   FV_COV=YES|NO           coverage build + collection (default YES). NO gives
#                           a reference run for measuring coverage overhead;
#                           the two modes need SEPARATE output dirs, the work
#                           library is built differently and is not shared.
#   FV_SEED_MODE=random|<n> per-lane seed. random (default) is what cv_regress
#                           emits; a fixed number makes two campaigns directly
#                           comparable and is applied as <n>+RUN_INDEX, so the
#                           lanes of one test stay distinct from each other.
#   FV_TIMEOUT=2400         default per-lane wall-clock cap in seconds
#   FV_TMO_FILE=<file>      per-lane timeout overrides, one "key seconds" per
#                           line, key being either "cfg/label" or a bare TEST
#                           name. Takes precedence over the built-in table.
#   FV_UVMTMO_FILE=<file>   per-lane UVM phase-timeout overrides (SIM time,
#                           ns), one "key ns" per line, same key format as
#                           FV_TMO_FILE. Appends CFG_PLUSARGS="+UVM_TIMEOUT=<ns>"
#                           to the lane's make command (make: last assignment
#                           wins, replacing the regress-yaml value, which for
#                           these tests carries only +UVM_TIMEOUT). Keep the
#                           value BELOW the TB watchdog (100e6 ns) or the
#                           uvm_fatal TIMEOUT fires first.
#   FV_XFAIL_FILE=<file>    known-open lanes, one "cfg/label" or bare TEST name
#                           per line (# comments allowed). They are reported as
#                           KNOWN_FAIL, kept out of the coverage merge, and do
#                           not affect the exit code. Empty by default: until a
#                           baseline campaign has run, every failure is
#                           unexpected.
#   FV_TRACE=YES|NO         CV32E40P_TRACE_EXECUTION core trace logs. Compile
#                           time flag, so it applies to the build too. Default
#                           YES = same build as the quick_val gate; NO saves a
#                           large amount of disk but is a different object.
#   FV_FILTER=<ere>         only run lanes whose "cfg/label" matches this
#                           extended regex. Manifests are still generated in
#                           full, so run indices stay identical to a full
#                           campaign. Use it to re-run a subset after a fix.
#   FV_DRY=1                generate the manifests, print the plan, run nothing
#   FV_PY=<python>          interpreter with jinja2 for cv_regress (default:
#                           micromamba run -n gvsoc_env_3_12 python)
#
# Examples:
#   test/full_verif.sh                                   # full campaign, W=16
#   FV_JOBS=24 test/full_verif.sh /data2/$USER/fv_run1    # more parallelism
#   FV_DRY=1 test/full_verif.sh                          # plan only
#   FV_CFGS=pulp FV_COV=NO FV_SEED_MODE=1 test/full_verif.sh /data2/$USER/ref
#
# Prerequisites: nothing. The script loads its own simulator environment
# (Questa 2025.3, pinned) and the CoreV toolchain; it only needs the GVSOC
# bridge libraries to be already built (see docs/TESTING.md).
#
# Output layout under <output-dir>:
#   manifest/<cfg>.lanes    parsed lane table (tab separated)
#   manifest/<cfg>.sh       raw cv_regress output, kept for audit
#   logs/<cfg>/<label>.log  per-lane make output
#   results/                CV_RESULTS tree (vsim_results/<cfg>/...)
#   cov/<cfg>/              per-config ucdb.list + merged.ucdb
#   cov/merged_all.ucdb     cross-config merge
#   reports/                HTML + per-module + per-covergroup reports
#   SUMMARY.txt             one line per lane, verdicts and wall clock
#
# Exit code: 0 only when every lane reported PASS or KNOWN_FAIL. Any unexpected
# FAIL / TIMEOUT / NO_SIM / NO_UCDB, any failed build, and any coverage merge
# error make it non-zero.

set -o pipefail

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
# Questa is pinned to 2025.3: that is the version the validated quick_val gate
# ran on (banner in the gate comp.log) and the one docs/TESTING.md documents.
# UCDBs from different Questa releases must not be mixed in a merge, so the
# same version has to cover build, run and merge.
QUESTA_MODULE=questa/2025.3
QUESTA_EXPECT=2025.3
# Same reasoning for the cross-compiler: it decides the stimulus binaries, so
# it is pinned to what the gate used rather than inherited from the caller
# (interactive profiles here export a stale, non-existent path).
TOOLCHAIN_PIN=/opt/riscv/corev-openhw-gcc-modded-v0.1
TOOLCHAIN_PREFIX=riscv64-unknown-elf-

setup_env() {
    # No `set -u` around this: the module init scripts dereference unset vars.
    if [ -r /etc/profile.d/modules.sh ]; then
        . /etc/profile.d/modules.sh
    fi
    if command -v module > /dev/null 2>&1; then
        module load "$QUESTA_MODULE" 2>&1 | grep -v '^$' >&2
    fi
    export CV_SIMULATOR=vsim
    if [ -n "$CV_SW_TOOLCHAIN" ] && [ "$CV_SW_TOOLCHAIN" != "$TOOLCHAIN_PIN" ]; then
        echo "full_verif: overriding inherited CV_SW_TOOLCHAIN=$CV_SW_TOOLCHAIN" >&2
    fi
    export CV_SW_TOOLCHAIN=$TOOLCHAIN_PIN
    export CV_SW_PREFIX=$TOOLCHAIN_PREFIX

    if ! command -v vsim > /dev/null 2>&1; then
        echo "full_verif: vsim not on PATH after 'module load $QUESTA_MODULE'" >&2
        return 1
    fi
    local have
    have=$(vsim -version 2>/dev/null | grep -o '[0-9]\{4\}\.[0-9][^ ]*' | head -1)
    if [ "$have" != "$QUESTA_EXPECT" ]; then
        echo "full_verif: expected Questa $QUESTA_EXPECT, got '${have:-unknown}'" >&2
        echo "            refusing to run: UCDBs from mixed versions cannot merge" >&2
        return 1
    fi
    if [ ! -x "$CV_SW_TOOLCHAIN/bin/${CV_SW_PREFIX}gcc" ]; then
        echo "full_verif: no ${CV_SW_PREFIX}gcc under $CV_SW_TOOLCHAIN/bin" >&2
        return 1
    fi
    return 0
}

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
SELF_DIR=$(cd "$(dirname "$0")" && pwd)
SUB=$(dirname "$SELF_DIR")
CVV=$(cd "$SUB/../.." && pwd)
UVMT="$CVV/cv32e40p/sim/uvmt"
REGRESS_DIR="$CVV/cv32e40p/regress"

OUT=${1:-/data2/$USER/fullverif_$(date +%Y%m%d_%H%M%S)}
JOBS=${FV_JOBS:-16}
CFGS=${FV_CFGS:-"pulp pulp_fpu pulp_fpu_zfinx"}
COV=${FV_COV:-YES}
SEED_MODE=${FV_SEED_MODE:-random}
TIMEOUT_DEF=${FV_TIMEOUT:-2400}
TMO_FILE=${FV_TMO_FILE:-}
UVMTMO_FILE=${FV_UVMTMO_FILE:-}
XFAIL_FILE=${FV_XFAIL_FILE:-}
TRACE=${FV_TRACE:-YES}
FILTER=${FV_FILTER:-}
DRY=${FV_DRY:-0}
PY=${FV_PY:-"micromamba run -n gvsoc_env_3_12 python"}

RESULTS="$OUT/results"
SUMMARY="$OUT/SUMMARY.txt"

# Regression yaml sets and the extra test_cfg each config needs. Kept as a
# lookup rather than a loop body so adding a config is a one-line change.
yamls_for_cfg() {
    case $1 in
        pulp)           echo "cv32e40pv2_xpulp_instr.yaml cv32e40pv2_interrupt_debug.yaml" ;;
        pulp_fpu)       echo "cv32e40pv2_xpulp_instr.yaml cv32e40pv2_interrupt_debug.yaml cv32e40pv2_fpu_instr.yaml" ;;
        pulp_fpu_zfinx) echo "cv32e40pv2_xpulp_instr.yaml cv32e40pv2_interrupt_debug.yaml cv32e40pv2_fpu_instr_zfinx.yaml" ;;
        # Raised-latency FPU configs (FPU_ADDMUL_LAT / FPU_OTHERS_LAT = 1, 2):
        # FPU instruction lanes only - the xpulp/interrupt yamls do not
        # exercise the APU write-back timing these configs exist to stress.
        pulp_fpu_1cyclat|pulp_fpu_2cyclat)             echo "cv32e40pv2_fpu_instr.yaml" ;;
        pulp_fpu_zfinx_1cyclat|pulp_fpu_zfinx_2cyclat) echo "cv32e40pv2_fpu_instr_zfinx.yaml" ;;
        *)              return 1 ;;
    esac
}

add_test_cfg_for_cfg() {
    case $1 in
        pulp_fpu)       echo "floating_pt_instr_en" ;;
        pulp_fpu_zfinx) echo "floating_pt_zfinx_instr_en" ;;
        pulp_fpu_1cyclat|pulp_fpu_2cyclat)             echo "floating_pt_instr_en" ;;
        pulp_fpu_zfinx_1cyclat|pulp_fpu_zfinx_2cyclat) echo "floating_pt_zfinx_instr_en" ;;
        *)              echo "" ;;
    esac
}

# ---------------------------------------------------------------------------
# Phase 1 - manifest
# ---------------------------------------------------------------------------
# cv_regress renders a sequential bash script. We harvest the make lines out of
# it rather than running it: the emitted script discards output (`>& /dev/null`)
# and never looks at make's exit code, so a stale log from a previous run reads
# as a pass.

gen_manifest() {
    local cfg=$1
    local yamls add_cfg args f
    yamls=$(yamls_for_cfg "$cfg") || { echo "full_verif: unknown cfg '$cfg'" >&2; return 1; }
    add_cfg=$(add_test_cfg_for_cfg "$cfg")

    args=()
    for f in $yamls; do args+=(-f "$f"); done
    args+=(--simulator vsim --cfg "$cfg" --iss GVSOC --results "$RESULTS")
    [ -n "$add_cfg" ] && args+=(--add_test_cfg "$add_cfg")
    [ "$COV" = YES ] && args+=(--cov)

    local raw="$OUT/manifest/$cfg.sh"
    ( cd "$CVV/bin" && $PY ./cv_regress --sh "${args[@]}" -o "$raw" ) \
        > "$OUT/manifest/$cfg.cv_regress.log" 2>&1
    if [ ! -s "$raw" ]; then
        echo "full_verif: cv_regress produced nothing for $cfg (see $OUT/manifest/$cfg.cv_regress.log)" >&2
        return 1
    fi
    parse_manifest "$cfg" "$raw" > "$OUT/manifest/$cfg.lanes"
}

# Turn the rendered script into a lane table. One record per lane:
#   cfg <tab> label <tab> test <tab> test_cfg_name <tab> run_index <tab> command
parse_manifest() {
    local cfg=$1 raw=$2
    # LC_ALL=C so the sort below collates the same way GNU make's $(sort) does.
    LC_ALL=C awk -v cfg="$cfg" -v seed="$SEED_MODE" -v cov="$COV" -v trace="$TRACE" '
    function shellsplit_sorted(s,   n, i, a, out, seen, k, tmp, j) {
        # sort -u over the comma/plus/space separated test_cfg list
        # (reproduces Common.mk:275-287, which decides the run dir and the
        # UCDB file name)
        gsub(/[,+]/, " ", s)
        n = split(s, a, /[ \t]+/)
        k = 0
        for (i = 1; i <= n; i++) {
            if (a[i] == "" || (a[i] in seen)) continue
            seen[a[i]] = 1
            out[++k] = a[i]
        }
        for (i = 1; i < k; i++)
            for (j = i + 1; j <= k; j++)
                if (out[j] < out[i]) { tmp = out[i]; out[i] = out[j]; out[j] = tmp }
        s = ""
        for (i = 1; i <= k; i++) s = (s == "" ? out[i] : s "__" out[i])
        return s
    }
    /^make / && / TEST=/ {
        cmd = $0
        # The template appends "  >& /dev/null;" - keeping it would blank every
        # per-lane log and turn every verdict into NO_SIM.
        sub(/[ \t]*>&[ \t]*\/dev\/null[ \t]*;?[ \t]*$/, "", cmd)

        # Lanes must not recompile: comp is serialized once per config and the
        # `opt` target is .PHONY, so a lane that runs it would rewrite the work
        # library other lanes are simulating out of.
        if (cmd !~ /(^| )COMP=0( |$)/) {
            print "full_verif: lane without COMP=0, refusing: " cmd > "/dev/stderr"
            bad = 1
            next
        }

        test = ""; tcfg = ""
        if (match(cmd, /(^| )TEST=[^ ]+/)) {
            test = substr(cmd, RSTART, RLENGTH); sub(/^ /, "", test); sub(/^TEST=/, "", test)
        }
        if (match(cmd, /TEST_CFG_FILE="[^"]*"/)) {
            tcfg = substr(cmd, RSTART, RLENGTH); sub(/^TEST_CFG_FILE="/, "", tcfg); sub(/"$/, "", tcfg)
        }
        tcfg = shellsplit_sorted(tcfg)

        # Renumber RUN_INDEX per (cfg, test). Two things force this:
        #  - two yaml keys in the base perimeter emit the same TEST with the
        #    same (empty) test_cfg and RUN_INDEX=0, so they share a run dir and
        #    a UCDB filename and the second silently overwrites the first;
        #  - gen_corev-dv writes its generated program to a directory keyed by
        #    TEST alone (vsim.mk:488), naming it <TEST>_<GEN_START_INDEX>.S, so
        #    concurrent lanes of one TEST would otherwise clobber each others
        #    stimulus regardless of their test_cfg.
        # GEN_START_INDEX must track RUN_INDEX: the hex/elf the run consumes are
        # <TEST>_<RUN_INDEX>.* (Common.mk:485) and the generator only produces
        # the index it was told to start at.
        idx = ridx[test]++
        sub(/(^| )RUN_INDEX=[0-9]+/,       " RUN_INDEX=" idx, cmd)
        sub(/(^| )GEN_START_INDEX=[0-9]+/, " GEN_START_INDEX=" idx, cmd)

        # A fixed seed is applied as <seed>+RUN_INDEX. cov.tcl names the UCDB
        # test record <TEST><_test_cfg>__<CFG>__<seed>, with no run index, so a
        # flat seed makes the duplicated TEST above collide in that namespace:
        # vcover merge collapses the two records into one MERGE_ERROR record
        # (message 6854) and still exits 0. The offset also stops those two
        # lanes from simulating byte-identical stimulus.
        if (seed != "random") sub(/(^| )SEED=[^ ]+/, " SEED=" (seed + idx), cmd)

        # Last assignment on a make command line wins, so these override
        # whatever cv_regress emitted without having to rewrite it.
        cmd = cmd " COV=" cov " ENABLE_TRACE_LOG=" trace

        label = test (tcfg == "" ? "" : "_" tcfg) "_" idx
        # "-" stands in for an absent test_cfg: tab is an IFS whitespace
        # character, so `read` would collapse an empty field and shift the rest.
        printf "%s\t%s\t%s\t%s\t%d\t%s\n", cfg, label, test, (tcfg == "" ? "-" : tcfg), idx, cmd
    }
    END { if (bad) exit 1 }
    ' "$raw"
}

# ---------------------------------------------------------------------------
# Phase 2 - build
# ---------------------------------------------------------------------------
# Serialized on purpose. comp and comp_corev-dv both write into
# $CV_RESULTS/vsim_results/<cfg>/{work,corev-dv/work}; running two configs at
# once is safe (different directories) but the vlog/vopt of one config already
# saturates several cores, and a build failure has to stop the campaign before
# any lane starts.
comp_cfg() {
    local cfg=$1
    local log="$OUT/logs/$cfg/comp.log"
    mkdir -p "$(dirname "$log")"
    echo "=== CFG=$cfg build (COV=$COV) ===" >> "$SUMMARY"
    local t0 rc
    t0=$(date +%s)
    ( cd "$UVMT" && make comp comp_corev-dv \
        CV_CORE=cv32e40p CFG="$cfg" SIMULATOR=vsim \
        USE_ISS=YES ISS=GVSOC COV="$COV" ENABLE_TRACE_LOG="$TRACE" \
        CV_RESULTS="$RESULTS" ) > "$log" 2>&1
    rc=$?
    echo "comp $cfg rc=$rc wall=$(( $(date +%s) - t0 ))s" >> "$SUMMARY"
    if [ $rc -ne 0 ]; then
        echo "BUILD_FAIL $cfg (see $log)" >> "$SUMMARY"
        return 1
    fi
    return 0
}

# ---------------------------------------------------------------------------
# Phase 3 - lane pool
# ---------------------------------------------------------------------------
# Lanes the quick_val gate measured beyond the default cap. Keyed by TEST so
# the override holds in every config. corev_rand_interrupt_wfi_mem_stress ran
# 3141s on 2026-08-11 and would otherwise be a guaranteed spurious TIMEOUT.
builtin_timeout() {
    case $1 in
        corev_rand_interrupt_wfi_mem_stress) echo 6000 ;;
        *)                                   echo "" ;;
    esac
}

lane_timeout() {
    local key=$1 test=$2 v
    if [ -n "$TMO_FILE" ] && [ -r "$TMO_FILE" ]; then
        v=$(awk -v k="$key" -v t="$test" '$1==k || $1==t {print $2; exit}' "$TMO_FILE")
        [ -n "$v" ] && { echo "$v"; return; }
    fi
    v=$(builtin_timeout "$test")
    echo "${v:-$TIMEOUT_DEF}"
}

# Per-lane +UVM_TIMEOUT override (ns of sim time). Empty when no override.
lane_uvm_timeout() {
    local key=$1 test=$2
    [ -n "$UVMTMO_FILE" ] && [ -r "$UVMTMO_FILE" ] || { echo ""; return; }
    awk -v k="$key" -v t="$test" '$1==k || $1==t {print $2; exit}' "$UVMTMO_FILE"
}

is_known_fail() {
    local key=$1 test=$2
    [ -n "$XFAIL_FILE" ] && [ -r "$XFAIL_FILE" ] || return 1
    awk -v k="$key" -v t="$test" '
        /^[[:space:]]*(#|$)/ { next }
        { gsub(/^[[:space:]]+|[[:space:]]+$/, "") }
        $0 == k || $0 == t { found = 1; exit }
        END { exit(found ? 0 : 1) }
    ' "$XFAIL_FILE"
}

# Directory the makefile will run the simulation in, and therefore where
# cov.tcl saves the UCDB (uvmt.mk:120-124, cov.tcl:3).
lane_rundir() {
    local cfg=$1 test=$2 tcfg=$3 ridx=$4
    if [ -n "$tcfg" ]; then
        echo "$RESULTS/vsim_results/$cfg/$test/$tcfg/$ridx"
    else
        echo "$RESULTS/vsim_results/$cfg/$test/$ridx"
    fi
}

lane_ucdb() {
    local cfg=$1 test=$2 tcfg=$3 ridx=$4
    local d
    d=$(lane_rundir "$cfg" "$test" "$tcfg" "$ridx")
    if [ -n "$tcfg" ]; then echo "$d/${test}_${tcfg}.ucdb"; else echo "$d/${test}.ucdb"; fi
}

run_lane() {
    local cfg=$1 label=$2 test=$3 tcfg=$4 ridx=$5 cmd=$6 slot=$7
    local logf="$OUT/logs/$cfg/$label.log"
    local rundir tmo uvmtmo t0 t1 rc verdict mm ucdb
    rundir=$(lane_rundir "$cfg" "$test" "$tcfg" "$ridx")
    tmo=$(lane_timeout "$cfg/$label" "$test")
    uvmtmo=$(lane_uvm_timeout "$cfg/$label" "$test")
    # make command-line variables: the LAST assignment wins, so appending a
    # CFG_PLUSARGS replaces the one embedded in the regress-yaml lane command
    # (verified: every CFG_PLUSARGS in cv32e40p/regress/*.yaml carries only
    # +UVM_TIMEOUT). UVM itself honours the FIRST +UVM_TIMEOUT plusarg, so
    # replacement - not addition - is the only safe override.
    [ -n "$uvmtmo" ] && cmd="$cmd CFG_PLUSARGS=\"+UVM_TIMEOUT=$uvmtmo\""
    mkdir -p "$(dirname "$logf")"

    # A leftover run dir is the one way this harness can report a false pass:
    # if make dies before vsim starts, the previous log still says PASSED.
    rm -rf "$rundir"

    # Latency-aware FS-lag window: FPU_ADDMUL_LAT/FPU_OTHERS_LAT is a
    # compile-time define the DPI bridge cannot see, so the APU latency is
    # exported per config as CV_RVVI_APU_LAT (bridge: window = 10 + 5*lat,
    # opened on every APU-class FP op; 0/unset = historical behaviour).
    # Read from the AUTHORITATIVE source - the cfg yaml the TB is compiled
    # with - never string-matched on the cfg NAME: a renamed cfg, a new cfg
    # with another naming scheme, or a define changed without a rename would
    # hand the bridge the wrong window and produce phantom FS-lag FAILs on a
    # conformant DUT, with no error anywhere.
    local apulat=0 cfgyaml othlat
    cfgyaml="$CVV/cv32e40p/tests/cfg/$cfg.yaml"
    if [ -r "$cfgyaml" ]; then
        apulat=$(grep -oE 'FPU_ADDMUL_LAT=[0-9]+' "$cfgyaml" | head -1 | cut -d= -f2)
        othlat=$(grep -oE 'FPU_OTHERS_LAT=[0-9]+' "$cfgyaml" | head -1 | cut -d= -f2)
        apulat=${apulat:-0}
        if [ -n "$othlat" ] && [ "$othlat" != "$apulat" ]; then
            echo "full_verif: WARN $cfg FPU_ADDMUL_LAT=$apulat != FPU_OTHERS_LAT=$othlat - the bridge window follows ADDMUL" >&2
        fi
    else
        echo "full_verif: WARN cfg yaml unreadable ($cfgyaml) - CV_RVVI_APU_LAT=0" >&2
    fi

    t0=$(date +%s)
    # No --foreground: GNU timeout then runs the child in its own process group
    # and signals the whole group, so vsim does not survive holding a license.
    ( cd "$UVMT" && CV_RVVI_APU_LAT=$apulat timeout -k 20 "$tmo" bash -c "$cmd" ) > "$logf" 2>&1
    rc=$?
    t1=$(date +%s)

    if [ $rc -eq 0 ] && grep -q "SIMULATION PASSED" "$logf"; then
        verdict=PASS
    elif [ $rc -eq 124 ] || [ $rc -eq 137 ]; then
        verdict=TIMEOUT
    elif ! grep -q "SIMULATION PASSED\|SIMULATION FAILED" "$logf"; then
        verdict=NO_SIM
    else
        verdict=FAIL
    fi

    ucdb=$(lane_ucdb "$cfg" "$test" "$tcfg" "$ridx")
    if [ "$COV" = YES ] && [ "$verdict" = PASS ] && [ ! -s "$ucdb" ]; then
        # The lane passed but produced no coverage: treat as a lane problem, a
        # silently missing UCDB would quietly shrink the merged database.
        verdict=NO_UCDB
    fi
    if [ "$verdict" != PASS ] && is_known_fail "$cfg/$label" "$test"; then
        verdict=KNOWN_FAIL
    fi

    mm=$(grep -o 'Total Reference model mismatches *= *[0-9]*' "$logf" \
         | grep -o '[0-9]*$' | tail -1)

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$cfg" "$label" "$verdict" "$rc" "$((t1-t0))" "${mm:-na}" "$ucdb" \
        > "$OUT/status/$slot"
}

# Dispatcher. Keeps at most $JOBS lanes in flight, and never runs two lanes of
# the same (cfg, TEST, test_cfg) at once.
#
# Why that key and not (cfg, TEST): gen_corev-dv runs in a scratch directory
# named after TEST alone (vsim.mk:488), so all lanes of one TEST share a working
# directory. Everything the generator names is made unique by the RUN_INDEX
# renumbering above - the program is <TEST>_<index>.S and the log carries the
# index too. The one file left over is the throwaway UCDB cov.tcl saves on exit,
# named <TEST>[_<test_cfg>].ucdb, which the recipe deletes at the end
# (vsim.mk:517-519): lanes that differ in test_cfg write different names and are
# safe together, lanes that differ only by iteration index are not.
# Keying on the full triple instead of on TEST matters: each config has 18
# corev_directed_pulp_hwloop_debug lanes, and one of them measured 317s on its
# own, so serializing the whole test would put a chain of well over an hour on
# the critical path - past the 3141s of the longest single lane in the
# perimeter. On the triple, no group holds more than two lanes.
run_pool() {
    local n=${#L_CFG[@]}
    local -A busy_group=()
    local -A pid_of=()
    local -a taken
    local launched=0 running=0 i pick g pid

    for ((i = 0; i < n; i++)); do taken[$i]=0; done

    while [ $launched -lt $n ] || [ $running -gt 0 ]; do
        while [ $running -lt "$JOBS" ] && [ $launched -lt $n ]; do
            pick=-1
            for ((i = 0; i < n; i++)); do
                [ "${taken[$i]}" = 1 ] && continue
                g="${L_GKEY[$i]}"
                [ -n "${busy_group[$g]:-}" ] && continue
                pick=$i
                break
            done
            [ $pick -lt 0 ] && break
            taken[$pick]=1
            g="${L_GKEY[$pick]}"
            busy_group[$g]=1
            run_lane "${L_CFG[$pick]}" "${L_LABEL[$pick]}" "${L_TEST[$pick]}" \
                     "${L_TCFG[$pick]}" "${L_RIDX[$pick]}" "${L_CMD[$pick]}" "$pick" &
            pid_of[$pick]=$!
            launched=$((launched + 1))
            running=$((running + 1))
        done

        [ $running -eq 0 ] && break
        wait -n 2>/dev/null

        # wait -n does not say which child finished; reap whatever died.
        for i in "${!pid_of[@]}"; do
            pid=${pid_of[$i]}
            if ! kill -0 "$pid" 2>/dev/null; then
                wait "$pid" 2>/dev/null
                g="${L_GKEY[$i]}"
                unset "pid_of[$i]"
                unset "busy_group[$g]"
                running=$((running - 1))
                report_lane "$i"
            fi
        done
    done
}

report_lane() {
    local f="$OUT/status/$1"
    [ -s "$f" ] || { echo "lane $1 produced no status" >> "$SUMMARY"; return; }
    local cfg label verdict rc wall mm ucdb
    IFS=$'\t' read -r cfg label verdict rc wall mm ucdb < "$f"
    echo "$cfg/$label $verdict rc=$rc wall=${wall}s mismatches=$mm" >> "$SUMMARY"
}

# ---------------------------------------------------------------------------
# Phase 5 - coverage merge and reports
# ---------------------------------------------------------------------------
# `make cov MERGE=YES` is not used: its find (vsim.mk:331) sweeps every UCDB
# under the config results including failed lanes, and it is config scoped so
# it cannot produce the cross-config view. The merge is done here from the
# lane verdicts instead.
# Number of test data records in a UCDB. vcover merge collapses records that
# share a name, reports it as the suppressible error 6854 and still exits 0, so
# counting the records is the only reliable way to see that an input was
# absorbed rather than merged.
ucdb_tests() {
    vcover report -testdetails "$1" 2>/dev/null | grep -c RUNCWD
}

merge_coverage() {
    local cfg f ucdb n rc got
    local -a per_cfg=()
    local merge_rc=0

    for cfg in $CFGS; do
        mkdir -p "$OUT/cov/$cfg"
        : > "$OUT/cov/$cfg/ucdb.list"
        for f in "$OUT"/status/*; do
            [ -s "$f" ] || continue
            IFS=$'\t' read -r c l v r w m ucdb < "$f"
            [ "$c" = "$cfg" ] || continue
            [ "$v" = PASS ] || continue
            [ -s "$ucdb" ] || continue
            echo "$ucdb" >> "$OUT/cov/$cfg/ucdb.list"
        done
        n=$(wc -l < "$OUT/cov/$cfg/ucdb.list")
        echo "cov $cfg: $n passing UCDB" >> "$SUMMARY"
        [ "$n" -eq 0 ] && continue

        ( cd "$OUT/cov/$cfg" && vcover merge -testassociated -verbose -64 \
            -multiuserenv -out merged.ucdb -inputs ucdb.list ) \
            > "$OUT/cov/$cfg/merge.log" 2>&1
        rc=$?
        echo "cov $cfg merge rc=$rc" >> "$SUMMARY"
        if [ $rc -ne 0 ]; then merge_rc=1; continue; fi
        got=$(ucdb_tests "$OUT/cov/$cfg/merged.ucdb")
        if [ "$got" -lt "$n" ]; then
            echo "cov $cfg: merged $n UCDB, database holds only $got test records" >> "$SUMMARY"
            echo "cov $cfg: duplicate record names collapsed (merge.log, message 6854) - coverage totals stay complete, per-test ranking does not" >> "$SUMMARY"
            merge_rc=1
        fi
        per_cfg+=("$OUT/cov/$cfg/merged.ucdb")
        vcover report -summary "$OUT/cov/$cfg/merged.ucdb" \
            > "$OUT/cov/$cfg/summary.txt" 2>&1
    done

    local top
    if [ ${#per_cfg[@]} -eq 0 ]; then
        echo "cov: nothing to merge" >> "$SUMMARY"
        return 1
    elif [ ${#per_cfg[@]} -eq 1 ]; then
        cp -f "${per_cfg[0]}" "$OUT/cov/merged_all.ucdb"
        top="$OUT/cov/merged_all.ucdb"
    else
        # Cross-config merge. The three configs are built with different
        # +define+ sets, so this is the step most likely to be rejected or,
        # worse, to succeed while dropping an input - hence the summary
        # cross-check below.
        vcover merge -testassociated -verbose -64 -multiuserenv \
            -out "$OUT/cov/merged_all.ucdb" "${per_cfg[@]}" \
            > "$OUT/cov/merge_all.log" 2>&1
        rc=$?
        echo "cov cross-config merge rc=$rc (${#per_cfg[@]} inputs)" >> "$SUMMARY"
        if [ $rc -ne 0 ]; then
            echo "cov: cross-config merge FAILED, per-config databases are still valid" >> "$SUMMARY"
            return 1
        fi
        top="$OUT/cov/merged_all.ucdb"
    fi

    vcover report -summary "$top" > "$OUT/cov/merged_all.summary.txt" 2>&1
    local want=0 have
    for f in "${per_cfg[@]}"; do
        have=$(ucdb_tests "$f")
        want=$((want + have))
    done
    got=$(ucdb_tests "$top")
    echo "cov merge test-record check: inputs=$want merged=$got" >> "$SUMMARY"
    if [ "$got" -lt "$want" ]; then
        echo "cov: merged database lost test records, treat the cross-config view as suspect" >> "$SUMMARY"
        merge_rc=1
    fi
    # Coverage items present in only some configs (the FPU covergroups) make
    # the cross-config merge emit message 6846 per scope: the union of the
    # coverage is correct, only per-test attribution inside those scopes is
    # approximate. Counted, not treated as a failure.
    have=$(grep -c 'vcover-6846' "$OUT/cov/merge_all.log" 2>/dev/null)
    [ "${have:-0}" -gt 0 ] && \
        echo "cov: $have config-specific scopes in the cross-config merge (message 6846, expected: FPU coverage exists only in the fpu configs)" >> "$SUMMARY"

    mkdir -p "$OUT/reports"
    # vcover, not `vsim -viewcov`: same report, no simulator license held while
    # it renders.
    vcover report -html -details -precision 2 -annotate \
        -output "$OUT/reports/html" "$top" \
        > "$OUT/reports/html.log" 2>&1 \
        || { echo "cov: HTML report failed" >> "$SUMMARY"; merge_rc=1; }
    vcover report -details -output "$OUT/reports/by_module.txt" "$top" \
        > "$OUT/reports/by_module.log" 2>&1 \
        || { echo "cov: per-module report failed" >> "$SUMMARY"; merge_rc=1; }
    vcover report -cvg -details -output "$OUT/reports/by_covergroup.txt" "$top" \
        > "$OUT/reports/by_covergroup.log" 2>&1 \
        || { echo "cov: per-covergroup report failed" >> "$SUMMARY"; merge_rc=1; }

    return $merge_rc
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
# status/ is the lane -> dispatcher channel and is indexed by position in the
# lane list, so a leftover from an earlier (differently filtered) run in the
# same output dir would be counted as a result of this one.
rm -rf "$OUT/status"
mkdir -p "$OUT/manifest" "$OUT/logs" "$OUT/status" "$RESULTS" || {
    echo "full_verif: cannot create $OUT" >&2; exit 1; }

setup_env || exit 1

: > "$SUMMARY"
{
    echo "full_verif start: $(date -Iseconds)"
    echo "out:      $OUT"
    echo "configs:  $CFGS"
    echo "jobs:     $JOBS"
    echo "coverage: $COV"
    echo "seed:     $SEED_MODE"
    echo "trace:    $TRACE"
    echo "timeout:  ${TIMEOUT_DEF}s default${TMO_FILE:+, overrides from $TMO_FILE}${UVMTMO_FILE:+; uvm-timeout overrides from $UVMTMO_FILE}"
    echo "questa:   $(vsim -version 2>/dev/null | head -1)"
} >> "$SUMMARY"

declare -a L_CFG L_LABEL L_TEST L_TCFG L_RIDX L_CMD L_GKEY

for cfg in $CFGS; do
    gen_manifest "$cfg" || exit 1
    while IFS=$'\t' read -r c label test tcfg ridx cmd; do
        [ -z "$c" ] && continue
        [ "$tcfg" = "-" ] && tcfg=""
        if [ -n "$FILTER" ] && ! printf '%s/%s' "$c" "$label" | grep -Eq "$FILTER"; then
            continue
        fi
        L_CFG+=("$c"); L_LABEL+=("$label"); L_TEST+=("$test")
        L_TCFG+=("$tcfg"); L_RIDX+=("$ridx"); L_CMD+=("$cmd")
        L_GKEY+=("$c/$test/$tcfg")
    done < "$OUT/manifest/$cfg.lanes"
    echo "manifest $cfg: $(wc -l < "$OUT/manifest/$cfg.lanes") lanes${FILTER:+ (filter active)}" >> "$SUMMARY"
done

NLANES=${#L_CFG[@]}
echo "total lanes: $NLANES" >> "$SUMMARY"

if [ "$DRY" = 1 ]; then
    {
        echo "--- plan (dry run, nothing executed) ---"
        for ((i = 0; i < NLANES; i++)); do
            printf '%s/%s  run_index=%s  tmo=%ss\n' "${L_CFG[$i]}" "${L_LABEL[$i]}" \
                "${L_RIDX[$i]}" \
                "$(lane_timeout "${L_CFG[$i]}/${L_LABEL[$i]}" "${L_TEST[$i]}")"
        done
    } >> "$SUMMARY"
    cat "$SUMMARY"
    exit 0
fi

for cfg in $CFGS; do
    comp_cfg "$cfg" || { cat "$SUMMARY"; exit 1; }
done

run_pool

# ---------------------------------------------------------------------------
# Phase 4 - verdicts
# ---------------------------------------------------------------------------
# SUMMARY invariant: exactly ONE verdict line per lane, and no other line that
# a naive `grep -c ' PASS '` / `' FAIL '` can match. Two earlier violations made
# the 2026-08-17 campaign read 392 PASS / 34 FAIL+TIMEOUT against a truth of
# 391 / 17: the end-of-run recap appended every non-PASS lane record a second
# time, and this totals line spelled the verdicts as bare upper-case words, so
# it counted itself. Counts come from status/, the per-lane channel, never from
# re-reading SUMMARY.
PASS=0; FAIL=0; XFAIL=0
declare -A VCOUNT=()
for ((i = 0; i < NLANES; i++)); do
    v=$(cut -f3 "$OUT/status/$i" 2>/dev/null)
    [ -n "$v" ] || v=NO_STATUS
    VCOUNT[$v]=$(( ${VCOUNT[$v]:-0} + 1 ))
    case $v in
        PASS)       PASS=$((PASS + 1)) ;;
        KNOWN_FAIL) XFAIL=$((XFAIL + 1)) ;;
        *)          FAIL=$((FAIL + 1)) ;;
    esac
done
{
    printf '=== TOTAL (from status/, authoritative): lanes=%d' "$NLANES"
    for v in PASS KNOWN_FAIL FAIL TIMEOUT NO_SIM NO_UCDB NO_STATUS; do
        [ "${VCOUNT[$v]:-0}" -gt 0 ] || continue
        printf ' %s=%d' "$(printf '%s' "$v" | tr 'A-Z' 'a-z')" "${VCOUNT[$v]}"
        unset "VCOUNT[$v]"
    done
    for v in "${!VCOUNT[@]}"; do
        printf ' %s=%d' "$(printf '%s' "$v" | tr 'A-Z' 'a-z')" "${VCOUNT[$v]}"
    done
    printf ' (unexpected=%d) ===\n' "$FAIL"
} >> "$SUMMARY"

COV_RC=0
if [ "$COV" = YES ]; then
    merge_coverage || COV_RC=1
fi

# Unexpected failures get their OWN file, rebuilt from status/ - appending them
# back into SUMMARY is what duplicated the lane records.
UNEXPECTED="$OUT/UNEXPECTED.txt"
: > "$UNEXPECTED"
for ((i = 0; i < NLANES; i++)); do
    [ -s "$OUT/status/$i" ] || continue
    IFS=$'\t' read -r ucfg ulabel uverdict urc uwall umm uucdb < "$OUT/status/$i"
    case $uverdict in PASS|KNOWN_FAIL|'') continue ;; esac
    echo "$ucfg/$ulabel $uverdict rc=$urc wall=${uwall}s mismatches=$umm" \
        >> "$UNEXPECTED"
done
if [ -s "$UNEXPECTED" ]; then
    echo "unexpected failures: $(wc -l < "$UNEXPECTED") - list in UNEXPECTED.txt" >> "$SUMMARY"
else
    rm -f "$UNEXPECTED"
fi
if grep -q '^BUILD_FAIL ' "$SUMMARY"; then
    echo "build failures present - see the BUILD_FAIL lines above" >> "$SUMMARY"
fi
echo "full_verif end: $(date -Iseconds)" >> "$SUMMARY"

cat "$SUMMARY"
[ -s "$UNEXPECTED" ] && { echo "--- unexpected failures:"; cat "$UNEXPECTED"; }
echo "results in $OUT"
[ $FAIL -eq 0 ] && [ $COV_RC -eq 0 ]
