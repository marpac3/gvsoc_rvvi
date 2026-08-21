# Running and debugging the co-simulation tests

How to set up a shell, run the CV32E40P/GVSOC co-simulation in its different
modes, read the results, and track a failure down to its cause. The one-time
build (submodules, GVSOC, bridge libraries) is covered in the top-level
[`README.md`](../README.md); the trace modes are covered in
[`RVVI_TEXT_TRACING.md`](RVVI_TEXT_TRACING.md) and source-level debugging in
[`DEBUG_COSIM.md`](DEBUG_COSIM.md).

## Shell setup

Every shell that runs simulations needs the simulator license and the
cross-toolchain variables. Interactive shells usually get these from the
user profile; non-interactive shells (scripts, CI, editors' terminals) do
not read `~/.bashrc`, so set them explicitly:

```bash
source /etc/profile.d/modules.sh
module load questa/2025.3
export CV_SIMULATOR=vsim
export CV_SW_TOOLCHAIN=/opt/riscv/corev-openhw-gcc-modded-v0.1
export CV_SW_PREFIX=riscv64-unknown-elf-
```

Rebuilds of GVSOC or the bridge also need the Python environment
(`micromamba activate gvsoc_env_3_12`, or the one-shot
`micromamba run -n gvsoc_env_3_12 ...` form) — see the README.

When every test dies within seconds, check the license first. `lmutil`
ships with Questa but is not on `PATH`; reach it relative to `vsim`:

```bash
$(dirname $(which vsim))/../linux_x86_64/lmutil lmstat -c $LM_LICENSE_FILE \
    | grep -A2 'Vendor daemon'
```

The FlexLM manager (`lmgrd`) can be up while the Siemens vendor daemon
(`saltd`) is down — the telltale is `saltd: No socket connection to license
server manager (-7,10015)`. In that state `vsim` exits almost immediately
(make rc=2, a few seconds of wall-clock) and nothing on the client side can
fix it; the daemon has to be restarted on the server.

## Running tests

All commands from `cv32e40p/sim/uvmt/` in the core-v-verif tree.

| Goal | Command |
|------|---------|
| Compile the TB for co-simulation | `make comp USE_ISS=YES ISS=GVSOC [CFG=...]` |
| Run one test in step-and-compare co-simulation | `make test TEST=hello-world USE_ISS=YES ISS=GVSOC [CFG=...]` |
| Same, skipping recompilation | append `COMP=NO` |
| RVVI-TEXT traces (RTL-only / bridge / dual) | see the "How to run" section of [`RVVI_TEXT_TRACING.md`](RVVI_TEXT_TRACING.md) |
| Conformance-check a produced trace | `make check-rvvi RVVI_TRACE_DIR=<dir>` |
| Formatter unit tests (no license needed) | `micromamba run -n gvsoc_env_3_12 make -C vendor_lib/gvsoc_rvvi test` |
| Standard validation gate (quick sweep, 4 configs, SEED=1) | `vendor_lib/gvsoc_rvvi/test/quick_val.sh [out-dir] [cfg ...]` |
| Full regression with code coverage (374 lanes, 3 configs) | `vendor_lib/gvsoc_rvvi/test/full_verif.sh [out-dir]` — see below |

A few things that save time:

- The reference model runs on the iss_v2 core: the run loads
  `libgvsoc_rvvi_v2.so` (or `libgvsoc_rvvi_v2_zfinx.so` on ZFINX CFGs) and
  the `gvsoc_config_v2_<CFG>.json` template. There is no core selector —
  the testbench Makefile fails with an explicit error if `GVSOC_ISS_V2=NO`
  is passed on the make line. The `.so` is loaded at vsim startup, not
  linked into the TB objects.
- `test/quick_val.sh [out-dir] [cfg ...]` is the standard validation gate
  before trusting a bridge or ISS change: one run of every test type in
  each of the four configs (`default`, `pulp`, `pulp_fpu`,
  `pulp_fpu_zfinx`), with generated tests pinned at SEED=1 so two sweeps on
  the same build are directly comparable. Verdicts are PASS / FAIL /
  TIMEOUT / NO_SIM, plus XFAIL and SKIP for known-open lanes; the exit code
  is 0 only when no lane reports FAIL/TIMEOUT/NO_SIM. It is the single
  validation flow: the earlier matrix/regression-list sweeps are retired.
- `CFG` selects the core configuration (`cv32e40p/tests/cfg/*.yaml`: `pulp`,
  `pulp_fpu`, `pulp_fpu_zfinx`, latency variants, ...). Changing `CFG`,
  `USE_ISS` or `RVVI_TRACE` changes compile-time defines, so the TB must be
  recompiled — drop `COMP=NO`.
- A TB compiled with `RVVI_TRACE=YES` also needs `RVVI_TRACE=YES` on the run
  line: the flag gates plusargs at runtime, not only defines at compile time.
- After editing ISS or bridge sources, rebuild the libraries in the submodule
  (`micromamba run -n gvsoc_env_3_12 make gvsoc && make` from
  `vendor_lib/gvsoc_rvvi/`) before rerunning. The `.so` files are loaded at
  vsim startup; a stale build silently runs the old code.
- Run artifacts land in `vsim_results/<CFG>/<TEST>/0/vsim-<TEST>.log`
  (the `0` is `RUN_INDEX`, not the seed; it only changes for multi-run
  invocations).

## Full regression with coverage

`quick_val.sh` is the fast gate — one run per test type. The full v2
regression perimeter is 374 lanes over three configs, and running it
sequentially the way `bin/cv_regress` emits it takes about ten core-hours.
`test/full_verif.sh` runs the same perimeter as a parallel pool and collects
Questa code coverage on the way:

```bash
vendor_lib/gvsoc_rvvi/test/full_verif.sh [output-dir]
```

It needs no shell setup — it loads Questa 2025.3 and the CoreV toolchain
itself, and refuses to start on any other simulator version (UCDBs from
different releases cannot be merged). The only prerequisite is that the GVSOC
bridge libraries are already built. The default output directory is
`/data2/$USER/fullverif_<timestamp>`; always keep it on `/data2`, a campaign
writes tens of gigabytes and the run tree must not land in the repo or on NFS.

| Knob | Default | Effect |
|------|---------|--------|
| `FV_JOBS` | 16 | Parallel lanes. 24 is the sane ceiling on the shared box; beyond ~16 the wall clock is set by the slowest single lane, not by the pool. |
| `FV_CFGS` | `pulp pulp_fpu pulp_fpu_zfinx` | Configs to run. |
| `FV_COV` | `YES` | Coverage build and collection. `NO` gives a reference run for measuring overhead — use a separate output directory, the work library differs. |
| `FV_SEED_MODE` | `random` | `random` is what `cv_regress` emits; a fixed number makes two campaigns directly comparable. |
| `FV_TIMEOUT` | 2400 | Per-lane wall-clock cap, seconds. Overridable per lane with `FV_TMO_FILE`. |
| `FV_XFAIL_FILE` | *(empty)* | Known-open lanes, `cfg/label` or bare `TEST` per line. Reported `KNOWN_FAIL`, excluded from the merge, ignored by the exit code. |
| `FV_FILTER` | *(empty)* | Extended regex on `cfg/label`; run a subset without changing anyone's run index. |
| `FV_DRY` | 0 | Generate manifests, print the plan, run nothing. |

The script first asks `cv_regress` for the lane list of each config, parses it
into `manifest/<cfg>.lanes`, and renumbers `RUN_INDEX`/`GEN_START_INDEX` per
`(config, test)` — two regression-list entries can otherwise ask for the same
run directory and silently overwrite each other's results and UCDB. It then
compiles each config once, serially (every lane runs with `COMP=0`, so nothing
recompiles into the shared work library while the pool is live), and only then
opens the pool.

Output layout under the run directory:

```text
manifest/<cfg>.lanes    parsed lane table, one line per lane
logs/<cfg>/<label>.log  per-lane make output
results/                CV_RESULTS tree (vsim_results/<cfg>/<TEST>/<idx>/)
cov/<cfg>/merged.ucdb   per-config merge of the passing lanes
cov/merged_all.ucdb     cross-config merge
reports/html/           browsable coverage report (open index.html)
reports/by_module.txt   per-module code coverage
reports/by_covergroup.txt   functional coverage, covergroup by covergroup
SUMMARY.txt             verdict and wall clock per lane
```

Verdicts are the same vocabulary as the quick gate — `PASS`, `FAIL`,
`TIMEOUT`, `NO_SIM`, `KNOWN_FAIL` — plus `NO_UCDB` for a lane that passed but
produced no coverage database, which means the run did not go through
`cov.tcl` and its coverage is silently missing from the merge. Only `PASS`
lanes feed the merge. The exit code is 0 only when every lane was `PASS` or
`KNOWN_FAIL`, so the first campaign on a new baseline is expected to exit
non-zero: read `SUMMARY.txt`, triage, and put the genuinely-open lanes in an
`FV_XFAIL_FILE` for the next run.

Reading the coverage output: `reports/by_module.txt` is the code-coverage view
(statement / branch / condition / expression / FSM, toggle deliberately off —
the build passes `+cover=bcsef` on the DUT only, so testbench and interface
code do not dilute the numbers). `reports/by_covergroup.txt` is the functional
view, and it is the one to look at first — a config-level hole shows up there
as a covergroup at low percentage, while code coverage tends to saturate
early. The HTML report under `reports/html/` cross-links the two and is the
practical way to walk from an uncovered bin to the source line.

Two caveats when comparing campaigns. Coverage is only merged from lanes that
passed, so a campaign with failures reports coverage of a smaller test set —
compare the lane counts before comparing percentages. And with
`FV_SEED_MODE=random` the generated tests differ between campaigns; pin a seed
if the point is to measure the effect of a change rather than to hunt bugs.

## Reading the results

The `SIMULATION PASSED` / `SIMULATION FAILED` banner is computed by the TB at
final phase: the test program must have finished (`sim_finished`), with zero
errors and zero UVM fatals. With the reference model enabled, the TB
additionally fails the run if no retire ever reached the reference model, if
no PC / GPR / instruction comparison was performed, or if the mismatch count
is non-zero — reported as `ERROR: Total Reference model mismatches = N`.

The bridge prints its own summary at shutdown:

```text
[rvvi-api2gvsoc] --- performance summary ---
[rvvi-api2gvsoc]   retires total  : 4937
[rvvi-api2gvsoc]   wall-clock     : 12.3 s
[rvvi-api2gvsoc]   retires/sec    : 401
[rvvi-api2gvsoc]   IRQ resyncs    : 2        # only if non-zero
[rvvi-api2gvsoc]   phase realigns : 2        # only if non-zero
[rvvi-api2gvsoc]   volatile syncs : 31       # only if non-zero
```

Every individual divergence is logged when it happens, throttled at 10 per
category:

```text
[rvvi-api2gvsoc] ERROR: PC mismatch #1 @ retire #1234: DUT=0x00001a2c ISS=0x00001a30
[rvvi-api2gvsoc] ERROR: CSR[0x342] mismatch @ retire #1234: DUT=0x0000000b ISS=0x00000002 (PC=0x00001a2c)
```

Categories are `PC`, `GPR`, `GPR-written`, `FPR`, `INSN` and `CSR[addr]`.
The first mismatch line is the one that matters — everything after the first
divergence is usually noise from the two machines drifting apart.

Two things to keep in mind when a run is green. GPR/FPR comparison runs on
the *written* set only (the RVFI write-back masks): a register the DUT never
rewrites after a divergence is not re-checked until something writes it
again. The effective divergence detectors are PC, the written GPR/FPR set,
the instruction binary (the ISS-side encoding travels with the commit ring;
rows served without an ISS execution — repairs, pins — skip the compare and
are not counted) and the enabled CSR set, which since the compare-hardening
pass includes fflags/frm/fcsr in full, dscratch0/dscratch1 and the modeled
retired-instruction counters (minstret/minstreth and the instreth alias).
And performance-counter reads of the CSRs still declared volatile by the TB
(cycle/mcycle and the hpm bank) are synchronized from the DUT by default
(`CV_RVVI_VOLATILE_CSR_SYNC=1`), which masks divergences on those counters
by design; disable the knob to measure them. The modeled counters above are
NOT synchronized: a read of minstret lands in the honest compare. The
`mstatus.FS` compare around the FPU is latency-aware: `full_verif.sh`
exports `CV_RVVI_APU_LAT` per config (parsed from the cfg yaml's
`FPU_ADDMUL_LAT`, never from the cfg name) and the bridge holds a short
compare window after APU-class ops — at zero latency only the iterative
fdiv/fsqrt open it, so FP loads sit outside the window (see the FS entry
under Known fragilities in `ARCHITECTURE.md`).

## From a red test to a cause

1. Summary first:
   `grep -E 'Total Reference model mismatches|phase realigns|retires total|UVM_ERROR :' vsim-<TEST>.log`,
   and check the wall-clock. A run that died in under 30 s never got past
   elaboration (license, missing `.so`, config error) — read the top of the
   log, not the bottom.
2. Find the first divergence:
   `grep -m1 'rvvi-api2gvsoc] ERROR' vsim-<TEST>.log` — note the retire
   number, PC and category.
3. Rerun in dual-trace mode (same seed) and diff the two traces:

   ```bash
   make test TEST=<t> USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES RVVI_TEXT_TRACE=/tmp/tr
   diff <(awk '{print $1,$2,$3}' /tmp/tr/dut.rvvi) \
        <(awk '{print $1,$2,$3}' /tmp/tr/ref.rvvi) | head
   ```

   The first differing line localizes a control-flow divergence exactly (the
   projection keeps token, PC and opcode only). For value-only mismatches —
   same PC, different register/CSR value — diff the full lines instead:
   `diff /tmp/tr/dut.rvvi /tmp/tr/ref.rvvi | head`.
4. Classify. Walk the few instructions before the divergence in both traces
   and decide which side is wrong against the ISA manual / core user manual.
   A DUT-side anomaly points at the RTL or at a test expectation; a ref-side
   anomaly points at the ISS model; a divergence that immediately self-heals
   (paired with `phase realigns` in the summary) points at retire alignment
   in the bridge, not at either model.
5. Reproduce ISS-side without Questa: `make trace BINARY=<elf>` in the
   submodule runs the same platform standalone and prints the instruction
   trace. Fastest way to confirm an ISS-model hypothesis, and it needs no
   license.
6. Turn up bridge visibility on a rerun when the mechanics of the sync are in
   question: `CV_RVVI_BRIDGE_VERBOSE=1` for per-call logging;
   `GVSOC_FORCE_TRAP_CSR=0` or `CV_RVVI_VOLATILE_CSR_SYNC=0` to
   isolate the effect of the sync mechanisms (expect forks — that is the
   point).
7. Go source-level when a hypothesis needs stepping through the ISS: gdb
   attaches to the live co-simulation through the `GVSOC_RVVI_GDB_WAIT` gate
   — procedure and useful breakpoints in
   [`DEBUG_COSIM.md`](DEBUG_COSIM.md).

## Common failures

| Symptom | Likely cause | Check / fix |
|---------|--------------|-------------|
| Every test rc=2 within seconds | License vendor daemon down | license check above; wait for server-side restart |
| `CV_SW_TOOLCHAIN not defined` | Non-interactive shell, env not exported | shell setup above |
| `SIMULATION FAILED`, mismatches > 0 | Real divergence | triage above |
| PASSED but `phase realigns` non-zero | Bridge recovered a retire misalignment — benign if mismatches = 0, still worth a look in the traces | dual-trace diff |
| `RUNAWAY detected: N consecutive stuck-PC timeouts` | ISS stopped retiring while the DUT kept going (no new commit within the engine's cycle budget) | dual-trace diff at the reported PC; `DEBUG_COSIM.md` |
| Sim hangs mid-run, log stopped growing | Deadlock (often around WFI/interrupt sync) | attach gdb (`DEBUG_COSIM.md`), inspect both step loops |
| Dual-trace run produces no `dut.rvvi` | TB compiled without `RVVI_TRACE=YES`, or flag missing on the run line | recompile / rerun with the flag |
| ISS edits appear to have no effect | Stale `.so` | rebuild in the submodule, then rerun |
