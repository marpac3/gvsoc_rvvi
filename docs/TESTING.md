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
| Offline trace comparison (RTL vs standalone GVSOC, no DPI) | `make test TEST=hello-world ISS=GVSOC_TRACE COMP=NO` |
| RVVI-TEXT traces (RTL-only / bridge / dual) | see the "How to run" section of [`RVVI_TEXT_TRACING.md`](RVVI_TEXT_TRACING.md) |
| Conformance-check a produced trace | `make check-rvvi RVVI_TRACE_DIR=<dir>` |
| Formatter unit tests (no license needed) | `micromamba run -n gvsoc_env_3_12 make -C vendor_lib/gvsoc_rvvi test` |
| Standard validation gate (quick sweep, 4 configs, SEED=1) | `vendor_lib/gvsoc_rvvi/test/quick_val.sh [out-dir] [cfg ...]` |
| Full validation sweep (unit + one lane per coverage axis) | `vendor_lib/gvsoc_rvvi/test/validation_matrix.sh [out-dir]` |

A few things that save time:

- The reference model runs on the iss_v2 core by default (`GVSOC_ISS_V2 ?=
  YES` in `mk/Common.mk`): the run loads `libgvsoc_rvvi_v2.so` (or
  `libgvsoc_rvvi_v2_zfinx.so` on ZFINX CFGs) and the
  `gvsoc_config_v2_<CFG>.json` template. `GVSOC_ISS_V2=NO` on the make line
  falls back to the legacy v1 bridge (`libgvsoc_rvvi.so`,
  `gvsoc_config_<CFG>.json`). Both selections are made at run time — the
  `.so` is loaded at vsim startup — so toggling the switch does not require
  a TB recompile.
- `test/quick_val.sh [out-dir] [cfg ...]` is the standard validation gate
  before trusting a bridge or ISS change: one run of every test type in
  each of the four configs (`default`, `pulp`, `pulp_fpu`,
  `pulp_fpu_zfinx`), with generated tests pinned at SEED=1 so two sweeps on
  the same build are directly comparable. Verdicts are PASS / FAIL /
  TIMEOUT / NO_SIM, plus XFAIL and SKIP for known-open lanes; the exit code
  is 0 only when no lane reports FAIL/TIMEOUT/NO_SIM. `validation_matrix.sh`
  remains available as the full sweep, but the quick sweep is the gate.
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

Two things to keep in mind when a run is green. The TB's "comparison
performed" sanity counters count calls, not effective compares: GPR/FPR
comparison is currently neutralized by the write-masks the TB clears before
the compare call, so PC and the enabled CSR set are the effective divergence
detectors today (see "Known fragilities" in
[`ARCHITECTURE.md`](ARCHITECTURE.md)). And performance-counter CSR reads are
synchronized from the DUT by default (`CV_RVVI_VOLATILE_CSR_SYNC=1`), which
masks genuine counter divergences by design; disable the knob to measure
them.

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
   `GVSOC_FORCE_TRAP_CSR=0` (v1 path only — the v2 bridge handles the trap
   seam through its commit stream) or `CV_RVVI_VOLATILE_CSR_SYNC=0` to
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
| `RUNAWAY detected: N consecutive stuck-PC timeouts` | ISS stopped retiring while the DUT kept going (v1: the PC-change poll timed out; v2: no new commit within the engine's cycle budget) | dual-trace diff at the reported PC; `DEBUG_COSIM.md` |
| Sim hangs mid-run, log stopped growing | Deadlock (often around WFI/interrupt sync) | attach gdb (`DEBUG_COSIM.md`), inspect both step loops |
| Dual-trace run produces no `dut.rvvi` | TB compiled without `RVVI_TRACE=YES`, or flag missing on the run line | recompile / rerun with the flag |
| ISS edits appear to have no effect | Stale `.so` | rebuild in the submodule, then rerun |
