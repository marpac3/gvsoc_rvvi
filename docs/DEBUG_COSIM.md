# Debugging GVSOC under DPI co-simulation

How to run gdb on the bridge (`rvvi_api2gvsoc.cpp`), on the engine wrapper
(`gvsoc_engine.cpp`) or on GVSOC itself while a `USE_ISS=YES ISS=GVSOC` test
is running.

Everything runs **in the simulator process**: Questa loads
`libgvsoc_rvvi.so`, which links the GVSOC libraries (`libpulpvp.so`, the ISS
model `.so`). So the debug story is: build with symbols, attach gdb to the
running `vsim`, set breakpoints anywhere in that stack.

## 1. Build with symbols

Bridge + engine wrapper (fast, ABI-compatible with the installed GVSOC —
see the note in the Makefile):

```bash
cd vendor_lib/gvsoc_rvvi
make clean && make DEBUG=1        # -g -O0 on the bridge objects
```

GVSOC itself (only when you need to step *inside* the ISS — long rebuild):

```bash
make gvsoc DEBUG=1                # RelWithDebInfo, forwarded to the GVSOC cmake
make clean && make DEBUG=1        # rebuild the bridge against it
```

When done, restore the optimized builds: `make clean && make` (and
`make gvsoc` if GVSOC was rebuilt).

## 2. Attach gdb: the `GVSOC_RVVI_GDB_WAIT` gate

The bridge pauses at the very start of `rvviRefInit` when
`GVSOC_RVVI_GDB_WAIT=<seconds>` is set, and prints the PID to attach to:

```bash
cd cv32e40p/sim/uvmt
GVSOC_RVVI_GDB_WAIT=300 make test TEST=hello-world USE_ISS=YES ISS=GVSOC
```

In the transcript:

```
[rvvi-api2gvsoc] GDB attach gate: waiting up to 300s for   gdb -p <pid>
```

From another shell:

```bash
gdb -p <pid>
```

The gate detects the attach and raises `SIGTRAP`: gdb stops inside
`rvviRefInit`, with every `.so` already loaded. Set breakpoints and continue:

```gdb
(gdb) break gvsoc_engine_step
(gdb) break rvviRefRetireAndCompare
(gdb) continue
```

If the timeout expires with no debugger, the run continues normally — a
leftover `GVSOC_RVVI_GDB_WAIT` in a batch run costs at most the timeout.

## 3. Breakpoints that pay off

| Symbol | Stops |
|--------|-------|
| `rvviRefRetireAndCompare` | once per DUT retire, before the ISS is stepped |
| `gvsoc_engine_step` | one ISS retire (PC-change detection loop inside) |
| `gvsoc_engine_init` | engine bring-up, config parsing |
| `gvsoc_engine_get_pc` / `gvsoc_engine_get_gpr` | state readback used by the compare |

For GVSOC-internal symbols, find the right name first:

```gdb
(gdb) info functions ^Iss::
(gdb) info functions exception
```

Conditional breakpoints keyed on the divergence PC are the usual workflow:

```gdb
(gdb) break gvsoc_engine_step if gvsoc_engine_get_pc() == 0x1a2c
```

## 4. Runtime knobs (no debugger needed)

All read once in `rvviRefInit`:

| Env var | Effect |
|---------|--------|
| `CV_RVVI_BRIDGE_VERBOSE=1` | per-call bridge logging in the transcript |
| `GVSOC_DUT_TRACE=1` | per-retire DUT PC trace |
| `RVVI_TEXT_TRACE=<dir>` | RVVI-TEXT `dut.rvvi`/`ref.rvvi` (see RVVI_TEXT_TRACING.md) |
| `CV_RVVI_BRIDGE_PROFILE=1` | ns counters per `rvviRef*` call, dumped at shutdown |
| `GVSOC_FORCE_TRAP_CSR=0` | disable force-resync on IRQ traps |

For behavioural (not source-level) ISS analysis, the standalone path avoids
Questa entirely: `make trace BINARY=<elf>` produces the instruction trace.

## 5. VS Code

`Attach to Process` on `vsim`, with the repo as workspace:

```json
{
  "name": "Attach to co-sim (vsim)",
  "type": "cppdbg",
  "request": "attach",
  "program": "${env:QUESTA_HOME}/linux_x86_64/vsimk",
  "processId": "${command:pickProcess}",
  "MIMode": "gdb"
}
```

Launch the test with `GVSOC_RVVI_GDB_WAIT=300`, pick the `vsimk` process,
then set breakpoints from the editor and continue.

## 6. Questa's native C Debug (`cdbg`) — known limitations

Questa ships a GUI-integrated C/C++ debugger (`Tools > C Debug` in the
Visualizer/vsim GUI, backed by the `cdbg` Tcl command family and an
attached gdb). It was evaluated as an alternative to the attach gate above;
here is what actually works and what does not, verified by driving `vsim
-gui` headlessly under Xvfb with scripted `-do` commands (not just read from
the manual).

**Setup, once per session:**

```tcl
cdbg set_debugger <path-to-gdb>   # any gdb works; the one Questa ships and
                                   # tests against is
                                   # $QUESTA_HOME/linux_x86_64/external/gdb-102
cdbg debug_on
```

**Breakpoint syntax.** In the vsim Transcript / `-do` script, C breakpoints
require the `-c` flag — this is easy to get wrong:

```tcl
bp -c rvviRefRetireAndCompare      # by function name
bp -c gvsoc_engine.cpp 500         # by file:line
```

`bp gvsoc_engine.cpp 500` (no `-c`) is parsed as an **HDL** breakpoint and
fails with `vsim-3325: Cannot find a reference to source file`. A bare
gdb-style `break gvsoc_engine.cpp:500` is not a vsim Transcript command at
all.

**Step-into from SV does not reach DPI-C.** Auto Step Mode / Auto Find bp
(the mechanism that lets you step from a `.sv` call site straight into the
C implementation) only recognizes **PLI/VPI/FLI-registered** function
calls, not plain `import "DPI-C"` imports — confirmed in the Questa User's
Manual (`Contain_IdentifyingAllRegisteredFunctionCalls`). Stepping from
`rvvi_trace2api.sv:196` (the `rvviRefRetireAndCompare(...)` call) into
`gvsoc_engine.cpp` will not auto-descend, regardless of debug info. Set the
breakpoint explicitly with `bp -c` instead.

**`-batch`/`-c` (console) mode cannot run C Debug at all.** `cdbg debug_on`
depends on a GUI-session-only Tcl variable
(`vsimPriv(is_kernel_running_on_valgrind)`) that is only populated by the
interactive GUI kernel init path. In `-batch`/`-c` mode it throws `can't
read "vsimPriv(is_kernel_running_on_valgrind)"`, and any subsequent `bp -c`
fails with `C breakpoints are not supported with QIS_DEFAULT` — that
QIS_DEFAULT message is a **downstream artifact of the failed `debug_on`**,
not an independent statement that the modern QIS vopt/elaboration flow
(the default since ~2020.1, and what this testbench's `VOPT_FLAGS` use)
blocks DPI-C breakpoints. Confirmed by re-running the identical sequence
with a real GUI kernel (`vsim -gui`, driven headlessly under Xvfb): there,
`debug_on` succeeds and `bp -c rvviRefRetireAndCompare` is accepted and
resolves to a real address — no vopt/QIS flag change needed.

**Open issue: the breakpoint is accepted but never actually stops the
sim.** With a real GUI kernel, both `cdbg debug_on` and `bp -c
rvviRefRetireAndCompare` succeed, but `run -all` then produces a stream of
generic `error from C debugger` messages (hundreds, roughly one per
retired instruction) and the simulation runs straight through to
`$finish` without ever halting at the breakpoint. This reproduces
identically with the Questa-bundled gdb (`external/gdb-102`, the version
Questa is tested against) and with a newer local gdb 12.1 — so it is not a
gdb-version mismatch. `/proc/sys/kernel/yama/ptrace_scope` is `0`
(unrestricted) on this host, ruling out the usual ptrace-scope cause. The
headless test harness used to reproduce this has **no controlling TTY**
(`tty` → "not a tty"); cdbg's gdb integration may depend on a real
interactive terminal that a scripted/headless session doesn't provide —
this is the one variable not yet ruled out. **Not yet confirmed whether
this also happens in a genuine interactive terminal session** — if you hit
the same `error from C debugger` spam there, C Debug is not usable for
DPI-C breakpoints on this Questa/host combination and the attach gate
(§2) is the reliable path; if it does stop cleanly there, this note should
be updated.
