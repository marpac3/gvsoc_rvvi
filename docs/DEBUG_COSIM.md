# Debugging GVSOC under DPI co-simulation

How to run gdb on the bridge (`rvvi_api2gvsoc.cpp`), on the engine wrapper
(`gvsoc_engine_v2.cpp`) or on GVSOC itself while a `USE_ISS=YES ISS=GVSOC`
test is running.

Everything runs in the simulator process: Questa loads the bridge library
(`libgvsoc_rvvi_v2.so`, or the `_zfinx` variant on ZFINX CFGs), which links
the GVSOC libraries (`libpulpvp.so`, the ISS model `.so`). So the debug
story is: build with symbols, attach gdb to the running `vsim`, set
breakpoints anywhere in that stack.

## Build with symbols

Bridge + engine wrapper (fast, ABI-compatible with the installed GVSOC —
see the note in the Makefile):

```bash
cd vendor_lib/gvsoc_rvvi
make clean && make DEBUG=1        # -g -O0 on the bridge objects
                                  # (all bridge libraries)
```

GVSOC itself (only when you need to step *inside* the ISS — long rebuild):

```bash
make gvsoc DEBUG=1                # RelWithDebInfo, forwarded to the GVSOC cmake
make clean && make DEBUG=1        # rebuild the bridge against it
```

When done, restore the optimized builds: `make clean && make` (and
`make gvsoc` if GVSOC was rebuilt).

## Attach gdb: the `GVSOC_RVVI_GDB_WAIT` gate

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

## Breakpoints that pay off

| Symbol | Stops |
|--------|-------|
| `rvviRefRetireAndCompare` | once per DUT retire, before the ISS is stepped |
| `gvsoc_engine_step` | one ISS retire (commit-stream pop) |
| `gvsoc_engine_init` | engine bring-up, config parsing |
| `gvsoc_engine_get_pc` / `gvsoc_engine_get_gpr` | state readback used by the compare |
| `gvsoc_engine_materialize_commit` | peek of the next commit PC on a DUT trap row |
| `gvsoc_engine_state_current` | trap-window check that gates state compares |

For GVSOC-internal symbols, find the right name first:

```gdb
(gdb) info functions ^Iss::
(gdb) info functions exception
```

Conditional breakpoints keyed on the divergence PC are the usual workflow:

```gdb
(gdb) break gvsoc_engine_step if gvsoc_engine_get_pc() == 0x1a2c
```

## Runtime knobs (no debugger needed)

All read once in `rvviRefInit`:

| Env var | Effect |
|---------|--------|
| `CV_RVVI_BRIDGE_VERBOSE=1` | per-call bridge logging in the transcript |
| `RVVI_TEXT_TRACE=<dir>` | RVVI-TEXT `dut.rvvi`/`ref.rvvi` (see RVVI_TEXT_TRACING.md) |
| `CV_RVVI_BRIDGE_PROFILE=1` | ns counters per `rvviRef*` call, dumped at shutdown |
| `GVSOC_FORCE_TRAP_CSR=0` | disable force-resync on trap entry (gates the async-trap resync and the synchronous-trap seam realign) |
| `CV_RVVI_VOLATILE_CSR_SYNC=0` | disable the DUT-value sync of performance-counter CSR reads (cycle/instret/hpm); disabling it forks the co-sim on any program that consumes a counter value |

For behavioural (not source-level) ISS analysis, the standalone path avoids
Questa entirely: `make trace BINARY=<elf>` produces the instruction trace.

## VS Code

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

## Questa's native C Debug (`cdbg`)

Questa ships a GUI-integrated C/C++ debugger (`Tools > C Debug`, backed by
the `cdbg` Tcl command family and an attached gdb). We tried it as an
alternative to the attach gate above, driving `vsim -gui` headlessly under
Xvfb with scripted `-do` commands. Short version: on this Questa/host
combination it never actually stops at a DPI-C breakpoint — use the attach
gate. What follows is the trail, for whoever picks this up again.

Setup, once per session:

```tcl
cdbg set_debugger <path-to-gdb>   # any gdb; the one Questa ships and tests
                                  # against is
                                  # $QUESTA_HOME/linux_x86_64/external/gdb-102
cdbg debug_on
```

C breakpoints need the `-c` flag, which is easy to get wrong:

```tcl
bp -c rvviRefRetireAndCompare      # by function name
bp -c gvsoc_engine_v2.cpp 500      # by file:line
```

`bp gvsoc_engine_v2.cpp 500` (no `-c`) is parsed as an HDL breakpoint and
fails with `vsim-3325: Cannot find a reference to source file`; a gdb-style
`break gvsoc_engine_v2.cpp:500` is not a vsim command at all.

Stepping from SV into DPI-C does not work either. Auto Step Mode / Auto Find
bp — the mechanism that descends from a `.sv` call site into the C
implementation — only recognizes PLI/VPI/FLI-registered function calls, not
plain `import "DPI-C"` imports (Questa User's Manual,
`Contain_IdentifyingAllRegisteredFunctionCalls`). Stepping from the
`rvviRefRetireAndCompare(...)` call in `rvvi_trace2api.sv` will not
auto-descend into `gvsoc_engine_v2.cpp`, regardless of debug info; the
breakpoint has to be set explicitly with `bp -c`.

Console mode (`-batch`/`-c`) cannot run C Debug at all: `cdbg debug_on`
reads a Tcl variable (`vsimPriv(is_kernel_running_on_valgrind)`) that only
the interactive GUI kernel initializes, so it throws, and every later
`bp -c` fails with `C breakpoints are not supported with QIS_DEFAULT`. That
QIS_DEFAULT message is a downstream artifact of the failed `debug_on`, not
an independent statement that the QIS vopt/elaboration flow (the default
since ~2020.1, and what this testbench's `VOPT_FLAGS` use) blocks DPI-C
breakpoints: with a real GUI kernel — `vsim -gui` driven headlessly under
Xvfb — the same sequence succeeds, `debug_on` comes up and
`bp -c rvviRefRetireAndCompare` resolves to a real address, with no
vopt/QIS flag changes.

Even then, the breakpoint never fires. With the GUI kernel and a `DEBUG=1`
bridge build (without `-g` symbols `bp -c` fails outright: `Unable to set
breakpoint, location not executable ... compiled with -g`), both
`cdbg debug_on` and `bp -c gvsoc_engine_init` succeed — `gvsoc_engine_init`
is a good probe because it runs exactly once, early, before the DUT program
starts, so per-retire noise is out of the picture. `run -all` then emits a
bounded burst of `Couldn't write extended state status: Bad address.` /
`error from C debugger` pairs (84 in the run we verified, not one per
retired instruction) and the simulation runs to completion as if the
breakpoint had never been set — full UVM report, `$finish`, zero errors.
The `Bad address` message is gdb failing a `PTRACE_SETREGSET`/XSAVE write
(`EFAULT`) while installing the breakpoint's register state on the traced
process; after enough failed attempts gdb gives up silently and execution
continues. The failure reproduces identically with the Questa-bundled gdb
(`external/gdb-102`) and with a local gdb 12.1, so it is not a gdb-version
mismatch; `/proc/sys/kernel/yama/ptrace_scope` is `0` (unrestricted), which
rules out the usual ptrace-scope cause; it is independent of which function
is targeted; and wrapping the whole process tree in `script` to give it a
controlling PTY changes nothing. The one configuration we have not tried is
a human-driven GUI session on a real display. Until someone gets further,
treat Questa's native C Debug as unusable for DPI-C breakpoints here and use
the attach gate instead — it does not depend on `cdbg` and has been
validated end-to-end.
