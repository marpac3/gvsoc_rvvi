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
