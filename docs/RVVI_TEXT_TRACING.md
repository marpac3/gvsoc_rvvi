# RVVI-TEXT Tracing

CV32E40P instruction tracing in the RVVI-TEXT format. One canonical C++
formatter (`rvvi_text_writer.{hpp,cpp}`) serves three run modes:

- RTL-only (`RVVI_TRACE=YES USE_ISS=NO`) — a standalone SV tracer is the
  only producer; it emits `dut.rvvi` through `librvvi_text.so`, no ISS in
  the loop.
- bridge-emit (`USE_ISS=YES ISS=GVSOC`, no `RVVI_TRACE`, env
  `RVVI_TEXT_TRACE` set) — the DPI co-simulation bridge emits both
  `dut.rvvi` and `ref.rvvi`.
- dual-trace (`USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES`) — co-simulation plus
  the SV tracer: the tracer is the sole `dut.rvvi` producer, the bridge is
  switched to ref-only and emits only `ref.rvvi`.

In every mode the step-and-compare path of the co-simulation is untouched;
trace emission is additive.

## The RVVI-TEXT format

RVVI-TEXT is the open Imperas text format for retired-instruction traces
(spec vendored at `RVVI/RVVI-TEXT/README.md`, v0.4, from
riscv-verification/RVVI). One line per retirement records the PC, the
opcode, and the architectural write-set of that instruction. A trace is
self-describing and machine-checkable — the conformance gate is the
vendored `RVVI/source/host/rvvi/rvviTextChecker.py` (exit 0 = valid).

Three header lines, then one line per event:

```
VERSION 0 1
VENDOR "gvsoc_rvvi" 0 1
PARAMS 6 ILEN 32 XLEN 32 FLEN 32 VLEN 0 NHART 1 RETIRE 1
RET 0x80 0x197 X 3 0x14080 C 0x300 0x1800 MODE 0x3
TRAP 0x88 0xffffffff C 0x300 0x1800 C 0x341 0x88 C 0x342 0x2 C 0x343 0xffffffff MODE 0x3
```

| Token | Meaning |
|-------|---------|
| `RET 0x<pc> 0x<insn>` | a normal retirement at `pc`, opcode `insn` |
| `TRAP 0x<pc> 0x<insn>` | a synchronous-exception event at the faulting `pc` |
| ` X <d> 0x<hex>` | GPR `x<d>` written this retirement |
| ` F <d> 0x<hex>` | FPR `f<d>` written this retirement |
| ` C 0x<addr> 0x<hex>` | CSR at `addr` written this retirement |
| ` MODE 0x<m>` | privilege mode (optional trailing column) |

`VERSION 0 1` matches the golden Imperas example
(`RVVI/RVVI-TEXT/examples/traps.rvvi`) and is accepted by the checker. The
`VENDOR` string must start with a letter and contain no hyphen (checker
STRING rule). `FLEN` in `PARAMS` is derived from the CFG on both producers,
so the two headers of a dual-trace run agree.

Trap semantics come from RVVI-TRACE (the interface): `TRAP` means a
synchronous exception, i.e. the faulting instruction does not retire.
Concretely:

- A sync exception (illegal instruction, misaligned access, access fault,
  `ECALL`/`EBREAK`) produces a `TRAP` line at the faulting PC carrying the
  four trap-entry CSRs (`mstatus`, `mepc`, `mcause`, `mtval`) and no
  `X`/`F` tokens — a faulting instruction retires no register writes.
- An async interrupt produces no `TRAP` line: RVVI-TEXT has no `intr`
  element, so the handler entry is a normal `RET` (with the trap-entry CSR
  writes attached), as the standard prescribes.
- At the trap seam the `ref.rvvi` line reuses DUT data (values and
  privilege mode): the ISS has not consumed the trap yet at that boundary,
  so its own post-trap state is not observable there.

The checker validates `TRAP` lines syntactically only (its `check_TRAP` is
identical to `check_RET`); the semantics above follow the RVVI-TRACE
definitions and the Imperas example, not a checker constraint.

CSR dedup is a formatter invariant: one `C` token per address per line,
emitted at the address's first position with the last value pushed
(`rvvi_text_write_line`). Producers legitimately push the same address
twice within one retirement — on a trap, both the generic `csr_wb` sparse
scan and the explicit trap-CSR re-push fire for the same addresses — and
the formatter absorbs it.

## Architecture

One canonical formatter — `rvvi_text_writer.{hpp,cpp}`, pure C++17 + stdio,
zero GVSOC dependency — owns the byte format. Callers hand it plain data:

```cpp
struct RvviTextParams   { const char *vendor; uint32_t ilen,xlen,flen,vlen,nhart,retire; };
struct RvviTextCsr      { uint32_t addr, value; };
struct RvviTextWriteSet {
    uint32_t pc, insn;  bool is_trap;
    bool has_mode; uint32_t mode;       // MODE column, per-producer opt-in
    uint32_t gpr_mask, gpr[32];
    uint32_t fpr_mask, fpr[32];
    std::vector<RvviTextCsr> csr;
};
void rvvi_text_write_header(FILE*, const RvviTextParams&);
void rvvi_text_write_line  (FILE*, const RvviTextWriteSet&);
```

It is linked into three shared objects (`vendor_lib/gvsoc_rvvi/Makefile`):
`libgvsoc_rvvi_v2.so` and `libgvsoc_rvvi_v2_zfinx.so` (the DPI
co-simulation bridge, `rvvi_api2gvsoc.cpp`, calls it through a thin adapter
fed from the bridge's DUT-push state and the ISS getters) and
`librvvi_text.so` (a standalone DPI shim, `rvvi_text_dpi.{hpp,cpp}` — the
`rvviText*` functions, zero GVSOC dependency — used by the SV tracer).

### The two producers

The SV tracer (`cv32e40p/tb/uvmt/uvmt_cv32e40p_rvvi_text_tracer.sv`, in the
superproject) reads the `rvviTrace` interface (`rvvi_if`), so the privilege
MODE comes from the existing wiring and no CSR logic is re-implemented in
SV. In RTL-only mode it also drives the interface itself, via the shared
RVFI→RVVI wiring in `uvmt_cv32e40p_iss_wrap_common.svh`; in dual-trace mode
that driving logic compiles out under `` `ifndef USE_ISS ``, making the
tracer a read-only observer of the `rvvi_if` that
`uvmt_cv32e40p_gvsoc_wrap.sv` drives — never a second driver. (A
preprocessor guard, not a generate-if: the shared include contains a
top-level `generate` block, and SystemVerilog forbids nesting it in another
generate scope.)

The DPI bridge (`rvvi_api2gvsoc.cpp`) emits at the end of the batched
retire-and-compare, and in `rvviDutTrap` for trap lines. The write-set —
*which* GPR/FPR/CSR changed — always comes from the DUT push; the `dut`
line takes DUT values and the `ref` line takes ISS values for the same
indices, which is what makes the two files line up 1:1 for a plain `diff`.

Two `ref.rvvi` columns are DUT-sourced by design at every retirement, not
only at the trap seam: the instruction word (the write-set model takes it
from the DUT push) and the `MODE` column (the DUT-reported privilege mode;
the ISS exposes no mode getter). On a clean run they coincide with the ISS
state anyway; at a divergence, read the ref line as ISS PC and register
values next to the DUT opcode and mode.

### Mode matrix

Two independent dimensions select the behavior: compile-time defines
(`USE_ISS`/`USE_GVSOC`, `RVVI_TRACE`) and the runtime environment variable
`RVVI_TEXT_TRACE=<dir>|1`, which enables emission on the bridge side
(unset/empty/`0` — bridge emission off, zero hot-path overhead). Mode
selection is compile-time only; there is no runtime plusarg to toggle the
tracer.

| Mode | Compile-time | Runtime | `dut.rvvi` | `ref.rvvi` | step-and-compare |
|------|-------------|---------|------------|------------|------------------|
| no-trace | any, without `RVVI_TRACE` | `RVVI_TEXT_TRACE` unset | — | — | yes if co-sim |
| RTL-only | `RVVI_TRACE`, no `USE_ISS` | `+rvvi_text_dut=` optional | SV tracer | — | — |
| bridge-emit | `USE_ISS`+`USE_GVSOC`, no `RVVI_TRACE` | `RVVI_TEXT_TRACE=<dir>\|1` | bridge | bridge | yes |
| dual-trace | `USE_ISS`+`USE_GVSOC`+`RVVI_TRACE` | `RVVI_TEXT_TRACE` optional | SV tracer | bridge (ref-only) | yes |
| Imperas co-sim | `USE_ISS`, no `USE_GVSOC` | — | — | — | yes (ImperasDV); `RVVI_TRACE=YES` ignored with a Make info message |

The tracer always emits the MODE column. The bridge emits it only in
dual-trace mode and only on `ref.rvvi`; `has_mode` is keyed on which file
is being written, never on the dut/ref data-source flag — trap lines on
`ref.rvvi` deliberately reuse DUT data, so keying on the data source would
silently drop MODE from every ref-side `TRAP` line. In bridge-emit mode
neither file carries MODE, which keeps that output byte-compatible with the
original inline emitter. The per-retire mode value reaches the bridge
through `rvviBridgeSetMode`, called by `rvvi_trace2api.sv`.

For FLEN, the tracer computes `FLEN = (FPU != 0) ? 32 : 0` from its `FPU`
parameter and the bridge receives the same value through
`rvviBridgeSetFlen`, called unconditionally from `gvsoc_wrap`'s `ref_init`
before `rvviRefInit`. In dual-trace mode `ref_init` also calls
`rvviBridgeSetRefOnly` (under `` `ifdef RVVI_TRACE ``) before `rvviRefInit`,
where the file-open decision happens: the bridge then opens only `ref.rvvi`
and the tracer is the sole `dut.rvvi` producer.

### Build wiring

- RTL-only: `mk/uvmt/vsim.mk` adds `+define+RVVI_TRACE` and compiles
  `cv32e40p/tb/uvmt/rvvi_trace.flist` — `rvviTrace.sv` plus the tracer.
  `rvviApiPkg.sv` is deliberately not compiled: nothing in the RTL-only
  build references the RVVI API. The DPI library is `librvvi_text.so`
  (`RVVI_TEXT_MODEL` in `mk/Common.mk`).
- Dual-trace: `vsim.mk` adds `+define+RVVI_TRACE` and compiles
  `cv32e40p/tb/uvmt/rvvi_trace_dual.flist` on top of `gvsoc.flist` — the
  tracer only, since the interface and API package are already in
  `gvsoc.flist`.
- `uvmt_cv32e40p_tb.sv` instantiates `rvvi_if` under
  `` `ifdef USE_ISS `` / `` `elsif RVVI_TRACE ``; the standalone tracer
  under `` `elsif RVVI_TRACE ``; and, in dual-trace, the tracer alongside
  `gvsoc_wrap` on the same `rvvi_if` instance.

### Robustness details

- `rvviTextOpen` returns a status; the tracer raises `$error` on failure —
  a passing run that silently wrote no trace is easy to miss in a batch.
- Both the bridge and the shim flush their file(s) every 1000 retirements,
  so a killed run keeps its trace tail.
- On a trap the bridge takes an emit-only snapshot of the GPR/FPR write
  masks before they are cleared for the next retirement; the compare masks
  are untouched, so the step-and-compare path is unchanged by construction.
- The tracer drives `mtval` itself in RTL-only mode
  (`RVVI_SET_TRAP_CSR` on `CSR_MTVAL_ADDR`), because the shared include
  gates that wiring behind `` `ifdef USE_GVSOC ``.

## How to run

All commands from `cv32e40p/sim/uvmt/`. Load the simulator license first:

```bash
module load questa/2025.3
```

RTL-only mode — RTL simulation, no ISS; the SV tracer writes `dut.rvvi`:

```bash
make test TEST=hello-world RVVI_TRACE=YES USE_ISS=NO
```

`dut.rvvi` lands in the run directory. Redirect with `RVVI_TEXT_TRACE=<dir>`
on the make line — `vsim.mk` creates the directory if missing and wires it
to the tracer's `+rvvi_text_dut=<path>` plusarg.

Bridge-emit mode — DPI co-simulation; the bridge writes both files:

```bash
make test TEST=hello-world USE_ISS=YES ISS=GVSOC RVVI_TEXT_TRACE=<dir>
```

`RVVI_TEXT_TRACE=<dir>` puts `dut.rvvi` + `ref.rvvi` in `<dir>`;
`RVVI_TEXT_TRACE=1` puts both files in the run cwd.

Dual-trace mode — DPI co-simulation plus the tracer; the tracer writes
`dut.rvvi`, the bridge writes `ref.rvvi`:

```bash
make test TEST=hello-world USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES RVVI_TEXT_TRACE=<dir>
```

Conformance gate on whatever a run produced:

```bash
make check-rvvi RVVI_TRACE_DIR=<dir>
```

(`mk/uvmt/uvmt.mk`: runs the vendored checker on whichever of
`dut.rvvi`/`ref.rvvi` exists in `<dir>`; fails if neither exists or the
checker rejects one.)

To confirm the bridge emitter engaged, grep the run log
(`vsim_results/<CFG>/<TEST>/0/vsim-<TEST>.log`) for `RVVI-TEXT ->`, printed
by `rvviRefInit`; it shows `ref-only=1` in dual-trace mode. With `ISS` set
to anything but GVSOC, `RVVI_TRACE=YES` is ignored and make prints
`Info: RVVI_TRACE=YES ignored - dual-trace requires ISS=GVSOC`.

## Limitations and open points

- No GVSOC-only headless runner (trace the ISS with no RTL and no Questa in
  the loop). It would need per-retire write-set capture inside the engine
  wrapper — the bridge derives the write-set from the DUT push, which does
  not exist without a DUT — and would have to bypass `rvviRefEventStep`,
  which force-resyncs the ISS to the (absent) DUT state.
- Dual-trace has been exercised on CFG `default`, `pulp` and `pulp_fpu`;
  `pulp_fpu_zfinx` and the FPU-trap path have not. When diffing
  `dut.rvvi`/`ref.rvvi` on the PULP CFGs, expect a few ref-side deltas that
  sit outside the compare set: the custom hwloop CSR echoes
  (`0xCC0-0xCC2`) read 0 on the ref side, and a 2-byte PC offset on the
  final WFI line at exit. The occasional `fflags` NX delta on `fmadd.s`
  listed here before is gone with the flexfloat flag fixes: the ISS no
  longer raises a spurious inexact on an infinite FMA result.
- No single front-end target selects the mode (something like
  `make trace LEVEL=...`); use the command lines above.
- The in-simulation GPR/FPR compare covers the *written* set only (see
  "Known fragilities" in `ARCHITECTURE.md`): a stale value in a register
  the DUT never rewrites stays latent, and diffing the two trace files is
  the practical way to catch it.
- The format allows quoted `'…'` comment annotations, ignored by the
  checker; disassembly, symbol names or ABI register names could be
  attached later without changing the machine content.

## File map

| File | Role | Standard RVVI (vendored) / custom |
|------|------|-----------------------------------|
| `rvvi_text_writer.{hpp,cpp}` | canonical formatter — header, line grammar, CSR dedup; pure C++17 + stdio | custom |
| `rvvi_text_dpi.{hpp,cpp}` | DPI shim over the formatter for the SV tracer (`rvviText*`); builds into `librvvi_text.so` | custom |
| `test/test_rvvi_text_writer.cpp` | formatter unit test (`make test`) | custom |
| `rvvi_api2gvsoc.cpp` | DPI bridge (RVVI-API → GVSOC): emitter adapter, env gate `RVVI_TEXT_TRACE`, ref-only / MODE / FLEN setters | implements the vendored RVVI-API, plus the custom extensions below |
| `rvvi_trace2api.sv` | SV driver: consumes `rvviTrace`, pushes DUT state and MODE, calls the batched retire-and-compare | consumes the vendored RVVI-TRACE interface; the batched call and the MODE push are custom |
| `cv32e40p/tb/uvmt/uvmt_cv32e40p_rvvi_text_tracer.sv` | DUT tracer: drives `rvvi_if` in RTL-only mode, read-only in dual-trace | custom (superproject) |
| `cv32e40p/tb/uvmt/uvmt_cv32e40p_gvsoc_wrap.sv` | co-sim wrap: `ref_init` calls `rvviBridgeSetRefOnly` (dual-trace) and `rvviBridgeSetFlen` (always) before `rvviRefInit` | custom (superproject) |
| `cv32e40p/tb/uvmt/rvvi_trace.flist` | RTL-only compile sources: `rvviTrace.sv` + tracer, no `rvviApiPkg.sv` | custom (superproject) |
| `cv32e40p/tb/uvmt/rvvi_trace_dual.flist` | dual-trace extra source on top of `gvsoc.flist`: the tracer only | custom (superproject) |
| `mk/uvmt/vsim.mk`, `mk/Common.mk`, `mk/uvmt/uvmt.mk` | mode selection and defines, `librvvi_text.so` hookup, `check-rvvi` target | custom (superproject) |
| `Makefile` | builds `rvvi_text_writer.o` into the three `.so`; `make test` | custom |
| `RVVI/RVVI-TEXT/README.md` | format spec v0.4 | vendored (riscv-verification/RVVI) |
| `RVVI/source/host/rvvi/rvviTextChecker.py` | conformance checker | vendored |
| `RVVI/source/host/rvvi/rvviTextParser.sv` | replayer skeleton (`+traceFile=`) | vendored |

### Standard vs custom DPI surface

The official RVVI pipeline is
`DUT → rvviTrace → rvvi_trace2api.sv → RVVI-API → rvvi_api2gvsoc.cpp → GVSOC`.
The following functions are not part of the vendored RVVI spec; they are
custom extensions of this bridge/tracer stack. The `rvviBridge*` prefix is
deliberate — it avoids collision with the vendored `rvviDut*`/`rvviRef*`
namespaces.

| Function(s) | Purpose |
|-------------|---------|
| `rvviRefRetireAndCompare` | batched ISS step + compares in one DPI crossing (performance) |
| `rvviRefInjectIrq`, `rvviRefSetInformedIrq` | OVPSim-style informed IRQ injection parity |
| `rvviRefIsFinished` | end-of-simulation poll from the wrap's watchdog |
| `rvviBridgeSetMode` | per-retire DUT privilege mode push for the ref-only emitter |
| `rvviBridgeSetRefOnly` | switches the bridge to ref-only file emission (dual-trace mode) |
| `rvviBridgeSetFlen` | CFG-derived FLEN for the `PARAMS` header |
| `rvviText*` family (`rvviTextOpen`, `rvviTextSetGpr`, `rvviTextSetFpr`, `rvviTextSetCsr`, `rvviTextSetMode`, `rvviTextWrite`, `rvviTextClose`) | tracer-side shim over the formatter, in `librvvi_text.so` |

Maintained alongside `ARCHITECTURE.md` (engine + bridge) and
`GVSOC_ENGINE.md` (GVSOC engine internals).
