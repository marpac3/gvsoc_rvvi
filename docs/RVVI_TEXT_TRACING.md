# RVVI-TEXT Tracing

CV32E40P instruction tracing in the RVVI-TEXT format. One canonical C++
formatter (`rvvi_text_writer.{hpp,cpp}`) serves three run modes:

- **RTL-only mode** (`RVVI_TRACE=YES USE_ISS=NO`) — a standalone SV tracer is
  the only producer; it emits `dut.rvvi` through `librvvi_text.so`, no ISS in
  the loop.
- **bridge-emit mode** (`USE_ISS=YES ISS=GVSOC`, no `RVVI_TRACE`, env
  `RVVI_TEXT_TRACE` set) — the DPI co-simulation bridge emits both `dut.rvvi`
  and `ref.rvvi`.
- **dual-trace mode** (`USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES`) — co-simulation
  plus the SV tracer: the tracer is the sole `dut.rvvi` producer, the bridge
  is switched to ref-only and emits only `ref.rvvi`.

In every mode the step-and-compare path of the co-simulation is untouched;
trace emission is additive.

---

## 1. The RVVI-TEXT format

RVVI-TEXT is the open Imperas text format for retired-instruction traces
(spec vendored at `RVVI/RVVI-TEXT/README.md`, v0.4, from
riscv-verification/RVVI). One line per retirement records the PC, the opcode,
and the architectural write-set of that instruction. A trace is
self-describing (header) and machine-checkable — the conformance gate is the
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

**Header.** `VERSION 0 1` is emitted for byte-per-byte parity with the golden
Imperas example (`RVVI/RVVI-TEXT/examples/traps.rvvi`); the checker accepts
it. The `VENDOR` string must start with a letter and contain no hyphen
(checker STRING rule). `FLEN` in `PARAMS` is CFG-derived on both producers
(§2).

**Trap semantics.** The meaning of `TRAP` comes from RVVI-TRACE (the
interface): a synchronous exception, i.e. the faulting instruction does *not*
retire. Concretely:

- A sync exception (illegal instruction, misaligned access, access fault,
  `ECALL`/`EBREAK`) produces a `TRAP` line at the faulting PC carrying the
  four trap-entry CSRs (`mstatus`, `mepc`, `mcause`, `mtval`) and **no
  `X`/`F` tokens** — a faulting instruction retires no register writes.
- An async interrupt produces **no** `TRAP` line: RVVI-TEXT has no `intr`
  element, so the handler entry is a normal `RET` (with the trap-entry CSR
  writes attached), as the standard prescribes.
- At the trap seam the `ref.rvvi` line reuses DUT data (values and privilege
  mode): the ISS has not consumed the trap yet at that boundary, so its own
  post-trap state is not observable there.

The checker validates `TRAP` lines syntactically only (its `check_TRAP` is
identical to `check_RET`); the semantics above follow the RVVI-TRACE
definitions and the Imperas example, not a checker constraint.

**CSR dedup is a formatter invariant.** One `C` token per address per line,
emitted at the address's first position with the last value pushed
(`rvvi_text_write_line` in `rvvi_text_writer.cpp`). Producers may
legitimately push the same address twice within one retirement (§5); the
formatter absorbs it.

---

## 2. Architecture today

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

- `libgvsoc_rvvi.so` / `libgvsoc_rvvi_zfinx.so` — the DPI co-simulation
  bridge (`rvvi_api2gvsoc.cpp`) calls it through a thin adapter fed from the
  bridge's DUT-push globals and the ISS getters.
- `librvvi_text.so` — a standalone DPI shim (`rvvi_text_dpi.{hpp,cpp}`, the
  `rvviText*` functions, zero GVSOC dependency) used by the SV tracer.

### 2.1 The two producers

- **SV tracer** (`cv32e40p/tb/uvmt/uvmt_cv32e40p_rvvi_text_tracer.sv`, in the
  superproject) reads the `rvviTrace` interface (`rvvi_if`), so the privilege
  MODE comes from the existing wiring and no CSR logic is re-implemented in
  SV. In RTL-only mode it also drives the interface itself (via the shared
  RVFI→RVVI wiring `uvmt_cv32e40p_iss_wrap_common.svh`); in dual-trace mode
  that driving logic compiles out under `` `ifndef USE_ISS ``, making the
  tracer a read-only observer of the `rvvi_if` that
  `uvmt_cv32e40p_gvsoc_wrap.sv` drives — never a second driver.
- **DPI bridge** (`rvvi_api2gvsoc.cpp`) emits at the end of the batched
  retire-and-compare (and in `rvviDutTrap` for trap lines). The write-set
  (*which* GPR/FPR/CSR changed) always comes from the DUT push; the `dut`
  line takes DUT values and the `ref` line takes ISS values for the same
  indices, which is what makes the two files line up 1:1 for a plain `diff`.

### 2.2 Mode matrix

Two independent dimensions select the behavior: compile-time defines
(`USE_ISS`/`USE_GVSOC`, `RVVI_TRACE`) and the runtime environment variable
`RVVI_TEXT_TRACE=<dir>|1`, which enables emission on the bridge side
(unset/empty/`0` → bridge emission off, zero hot-path overhead). Mode
selection is compile-time only; there is no runtime plusarg to toggle the
tracer.

| Mode | Compile-time | Runtime | `dut.rvvi` | `ref.rvvi` | step-and-compare |
|------|-------------|---------|------------|------------|------------------|
| no-trace | any, without `RVVI_TRACE` | `RVVI_TEXT_TRACE` unset | — | — | yes if co-sim |
| RTL-only | `RVVI_TRACE`, no `USE_ISS` | `+rvvi_text_dut=` optional | SV tracer | — | — |
| bridge-emit | `USE_ISS`+`USE_GVSOC`, no `RVVI_TRACE` | `RVVI_TEXT_TRACE=<dir>\|1` | bridge | bridge | yes |
| dual-trace | `USE_ISS`+`USE_GVSOC`+`RVVI_TRACE` | `RVVI_TEXT_TRACE` optional (§3) | SV tracer | bridge (ref-only) | yes |
| Imperas co-sim | `USE_ISS`, no `USE_GVSOC` | — | — | — | yes (ImperasDV); `RVVI_TRACE=YES` ignored with a Make info message |

**MODE column.** The tracer always emits it. The bridge emits it only in
dual-trace mode and only on `ref.rvvi` (`has_mode` is keyed on ref-only being
active *and* on which file is being written — see §5 for why not on the
dut/ref data-source flag). In bridge-emit mode neither file carries MODE,
which keeps that output byte-compatible with the pre-extraction emitter. The
per-retire mode value reaches the bridge through `rvviBridgeSetMode`, called
by `rvvi_trace2api.sv`.

**FLEN.** CFG-derived on both sides: the tracer computes
`FLEN = (FPU != 0) ? 32 : 0` from its `FPU` parameter; the bridge receives
the same value through `rvviBridgeSetFlen`, called unconditionally from
`gvsoc_wrap`'s `ref_init` before `rvviRefInit`. The two headers therefore
agree on every CFG.

**Ref-only switch.** In dual-trace mode `gvsoc_wrap`'s `ref_init` calls
`rvviBridgeSetRefOnly` (under `` `ifdef RVVI_TRACE ``) before `rvviRefInit`,
where the file-open decision happens: the bridge then opens only `ref.rvvi`
and the tracer is the sole `dut.rvvi` producer.

### 2.3 Build wiring

- RTL-only: `mk/uvmt/vsim.mk` adds `+define+RVVI_TRACE` and compiles
  `cv32e40p/tb/uvmt/rvvi_trace.flist` — `rvviTrace.sv` plus the tracer.
  `rvviApiPkg.sv` is deliberately *not* compiled: nothing in the RTL-only
  build references the RVVI API. The DPI library is `librvvi_text.so`
  (`RVVI_TEXT_MODEL` in `mk/Common.mk`).
- Dual-trace: `vsim.mk` adds `+define+RVVI_TRACE` and compiles
  `cv32e40p/tb/uvmt/rvvi_trace_dual.flist` on top of `gvsoc.flist` — the
  tracer only, since the interface and API package are already in
  `gvsoc.flist`.
- `uvmt_cv32e40p_tb.sv` instantiates `rvvi_if` under
  `` `ifdef USE_ISS `` / `` `elsif RVVI_TRACE ``; the standalone tracer under
  `` `elsif RVVI_TRACE ``; and, in dual-trace, the tracer alongside
  `gvsoc_wrap` on the same `rvvi_if` instance.

### 2.4 Robustness details

- `rvviTextOpen` returns a status; the tracer raises `$error` on failure — a
  passing run that silently wrote no trace is easy to miss in a batch.
- Both the bridge and the shim flush their file(s) every 1000 retirements, so
  a killed run keeps its trace tail.
- On a trap the bridge takes an emit-only snapshot of the GPR/FPR write masks
  before they are cleared for the next retirement; the compare masks are
  untouched (§5).
- The tracer drives `mtval` itself in RTL-only mode, because the shared
  include gates that wiring behind `` `ifdef USE_GVSOC `` (§5).

---

## 3. How to run

All commands from `cv32e40p/sim/uvmt/`. Load the simulator license first:

```bash
module load questa/2025.3
```

**RTL-only mode** — RTL simulation, no ISS; the SV tracer writes `dut.rvvi`:

```bash
make test TEST=hello-world RVVI_TRACE=YES USE_ISS=NO
```

`dut.rvvi` lands in the run directory. Redirect with `RVVI_TEXT_TRACE=<dir>`
on the make line — `vsim.mk` creates the directory if missing and wires it to
the tracer's `+rvvi_text_dut=<path>` plusarg.

**Bridge-emit mode** — DPI co-simulation; the bridge writes both files:

```bash
make test TEST=hello-world USE_ISS=YES ISS=GVSOC RVVI_TEXT_TRACE=<dir>
```

`RVVI_TEXT_TRACE=<dir>` → `<dir>/dut.rvvi` + `<dir>/ref.rvvi`;
`RVVI_TEXT_TRACE=1` → both files in the run cwd.

**Dual-trace mode** — DPI co-simulation plus the tracer; the tracer writes
`dut.rvvi`, the bridge writes `ref.rvvi`:

```bash
make test TEST=hello-world USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES RVVI_TEXT_TRACE=<dir>
```

**Conformance gate** on whatever a run produced:

```bash
make check-rvvi RVVI_TRACE_DIR=<dir>
```

(`mk/uvmt/uvmt.mk`: runs the vendored checker on whichever of
`dut.rvvi`/`ref.rvvi` exists in `<dir>`; fails if neither exists or the
checker rejects one.)

To confirm the bridge emitter engaged, grep the run log
(`vsim_results/<CFG>/<TEST>/0/vsim-<TEST>.log`) for `RVVI-TEXT ->`, printed
by `rvviRefInit`; it shows `ref-only=1` in dual-trace mode. With `ISS` set to
anything but GVSOC, `RVVI_TRACE=YES` is ignored and make prints
`Info: RVVI_TRACE=YES ignored - dual-trace requires ISS=GVSOC`.

---

## 4. Design history and remaining work

The formatter and the three modes landed in phases, each leaving a working
state:

- Formatter extracted from the bridge's inline emitter into
  `rvvi_text_writer.{hpp,cpp}`, output byte-identical — done.
- `TRAP` lines and `X`/`F` delta emission added on the bridge — done.
- RTL-only mode: SV tracer + `librvvi_text.so` + build wiring — done.
- Dual-trace mode: tracer as sole `dut.rvvi` producer, bridge ref-only — done.
- Robustness pass: CSR dedup moved into the formatter, open-status
  propagation, CFG-derived FLEN on the bridge, `make check-rvvi` — done.

Remaining work:

- **GVSOC-only headless runner** — trace the ISS with no RTL and no Questa in
  the loop. Not started. Requires per-retire write-set capture inside
  `gvsoc_engine` (the bridge derives the write-set from the DUT push, which
  does not exist without a DUT) and must bypass `rvviRefEventStep`, which
  force-resyncs the ISS to the absent DUT state.
- **Dual-trace on the remaining CFGs** — validated on CFG `default` only
  (§6); `pulp`, `pulp_fpu`, `pulp_fpu_zfinx` — in particular `F` columns and
  the FPU-trap path — are unexercised.
- **`make trace LEVEL=...` UX** — a single front-end target selecting the
  mode. Not started.
- Unscheduled option: the format supports quoted `'…'` comment annotations,
  ignored by the checker — a later extension could attach disassembly, symbol
  names, or ABI register names without changing the machine content.

---

## 5. Findings and post-mortem

Real bugs found by empirical validation of the modes; none was visible from
code reading alone, which is why they are recorded here. All fixed except the
last item.

**Duplicate CSR tokens on `TRAP` lines.** On a real trap, the SV driver's
generic `csr_wb` sparse scan and its explicit trap-CSR re-push both fired for
the same addresses, so `mstatus`/`mepc`/`mcause` appeared twice on every
`TRAP` line — in the bridge path (`rvvi_trace2api.sv` → `rvviDutCsrSet`) and,
independently, in the tracer path (`rvviTextSetCsr`). The checker was silent
because it validates each token independently. Fix: dedup is now owned by the
formatter — one `C` token per address, first position, last value — and
covered by a regression unit check (`test/test_rvvi_text_writer.cpp`).

**MODE missing on `TRAP` lines of `ref.rvvi`.** The first MODE implementation
keyed `has_mode` on the `dut_side` data-source flag, assuming
`dut_side=false` always means "writing the ref file". On the trap path that
is false: the ref line deliberately reuses DUT data (§1), so `dut_side` is
true while the target is the ref file — MODE silently vanished from every
ref-side `TRAP` line while `RET` lines stayed correct; the MODE-token gap
between the two files exactly matched the `TRAP`-line count. Fix: key on
which file is being written (`fp == g_rvvi_text_ref_fp`), never on the data
source.

**Illegal nested `generate`.** The first attempt to make the tracer's driving
logic conditional wrapped the shared include in a generate-if region.
`uvmt_cv32e40p_iss_wrap_common.svh` contains a top-level explicit
`generate`/`endgenerate` block, and SystemVerilog forbids nesting one inside
another generate scope — Questa error `vlog-13205`. The dual-trace build
masked the bug (its include guard made the tracer's include a no-op, since
`gvsoc_wrap` includes the file first); the RTL-only build tripped it. Fix: a
plain preprocessor guard, `` `ifndef USE_ISS ``, around the block — the mode
split is a compile-time fact, so the preprocessor is the right mechanism.

**`mtval` undriven in RTL-only mode.** The shared include gates the `mtval`
trap-CSR wiring behind `` `ifdef USE_GVSOC ``, so the RTL-only build left it
undriven and trap lines emitted `mtval=0`. Fix: the tracer drives it itself
(`RVVI_SET_TRAP_CSR` on `CSR_MTVAL_ADDR` in
`uvmt_cv32e40p_rvvi_text_tracer.sv`).

**`X`/`F` deltas zeroed mid-retire.** The bridge's `record_dut_event` zeroes
the GPR/FPR write masks before the emitter read them, so early traces carried
zero `X`/`F` tokens. Fix: an emit-only snapshot of the masks taken before the
zeroing; the compare masks are untouched, so the step-and-compare path is
unchanged by construction.

**`FLEN` header mismatch on no-FPU CFGs.** The tracer derived FLEN from its
`FPU` parameter while the bridge hardcoded `FLEN 32`, so on CFG `default` the
two headers of a dual-trace run disagreed (`FLEN 0` vs `FLEN 32`) — no impact
on data rows (no `F` lines exist without an FPU), but a permanent header
diff. Fixed: `rvviBridgeSetFlen` pushes the CFG-derived value from
`gvsoc_wrap` before `rvviRefInit`.

**Pre-existing, out of scope, not touched:** the masked GPR/FPR *compare*
inside the batched retire-and-compare is dead in DPI co-simulation. The same
zeroed masks that broke the emitter make `rvviRefGprsCompareWritten` /
`rvviRefFprsCompare` skip every index, and the unmasked variants are never
called by the driver — a wrong register value only surfaces indirectly, via a
later PC/CSR divergence. Reviving the compare changes pass/fail behavior and
needs its own regression-validated change.

---

## 6. Validation record and file map

### 6.1 Validation record

| What | Result | Reproduce |
|------|--------|-----------|
| Formatter unit test | 12/12 checks pass (header, RET/TRAP, X/F/C ordering, CSR dedup, MODE, DPI shim open-fail status) | `cd vendor_lib/gvsoc_rvvi && make test` |
| Formatter extraction gate | `dut.rvvi`/`ref.rvvi` byte-identical (md5) on `hello-world` between the original inline emitter and the extracted formatter, at the extraction point | run the §3 bridge-emit command twice with the same `RVVI_TEXT_TRACE=<dir>` — once built at the pre-extraction revision, once at the extraction revision — then `md5sum` the four files. Not re-runnable against HEAD 1:1: later features (`TRAP`, `X`/`F`, MODE) intentionally extend the stream |
| RTL-only mode | 4/4 tests `SIMULATION PASSED` + checker exit 0 on each `dut.rvvi`: `hello-world`, `cv32e40p_csr_access_test` (CFG `default`), `pulp_hardware_loop` (CFG `pulp`, exercises `TRAP`), `fpu_func_cov_improve_test` (CFG `pulp_fpu`, exercises `F`) | `make test TEST=<t> CFG=<cfg> RVVI_TRACE=YES USE_ISS=NO && make check-rvvi RVVI_TRACE_DIR=<run dir>` |
| Dual-trace mode (CFG `default`) | `illegal_instr_test`: 524108 event lines per file (476512 `RET` + 47596 `TRAP`), 0 duplicate `C` tokens, 0 lines missing MODE, checker exit 0 on both files; `dut.rvvi`/`ref.rvvi` headers identical (`FLEN 0` on the no-FPU CFG) | `make test TEST=illegal_instr_test USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES RVVI_TEXT_TRACE=<dir> RNDSEED=1 && make check-rvvi RVVI_TRACE_DIR=<dir>` |
| Cross-mode equivalence (`hello-world`, fixed seed) | the dual-trace `dut.rvvi` is byte-identical to the RTL-only `dut.rvvi` (`diff` empty; both tracer-produced) and identical to the bridge-emit `dut.rvvi` once the `MODE` token is stripped; the plain co-sim baseline is unchanged by the trace wiring (`SIMULATION PASSED`, `UVM_ERROR : 0`) | run the three §3 command lines with `TEST=hello-world RNDSEED=1`, one `RVVI_TEXT_TRACE` dir per mode, then `diff` the `dut.rvvi` files |
| Dual-trace mode, PULP/FPU CFGs | `pulp_hardware_loop` (CFG `pulp`): `SIMULATION PASSED`, 12192 lines per file, headers identical (`FLEN 0`), checker exit 0 on both, 16 `TRAP` per side. `matmul_32b_float` (CFG `pulp_fpu`): `SIMULATION PASSED`, 18954 lines per file, headers identical (`FLEN 32`), `F` writes equal on both sides. Known ref-side deltas, all outside the compare set: the custom hwloop CSR echoes (`0xCC0-0xCC2`) read 0 on the ref side, one `fflags` NX delta on an `fmadd.s`, and a 2-byte PC offset on the final WFI line snapshot at exit | `make test TEST=<t> CFG=<cfg> USE_ISS=YES ISS=GVSOC RVVI_TRACE=YES RVVI_TEXT_TRACE=<dir> && make check-rvvi RVVI_TRACE_DIR=<dir>` |

### 6.2 File map

| File | Role | Standard RVVI (vendored) / custom |
|------|------|-----------------------------------|
| `rvvi_text_writer.{hpp,cpp}` | canonical RVVI-TEXT formatter — header, line grammar, CSR dedup; pure C++17 + stdio, no GVSOC dependency | custom |
| `rvvi_text_dpi.{hpp,cpp}` | DPI shim over the formatter for the SV tracer (`rvviText*` functions); builds into `librvvi_text.so` | custom |
| `test/test_rvvi_text_writer.cpp` | formatter unit test (`make test`) | custom |
| `rvvi_api2gvsoc.cpp` | DPI bridge (RVVI-API → GVSOC): emitter adapter, env gate `RVVI_TEXT_TRACE`, ref-only / MODE / FLEN setters | implements the vendored RVVI-API, plus the custom extensions of §6.3 |
| `rvvi_trace2api.sv` | SV driver: consumes `rvviTrace`, pushes DUT state and MODE, calls the batched retire-and-compare | consumes the vendored RVVI-TRACE interface; the batched call and the MODE push are custom |
| `cv32e40p/tb/uvmt/uvmt_cv32e40p_rvvi_text_tracer.sv` | DUT tracer: drives `rvvi_if` in RTL-only mode (`` `ifndef USE_ISS ``), read-only in dual-trace mode | custom (superproject) |
| `cv32e40p/tb/uvmt/uvmt_cv32e40p_gvsoc_wrap.sv` | co-sim wrap: `ref_init` calls `rvviBridgeSetRefOnly` (dual-trace) and `rvviBridgeSetFlen` (always) before `rvviRefInit` | custom (superproject) |
| `cv32e40p/tb/uvmt/rvvi_trace.flist` | RTL-only compile sources: `rvviTrace.sv` + tracer, no `rvviApiPkg.sv` | custom (superproject) |
| `cv32e40p/tb/uvmt/rvvi_trace_dual.flist` | dual-trace extra source on top of `gvsoc.flist`: the tracer only | custom (superproject) |
| `mk/uvmt/vsim.mk`, `mk/Common.mk`, `mk/uvmt/uvmt.mk` | mode selection and defines, `librvvi_text.so` hookup, `check-rvvi` target | custom (superproject) |
| `Makefile` | builds `rvvi_text_writer.o` into `libgvsoc_rvvi.so`, `libgvsoc_rvvi_zfinx.so`, `librvvi_text.so`; `make test` | custom |
| `RVVI/RVVI-TEXT/README.md` | format spec v0.4 | vendored (riscv-verification/RVVI) |
| `RVVI/source/host/rvvi/rvviTextChecker.py` | conformance checker | vendored |
| `RVVI/source/host/rvvi/rvviTextParser.sv` | replayer skeleton (`+traceFile=`) | vendored |

### 6.3 Standard vs custom DPI surface

The official RVVI pipeline is
`DUT → rvviTrace → rvvi_trace2api.sv → RVVI-API → rvvi_api2gvsoc.cpp → GVSOC`.
The following functions are **not** part of the vendored RVVI spec; they are
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

---

*Maintained alongside `ARCHITECTURE.md` (engine + bridge) and
`GVSOC_ENGINE.md` (GVSOC engine internals).*
