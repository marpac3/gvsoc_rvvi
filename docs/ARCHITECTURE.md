# Architecture — GVSOC engine + RVVI bridge

How `gvsoc_rvvi` drives the embedded GVSOC engine for CV32E40P RVVI
step-and-compare co-simulation, why it does it this way, and how it differs
from the official GVSOC examples. Three points frame everything else: the
engine lifecycle follows the official GVSOC external-control pattern; the
step model does not (we are retire-based, the official examples are
time-driven — a difference forced by RVVI, not a bug); and the real source
of fragility is neither of those but the direct reach-in into ISS state,
which is unavoidable because the public GVSOC API exposes no architectural
state readback at all.

Code references use function and symbol names; line numbers are avoided
because they rot.

## Why a dedicated bridge

The goal is an instruction-accurate step-and-compare co-simulation: for each
instruction retired by the RTL DUT (CV32E40P), the reference model (the
GVSOC ISS) advances by one instruction and the architectural state — PC,
GPR, FPR, CSR — is compared. That is what the RVVI standard (RISC-V
Verification Interface) requires: a comparison at every retire.

GVSOC, however, is an event-driven virtual platform designed for modeling
and for time-domain co-simulation, not for architectural step-and-compare.
Two gaps follow, and the bridge exists to close them:

| RVVI needs | Public GVSOC API offers | Consequence |
|---|---|---|
| stopping at the instruction boundary (retire) | only time stepping (`step(ps)` / `step_until(t)`) | the retire is taken from a commit stream the core personality records |
| reading the model's PC/GPR/CSR/FPR | no state readback (only Io/Wire/Vcd/Power bindings) | direct reach-in into the ISS structs |

## The v2 bridge

The engine wrapper `gvsoc_engine_v2.cpp` sits behind a pure-C API
(`gvsoc_engine.hpp`) and drives the modular `iss_v2` core model. It passes
the full no-pulp regression and is validated with `test/quick_val.sh`
across the four supported configs. (The "v2" in the file and library names
is a generation label: it replaced a retired first-generation bridge built
on the original `iss` core model.)

The load-bearing design points, in one screen:

- **Retire = commit stream.** The CV32E40P personality's `Cv32e40pEvents`
  slot (pulp repo, `cpu/iss_v2/{include,src}/cores/cv32e40p/`) records
  every architectural commit in a ring: instructions that park in the
  commit FIFO are captured at issue and drained through the
  `insn_stall_account` hook, synchronous instructions commit inline.
  `gvsoc_engine_step()` pops exactly one commit per call, clocking the
  engine only while the ring is empty. A bounded cycle budget and the
  runaway net act as safety checks (a timeout means "no new commit showed
  up").
- **Capability API.** Four queries in the engine header serve the
  retire/backlog/trap-window bookkeeping the RVVI front-end needs:
  `gvsoc_engine_commit_stream()`;
  `gvsoc_engine_pending_commits()` (backlog left by a multi-commit clock —
  while non-zero the sampled ISS state is ahead of the retire being served,
  so state compares are gated to the tail of the drain burst);
  `gvsoc_engine_state_current()` (0 when an asynchronous trap redirect
  landed inside a drain window — detected through a `trap_seq` stamp on
  each commit — so state compares on that retire are skipped until the
  stream re-arms); and `gvsoc_engine_materialize_commit()` (peek the next
  commit PC without consuming it, used on DUT trap rows: an instruction
  that executes and then traps, ecall-class, commits and is consumed on the
  trap row; a refused instruction, illegal-class, never commits).
- **IRQ = wire-only.** `rvviRefNetSet` drives one of the 19 lines (`msi`,
  `mti`, `mei`, `external_irq_16..31`) of the `cv32e40p_irq_injector`
  component (pulp repo, `pulp/cv32e40p_irq_injector/`), bound once at init
  with `wire_bind` — the GVSOC channel intended for raw signals. There is
  no direct `mip` poke: `mip` is wire-driven, and the personality
  exposes a read-only `mip_view` CSR front-end to software. WFI wake
  happens natively through the ISS interrupt check. The informed-injection
  mechanism (`settle_irq`, `take_irq_for_one_step`) survives unchanged on
  top of the wires.
- **Drain before redirect.** `gvsoc_engine_set_pc` lets parked work finish
  before flushing the commit stream: while an instruction sits in the
  commit FIFO or an LSU response is in flight, dropping it would leave its
  load-use scoreboard bit set forever and deadlock the next reader. The
  drain is bounded and skipped on a held WFI; the architectural state is
  forced from the DUT after the redirect either way.
- **Hwloop CSRs in the compare set.** `gvsoc_engine_get_csr` resolves
  `lpstart`/`lpend`/`lpcount` (0xCC0-0xCC6) through the hwloop module
  accessors and the personality's architectural `lpend` shadow (the module
  itself stores the loop-back point, LPEND-4), so the TB compares them on
  every retire like any other mapped CSR.
- **Personality, not `#ifdef`s.** The v2 core model is assembled by the
  recipe `pulp/cpu/iss/cv32e40p_v2.py` from substitutable class slots (Csr,
  Irq, Core, Exception, Regfile, Events, Exec, priv); the shared iss_v2
  sources carry no CV32E40P `#ifdef`.
- **Platforms on io_v2.** The v2 configs use the
  `cv32e40p-v2-standalone{,-fpu,-zfinx,-nopulp}` targets, built on the v2
  I/O stack (`router_v2`, `memory_v3`, `loader_v2`, the v2 sparse memory
  and exit device). Config templates are `gvsoc_config_v2_<CFG>.json`,
  generated by `make config-v2`.
- **Libraries.** `libgvsoc_rvvi_v2.so` / `libgvsoc_rvvi_v2_zfinx.so`,
  selected by `mk/Common.mk` (ZFINX picks the `_zfinx` variant).

What the commit stream does not change: the engine lifecycle, the
compilation firewall and the reach-in. There is still no public
state-readback API, so the bridge acquires the core with
`get_component("soc/core")` — the component is the `Iss` object itself,
iss_v2 has no wrapper component — and reads public struct members, with
`static_assert` layout tripwires and a runtime layout canary (known-value
CSR readback at init) as the guard rails. The sections below describe that
machinery.

## Two layers: the compilation firewall

```
SV testbench  --DPI-->  rvvi_api2gvsoc.cpp  --pure-C ABI-->  gvsoc_engine_v2.cpp  --C++ API-->  gv::Gvsoc + ISS
 (rvviTrace)   (~60 fn)   [RVVI front-end]   (gvsoc_engine.hpp)   [engine wrapper]   (heavy ISS/GVSOC headers)
```

`rvvi_api2gvsoc.cpp` is the RVVI front-end: it implements the ~60
`extern "C"` `rvvi*` functions and includes only `gvsoc_engine.hpp` (a
lightweight pure-C header) plus `svdpi.h`/VPI. It knows RVVI and DPI, not
GVSOC. The engine wrapper — `gvsoc_engine_v2.cpp` — includes the heavy
GVSOC/ISS headers, knows GVSOC,
and knows nothing about DPI. `gvsoc_engine.hpp` is the firewall between
them — Questa's `svdpi.h` and the ISS model headers have colliding macros
and types, and keeping the two header sets out of any common translation
unit eliminates that whole class of build errors.

## The official external-control pattern

GVSOC documents control "from an external simulator" in tutorial 17
(`gvsoc/core/docs/developer_manual/tutorials/17_how_to_control_gvsoc_from_an_external_simulator/`,
prose in `tutorials.rst`, canonical solution in `solution/launcher.cpp`).
The authoritative API is `gvsoc/engine/engine/include/gv/gvsoc.hpp`; the
user manual covers the out-of-process alternative in `remote_control.rst`
and `target_control.rst`.

The tutorial prescribes, for in-process synchronous control:

```cpp
gv::GvsocConf conf = { .config_path=…, .api_mode=gv::Api_mode_sync };
gv::Gvsoc *gvsoc = gv::gvsoc_new(&conf);
gvsoc->open();
io = gvsoc->io_bind(this, "/soc/axi_proxy", "");   // MMIO binding (the tutorial's use case)
gvsoc->start();
gvsoc->step(10000000000);                          // 10 ms of boot
for (...) { axi->access(&req); gvsoc->step(1000000000); }
gvsoc->quit(0); gvsoc->join(); gvsoc->close();
```

Two official rules matter to us. First, synchronous mode
(`Api_mode_sync`) runs the engine in the caller's thread and executes every
command inline; it is the documented choice for deterministic control (the
header's default is async). Second, "no GVSOC API can be called from this
callback" — repeated on all the `Io_user`/`Vcd_user` callbacks: the engine
must not be re-entered from inside a handler.

There are two control planes: in-process C++ (tutorial 17, the one we use)
and remote/proxy (`GvsocConf::proxy_socket` plus `vp/proxy.hpp`, socket
control from a separate process). For DPI lockstep the in-process plane is
the right choice — zero socket latency.

## Engine lifecycle

Single entry point: `gvsoc_engine_init()`, called from `rvviRefInit()`.
Three phases:

1. create — `g_conf.api_mode = Api_mode_sync`, `gv::gvsoc_new(&g_conf)`,
   then `g_gvsoc->bind(&g_user)`, registering `BridgeUser` (a subclass of
   `gv::Gvsoc_user`);
2. open and start — `open()` elaborates the component graph, then
   `start()`;
3. acquire the core — `get_component("soc/core")` (the returned component
   is the `Iss` object itself), `build_csr_map()`, then CSR reset forcing
   (below).

`BridgeUser` overrides only two callbacks: `has_ended(status)` (the exit
device ended the simulation — sets `g_finished`) and `has_stopped()`
(a no-op: stepping is driven by hand). The remaining callbacks
(`handle_step_end`, `was_updated`, `handle_syscall_stop`) are not needed in
synchronous mode.

Shutdown (`gvsoc_engine_shutdown`, from `rvviRefShutdown`) wraps each call
in try/catch, since DPI does not propagate exceptions:
`stop() → quit(0) → join() → close()`. The graceful sequence runs only when
the simulation finished on its own (`g_finished`). On an abnormal
termination — mismatch-watchdog abort, UVM timeout — it is skipped
entirely: in sync mode `Controller::start()` leaves the engine mutex owned
by the external loop, and only the internal sim-finished path releases it,
so `stop()` would self-deadlock on the relock and `join()` would resume the
simulation. The process is exiting anyway; the OS reclaims the engine.

What we use of the `gv::Gvsoc` API, and what we deliberately don't:

| API | Used | Note |
|---|---|---|
| `gvsoc_new`, `bind`, `open`, `start`, `stop`, `quit`, `join`, `close` | yes | lifecycle as in the tutorial |
| `step(duration_ps)` | yes, unusually | not to "advance by N ps" but as the primitive of a retire poll loop (next section) |
| `get_component(path)` | yes | the only channel to reach ISS state |
| free-running `run`/`stop`, async `step` + `handle_step_end` | no | incompatible with retire-by-retire determinism |
| `get_time`, `get_next_event_time`, `update`, `wait_runnable` | no | we do not synchronize on time; `get_next_event_time` is a candidate improvement (see Future improvements) |
| `wire_bind`, `io_bind` | no | `io_bind` is inbound MMIO (not applicable); `wire_bind` could carry IRQs (see Future improvements) |
| `lock`/`unlock`, `flush`, `terminate` | no | `lock`/`unlock` documented as high-cost |
| `Vcd`, `Power`, `Testbench`, proxy | no | out of scope for RVVI co-sim |

## Step semantics: retire-based on a time-based API

The public API only advances time, so retire-based stepping is built on top
of it: the CV32E40P personality records every architectural commit in a
ring (see "The v2 bridge"), and `rvviRefEventStep` maps to
`gvsoc_engine_step()`, which pops exactly one commit per call, clocking the
engine one cycle at a time (`g_gvsoc->step(g_clock_ps)`, `g_clock_ps` =
20000 ps = 50 MHz) only while the ring is empty. Retires are not counted
through `instret` — GVSOC's `csr.instret` does not auto-increment, so it is
not a reliable retire counter; the commit stream is recorded by the
personality itself, so a commit is a commit, whatever the PC does.

Two safety nets bound the clocking:

- timeout (`STEP_MAX_CYCLES` (=2000) cycles without a new commit): returns
  "no retire" plus diagnostics;
- runaway detection: consecutive timeouts with an unchanged, non-WFI PC are
  counted; at `RUNAWAY_THRESHOLD` (=16) the sticky `g_runaway` flag latches
  (cleared only by init). A clean retire resets the counter; WFI is
  excluded. Exposed as bit 0x20 in the batched compare call.

Clocking cycle-by-cycle while the ring is empty is still polling in time —
`step()` was meant as "advance by N ps", not "advance until retire".
Adaptive stepping via `get_next_event_time()` (see Future improvements)
targets exactly this.

## Reading ISS state: the reach-in

The `gv::Gvsoc` API has no method to read PC, GPR, CSR or FPR: it exposes
only Io (memory), Wire (signals), Vcd (trace) and Power bindings.
Step-and-compare needs the state, so the bridge reaches in:
`get_component("soc/core")` — the component is the `Iss` object itself —
then direct access to the public struct members of the ISS.

| State | Symbol | Note |
|---|---|---|
| PC | `iss.exec.current_insn` | it is the current PC |
| GPR | `iss.regfile.regs[i]` | x0 always 0 |
| FPR | `iss.regfile.fregs[i]`, or `regs[i]` under ZFINX | `#ifdef ISS_SINGLE_REGFILE` |
| CSR | `g_csr_value_map[addr]` → `&csr.<reg>.value` | map built by `build_csr_map()` (~110 CSRs); `mip` is not read this way — it is wire-driven through the IRQ injector and mirrored to software as `mip_view` |
| WFI / commit ring | `iss.exec.wfi`, `iss.timing` | the commit ring lives in `iss.timing` |

The contract is strict: only public data members are accessed, never the
ISS methods (`get_csr()`, `access()`, …). Those methods are compiled into
the model `.so` files that GVSOC loads dynamically, and are not linkable at
DPI time when the simulator loads the bridge library. This one constraint
cascades:

- read fixups are re-implemented by hand: reading `.value` directly
  bypasses the ISS CSR access callbacks, so the getter reproduces them —
  e.g. rebuilding the mstatus SD bit from FS/XS and forcing MPP=M, or
  forcing mtval/tdata2 to 0 because they read as 0 in the RTL;
- write masks are re-implemented by hand: `gvsoc_engine_set_csr` applies
  the CV32E40P write mask itself (e.g. mstatus `0x7888`);
- reset forcing after the acquire: some CSRs (depc, mepc, mcause, …) are
  not brought to the RTL reset values by `Csr::reset()`, so the wrapper
  forces them — defense in depth against a reset-scope hole in the ISS.

The reach-in is not a convenience hack — it is imposed by the absence of an
introspection API. It is, however, the source of the ABI fragility listed
under Known fragilities.

One boundary is kept deliberately: the retire/backlog/trap-window
bookkeeping the RVVI front-end needs is served through the capability API
(`gvsoc_engine_commit_stream` / `pending_commits` / `state_current` /
`materialize_commit`, see "The v2 bridge") rather than read from ISS fields
by the front-end.

## RVVI API mapping

| RVVI function | Engine action |
|---|---|
| `rvviRefInit` | env + `init_net_map` + `create_temp_config` (injects the ELF into the JSON) + `gvsoc_engine_init` |
| `rvviRefShutdown` | `gvsoc_engine_shutdown` + performance/profile dump |
| `rvviRefProgramLoad` | stub (the ELF is already in the config) |
| `rvviRefNetIndexGet` / `rvviRefNetSet` | name→index lookup; `gvsoc_engine_set_irq` (drives the injector wire) + `settle_irq` |
| `rvviRefEventStep` | `gvsoc_engine_step` (pops one commit from the stream) + resync/realign logic |
| `rvviRefPcGet`/`GprGet`/`FprGet`/`CsrGet` | readback from the ISS |
| `rvviRefPcCompare`/`GprsCompare`/`CsrsCompare`/`FprsCompare` | DUT state vs ISS readback |
| `rvviRefCsrSetVolatile`/`Mask` | configure the compare (CSRs the ISS does not model cycle-by-cycle) |
| `rvviRefGprSet`/`FprSet`/`CsrSet` | direct injection into the ISS (used in `ref_init`, e.g. mtvec) |
| `rvviRefRetireAndCompare` | GVSOC-only batch (see below); also syncs performance-counter CSR reads — after the step, the ISS rd of a cycle/instret/hpm read is overwritten with the DUT rd value, so counter-dependent control flow (e.g. printing a cycle delta) cannot fork the two sides. `CV_RVVI_VOLATILE_CSR_SYNC=0` disables this |
| `rvviRefInjectIrq` / `rvviRefSetInformedIrq` | informed injection (see IRQ injection) |
| ~25 vector/memory/conn functions | stubs (not applicable to CV32E40P) |

State crosses the DPI boundary not as a packed struct but through scalar
function signatures plus the SV `rvviTrace` interface, read field-by-field
by the `rvvi_trace2api.sv` driver.

## IRQ injection

CV32E40P evaluates interrupts combinationally in the DECODE FSM; GVSOC
evaluates them post-execute. The result is an intrinsic ~1-cycle desync on
interrupt entry, and the bridge offers two ways to deal with it.

Reactive (the default). `rvviRefNetSet` → `gvsoc_engine_set_irq` maps the
RVVI net index onto one of the 19 lines of the `cv32e40p_irq_injector`
component (`msi`, `mti`, `mei`, `external_irq_16..31`), bound at init
through `wire_bind`. The wire lands in the personality's RISC-V interrupt
model, which updates `mip` and performs the WFI wake natively; software
sees `mip` through the read-only `mip_view` CSR front-end.
`settle_irq()` runs a few drain cycles so the prefetch guard
fires in the same cycle as the RTL. Since `skip_irq_check` is held asserted
(the ISS never takes IRQs on its own), the force-resync path in
`rvviRefEventStep` realigns PC, CSRs and GPR/FPR from the DUT when the DUT
enters an asynchronous trap — RVFI does not set `rvfi_trap` for async
traps, so the bridge has to detect the entry itself. Detection uses two
signatures: the MIE skew (DUT `mstatus.MIE` already 0, ISS still 1), and —
because the CSR sync can align `mstatus` before the divergence surfaces —
a DUT-side one: `mcause` with the interrupt bit set, `mepc` equal to the
PC the ISS currently sits on, and the row PC equal to the vectored target
(`mtvec_base + 4*cause`). The CSR restore covers the four trap CSRs plus
every compared non-volatile CSR (`mip` excluded: it mirrors the interrupt
wires); a WFI-parked ISS is first released through the legitimate wire
path (a wake edge held across one step on a `mie`-enabled cause) so the
redirect never lands on a stalled core — and when that wake step already
lands the ISS on the row's PC, it counts as the retire itself: no
redirect, no extra step (a second step would run past the just-retired
WFI).

Informed, in the style of OVPSim's "deferint" (opt-in via the
`+rvvi_informed_irq` plusarg). `rvviRefInjectIrq` detects the vectored
entry (DUT PC equal to `mtvec_base + cause*4`, with the ISS MIE still 1)
and calls `gvsoc_engine_take_irq_for_one_step(cause)`: it masks `mip` down
to the single cause (CV32E40P ranks the fast local IRQs above MEI, whereas
the generic `Irq::check()` would apply the standard RISC-V priority),
lowers `skip_irq_check` for exactly one step, and lets the ISS compute the
entry itself — mepc, mstatus.MIE→MPIE, mcause=(1<<31)|id,
`current_insn`=vector. Then it restores `mip` and the flags. The DUT is the
oracle for which interrupt and when; the ISS is the calculator; the bridge
compares as usual.

A trap costs two ISS steps: GVSOC models the entry as an extra step, so on
the retire of a trap the bridge issues one extra silent `rvviRefEventStep`
and skips the compare. This is intentional. In the same spirit,
pipeline-flush artifacts — a retire slot with PC 0 and a zero instruction
word — are skipped; a genuine retire at address 0 has a nonzero instruction
word and is processed normally.

## Debug entry

The bridge follows the DUT into debug mode the informed way, mirroring
the informed-IRQ pattern: the SV layer drives `rvvi.debug_mode` per
retire, and on its 0→1 edge the bridge calls
`gvsoc_engine_take_debug_for_one_step(cause)` so the ISS performs the
entry itself (dcsr.cause from the DUT, redirect into the debug ROM). At
the entry seam the bridge force-syncs from the DUT the CSRs the two sides
legitimately disagree on: `dpc` — CV32E40P *retires* the ebreak that
enters debug, so the ISS has already advanced past it and would capture
dpc one instruction high — and the hwloop CSRs (0xCC0-0xCC6: a preempted
hardware loop rolls back differently on the two sides). On the model
side, `dret` executed outside debug mode raises an illegal instruction
(per the Debug spec) instead of jumping through a stale dpc, and
`dcsr.prv` is WARL-pinned to M (the core implements no other privilege
mode). The residual, visible in the two debug×hwloop lanes (KNOWN_FAIL in
`test/quick_val.sh`), is a one-instruction retire misalignment inside the
debug ROM when debug entries re-arm in rapid succession; it is
characterized in the validation evidence but the root cause is still
open.

## RVVI conformance

| RVVI component | Status |
|---|---|
| RVVI-API (init/step/compare/csr/gpr/fpr) | ~60 functions implemented, ~25 vector/memory stubs where CV32E40P has no such state |
| RVVI-TRACE (the `rvviTrace` interface) | consumed on the SV side by `rvvi_trace2api.sv` |
| RVVI-TEXT (textual trace logging) | implemented in three modes (RTL-only, bridge-emit, dual-trace) from the single formatter `rvvi_text_writer.{hpp,cpp}` — see RVVI_TEXT_TRACING.md |
| RVVI-VVP | not used |

A few functions are extensions beyond the vendored RVVI standard, each with
a concrete reason to exist:

| Function | Why |
|---|---|
| `rvviRefRetireAndCompare` | one DPI crossing instead of five (`EventStep` + `PcCompare` + `GprsCompareWritten` + `CsrsCompare` + `FprsCompare`), plus the runaway bit. Purely a performance batch |
| `rvviRefInjectIrq`, `rvviRefSetInformedIrq` | IRQ parity with OVPSim; without them, every async IRQ produces false mismatches (the ±1-cycle desync, and RVFI not marking async traps) |
| `rvviRefIsFinished` | end-of-simulation polling from the `gvsoc_wrap` watchdog; the RVVI API does not expose "the ISS finished" |
| `rvviBridgeSetMode`, `rvviBridgeSetRefOnly`, `rvviBridgeSetFlen` | RVVI-TEXT emission: per-retire MODE on the ref side, ref-only switch in dual-trace mode, FLEN for the `PARAMS` header. The `rvviBridge*` prefix avoids the vendor `rvviDut*`/`rvviRef*` namespaces |
| `rvviText*` family (`Open`/`SetGpr`/`SetFpr`/`SetCsr`/`SetMode`/`Write`/`Close`) | DPI shim for the SV tracer (RTL-only and dual-trace modes), built into `librvvi_text.so` with zero GVSOC dependency |

Shrinking this custom surface is only partly possible: the batch call could
in theory be replaced by the five standard calls (giving up the performance
win), but the informed-IRQ functions cannot go away until RVVI standardizes
a deferint-like mechanism.

## Relation to the in-tree examples

GVSOC ships its own DPI wrapper (`gvsoc/engine/dpi-wrapper/src/dpi.cpp`)
and several launchers. They all belong to a time-driven family; this bridge
is retire-driven. The two solve different problems.

| In-tree example | Sync model | Technique |
|---|---|---|
| `dpi-wrapper/src/dpi.cpp` | time-driven | `step_until(SV_time)` + `get_next_event_time()`, sleep until the next event; ISS is a black box; stimulus via `wire_bind`/`io_bind` |
| `engine/src/main_systemc.cpp` | time-driven (SystemC) | `SC_THREAD`: `step_until(sc_time)` + `get_next_event_time()` + `wait()`; `was_updated`/`has_ended` callbacks |
| `engine/src/launcher.cpp`, `main.cpp` | run-to-completion | `open → start → run`, no ISS introspection |
| tutorial 17 `solution/launcher.cpp` | time-stepped | coarse `step(duration)` + `io_bind` MMIO |
| `rvvi_api2gvsoc.cpp` + `gvsoc_engine_v2.cpp` (this bridge) | retire-driven | pop the personality's commit stream, clock only to drain; IRQs over `wire_bind`; reach-in for state |

The official drivers do two things better, and both are borrowable without
changing the model: they sleep until the next event via
`get_next_event_time()` instead of polling every cycle, and they use
`wire_bind()` as a first-class stimulus channel, IRQs included. They are
production-vetted and never touch ISS state.

But the bridge cannot simply become the official dpi-wrapper. Those drivers
read no architectural state (they compare nothing) and never stop at a
retire — between two `step_until(t)` calls the ISS may retire any number of
instructions, and they only see time. Step-and-compare needs exactly what
they don't provide: instruction granularity and GPR/CSR reads. GVSOC
exports no after-retire callback, no `step_until_retire()`, no state API —
no in-tree example solves this problem, and the deviation is forced rather
than chosen. Note also that the reach-in would not disappear by moving to
`step_until`: it is imposed by the absence of a state-read API, which is
orthogonal to how time is advanced.

## Known fragilities

P0 — certain breakage on submodule bumps / ISS refactors:

- reach-in via down-cast plus ISS struct access: it depends on the memory
  layout of the ISS structs. A field reorder or a type change in the model
  means wrong reads or a silent SIGSEGV. The `#error` guards in the
  wrapper (`CONFIG_GVSOC_ISS_V2`, the force-included ISA header) protect
  against missing defines, not against layout drift; the `static_assert`
  size tripwires and the runtime known-value canary at init turn the most
  likely drifts into fail-fast errors;
- ABI coupling through defines and ZFINX: the bridge translation unit must
  compile with the same defines as the ISS `.so`; there is no runtime check
  that the two layouts match;
- path coupling on `get_component("soc/core")`: a hierarchy change that
  breaks the path fails initialization with a clean diagnostic (null return
  and exceptions are both caught). The residual hazard is a path that
  resolves to a different component, since the down-cast to `Iss*`
  is unchecked.

P1 — behavioral (false mismatches / stalls):

- register comparison is currently neutralized: the SV side clears the
  GPR/FPR write-masks right before the batched compare call, so the masked
  compares skip every register and the unmasked variants are never
  invoked — register-value divergences surface only indirectly, through PC
  or CSR effects. The instruction-binary compare has no SV call-site at
  all; its "comparisons performed" counter is incremented as a proxy, so
  the end-of-test sanity checks pass either way. PC and the enabled CSR set
  are the effective detectors today;
- hand re-implemented CSR read fixups and write masks: every CSR semantics
  change in the ISS must be mirrored manually in the bridge, so drift over
  time is guaranteed;
- `skip_irq_check` re-asserted at every step, and the post-acquire reset
  forcing: both depend on ISS side effects that might change.

P2 — localized:

- trap-CSR snapshot and the 2-step realign are sensitive to the number of
  steps a trap entry takes. The synchronous-trap seam adds a consume-once
  flag (`g_sync_trap_seam`, armed in `rvviDutTrap`, consumed at the first
  `rvviRefEventStep` after it) that realigns mstatus only — forcing mepc
  there proved harmful and is deliberately avoided;
- debug entry has one known residual: a one-instruction retire misalignment
  inside the debug ROM when debug entries re-arm in rapid succession (the
  two debug×hwloop KNOWN_FAIL lanes — see Debug entry).

## Future improvements

The GVSOC-side analysis in GVSOC_ENGINE.md refines this list. One
correction worth repeating here: the GVSOC gdbserver is not a drop-in
replacement for the reach-in. The `gv::Gdbserver_core` interface exists in
the header, but the ISS implementation exposes only GPR+PC and zero CSRs,
single-register `reg_get` is unimplemented, `stepi` is asynchronous over a
socket, and the server is not even instantiated on the cv32e40p platform.
The reach-in remains the only synchronous access to PC+GPR+CSR.

| Improvement | What it buys | Cost |
|---|---|---|
| ABI layout canary, fail-fast at first dereference | turns silent SIGSEGVs from struct drift into diagnosable errors at submodule bumps | low |
| build + smoke-test gate on every submodule bump | catches drift before the merge | low |
| `get_next_event_time()` + `was_updated` for adaptive stepping | jump to the next event instead of polling up to 2000 cycles; removes the root cause of runaways on WFI/far events | medium — a mis-calibration would break retire detection, so prototype first and re-run the regression suite |
| `wire_bind()` on the IRQ lines | done — the bridge drives the `cv32e40p_irq_injector` lines over `wire_bind` (see the caveat below for how the desync risk was handled) | — |
| CSR read-fixup/write-mask as a data table | centralizes the ISS↔bridge CSR drift, today scattered inline | low |
| RVVI-TEXT trace logging | done — three modes implemented, see RVVI_TEXT_TRACING.md |
| robust shutdown on error paths | done — abnormal termination skips the graceful teardown (see Engine lifecycle); aborted runs exit in seconds with complete RVVI-TEXT files |
| shadowed RVVI `state[]`, RVVI-VVP, `step_until` time-sync, proxy socket | alignment for its own sake; no value here — skipped |

Caveat on `wire_bind` for IRQs: the reactive machinery (`settle_irq`,
`skip_irq_check`, force-resync) and the informed injection exist precisely
to handle the ±1-cycle desync between GVSOC (post-execute IRQ evaluation)
and the RTL (combinational in DECODE). Injecting through `wire_bind` alone
would have the IRQ handled at GVSOC's own timing, reintroducing the very
desync the informed injection was built to eliminate. The wires of the
`cv32e40p_irq_injector` component are therefore only the injection channel,
not the timing control: `settle_irq`, the `skip_irq_check` discipline and
the informed injection keep deciding *when* the entry is taken. Parity was
validated with `test/quick_val.sh` across the four supported configs.

## References

Code (this repository):

- `rvvi_trace2api.sv` — SV driver: consumes `rvviTrace`, calls the RVVI API
- `rvvi_api2gvsoc.cpp` — RVVI DPI front-end (RVVI-API → engine)
- `gvsoc_engine.hpp` — pure-C firewall header (the engine API)
- `gvsoc_engine_v2.cpp` — engine wrapper (iss_v2 commit stream, IRQ injector wires)
- `test/quick_val.sh` — standard validation gate (quick sweep, four configs, SEED=1)

GVSOC API and documentation:

- `gvsoc/engine/engine/include/gv/gvsoc.hpp` — the `gv::Gvsoc` API (authoritative)
- `gvsoc/core/docs/developer_manual/tutorials.rst`, tutorial 17, and its `solution/launcher.cpp`
- `gvsoc/core/docs/user_manual/remote_control.rst`, `target_control.rst`
- `gvsoc/engine/dpi-wrapper/src/dpi.cpp` — the official (time-driven) DPI wrapper
- `gvsoc/core/models/cpu/iss_v2/` — the shared iss_v2 framework
- `gvsoc/pulp/cpu/iss_v2/{include,src}/cores/cv32e40p/` — the CV32E40P personality (`Cv32e40pEvents`, `mip_view`, ...)
- `gvsoc/pulp/pulp/cpu/iss/cv32e40p_v2.py` — the personality recipe
- `gvsoc/pulp/pulp/cv32e40p_irq_injector/` — the wire-only IRQ injector component
- `gvsoc/pulp/cv32e40p-v2-standalone{,-fpu,-zfinx,-nopulp}.py` — the v2 platforms (io_v2 stack)

RVVI standard: `RVVI/RVVI-API/`, `RVVI/RVVI-TRACE/` (v1.7), `RVVI/RVVI-TEXT/`, `RVVI/RVVI-VVP/`.

Testbench (parent repo core-v-verif):

- `cv32e40p/tb/uvmt/uvmt_cv32e40p_gvsoc_wrap.sv` — GVSOC wrap (`ref_init`, WFI watchdog, FPU/ZFINX)
- `cv32e40p/tb/uvmt/uvmt_cv32e40p_iss_wrap_common.svh` — shared RVFI→RVVI wiring (GVSOC + Imperas)
- `cv32e40p/tb/uvmt/gvsoc.flist` — file list for the GVSOC path
