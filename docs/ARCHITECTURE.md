# Architecture — GVSOC engine + RVVI bridge

> How `gvsoc_rvvi` drives the embedded GVSOC engine for CV32E40P RVVI
> step-and-compare co-simulation, **why** it does it this way, where the GVSOC
> documentation prescribes it, and how we differ from the official examples.
>
> Code references are indicative of submodule state `37f710e`; prefer
> symbol/function names (stable) over line numbers (volatile).

---

## 0. TL;DR

1. The engine **lifecycle** (`gvsoc_new → open → start → … → quit → join → close`,
   **synchronous** mode) **conforms** to the official GVSOC pattern (tutorial 17).
2. The **step model is different, and has to be**: we are **retire-based
   step-and-compare** (RVVI), the official examples are **time-driven**. The
   difference is *forced* by the RVVI(retire granularity) ↔ GVSOC(time/event
   granularity) mismatch — it is not a bug.
3. The real source of fragility is **not** the step model: it is the **direct
   reach-in into ISS state** (`g_wrapper->iss.*`). It is **unavoidable** — the
   public GVSOC API exposes *no* architectural state readback (PC/GPR/CSR) at all —
   but it couples the bridge to the model's layout/ABI (the BUILD-1..5 family).
4. The **custom** RVVI functions (`rvviRefRetireAndCompare`, `rvviRefInjectIrq`,
   `rvviRefSetInformedIrq`, `rvviRefIsFinished`, the `rvviBridgeSet*` setters for
   RVVI-TEXT emission, the `rvviText*` family of the RTL-only tracer) are
   **necessary extensions** (DPI performance, OVPSim-style IRQ parity, RVVI-TEXT
   tracing), not removable shortcuts — details in §10.
5. Concrete improvement levers, adoptable **without** abandoning the retire-based model:
   `get_next_event_time()` (adaptive anti-runaway stepping), `wire_bind()` for IRQs
   (to be validated, see caveat §13). RVVI-TEXT is now **implemented and validated**
   in 3 modes (RTL-only, bridge-emit, dual-trace) — see `RVVI_TEXT_TRACING.md`.

---

## 1. The problem — why a dedicated bridge is needed

The goal is an **instruction-accurate step-and-compare** co-simulation: for each
instruction retired by the RTL DUT (CV32E40P), the reference model (GVSOC ISS) is
advanced by **one instruction** and the architectural state (PC, GPR, FPR, CSR) is
compared. This is exactly what the **RVVI** standard (RISC-V Verification
Interface) requires: a comparison **at every retire**.

GVSOC, however, is an **event-driven virtual platform** designed for *modeling* and
for **time-domain** co-simulation (synchronizing simulated time with an external
simulator), not for architectural step-and-compare. This produces two mismatches
that the bridge must close:

| RVVI wants… | GVSOC offers (public API)… | Consequence |
|---|---|---|
| Stopping at the **instruction** boundary (retire) | Only **time** stepping (`step(ps)` / `step_until(t)`) | We detect the retire from a PC change |
| Reading the model's **PC/GPR/CSR/FPR** | **No** state readback (only Io/Wire/Vcd/Power bindings) | Direct reach-in into the ISS structs |

The bridge is the translation between these two worlds.

---

## 2. 2-layer topology — the "compilation firewall"

```
SV testbench  --DPI-->  rvvi_api2gvsoc.cpp  --pure-C ABI-->  gvsoc_engine.cpp  --C++ API-->  gv::Gvsoc + IssWrapper
 (rvviTrace)   (~60 fn)   [RVVI front-end]   (gvsoc_engine.hpp)  [engine wrapper]  (heavy ISS/GVSOC headers)
```

- **`rvvi_api2gvsoc.cpp`** — RVVI front-end. Implements ~60 `extern "C"` `rvvi*`
  functions. Includes **only** `gvsoc_engine.hpp` (lightweight pure-C header) +
  `svdpi.h`/`vpi`. Knows RVVI and DPI, does **not** know GVSOC.
- **`gvsoc_engine.cpp`** — engine wrapper. Includes the heavy GVSOC/ISS headers.
  Knows GVSOC, does **not** know DPI.
- **`gvsoc_engine.hpp`** — the pure-C `extern "C"` interface between the two. It is
  the *firewall*: the two header sets (Questa's `svdpi.h` ↔ GVSOC/ISS) **never meet
  in the same translation unit**.

Why it is needed: Questa's `svdpi.h` and the ISS model headers have colliding
macros/types; keeping them apart eliminates an entire class of build errors. It is
sound engineering, and it is the backbone of the design.

---

## 3. The official GVSOC external-control pattern — where the docs prescribe it

GVSOC explicitly documents control "from an external simulator":

- **Tutorial 17** — `gvsoc/core/docs/developer_manual/tutorials/17_how_to_control_gvsoc_from_an_external_simulator/`
  (prose in `…/docs/developer_manual/tutorials.rst`, canonical solution in
  `solution/launcher.cpp`).
- **Authoritative API** — `gvsoc/engine/engine/include/gv/gvsoc.hpp`.
- **User manual** — `user_manual/remote_control.rst` (out-of-process control via
  the Python proxy) and `target_control.rst`.

The tutorial prescribes (in-process, **synchronous mode**):

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

Two official rules relevant to us:
- **Synchronous mode** (`Api_mode_sync`): the engine runs in the caller's thread,
  every command executes inline. It is the documented choice for **deterministic
  control** (`gvsoc.hpp` doc of `Api_mode`). The header's default is `async` instead.
- **"No GVSOC API can be called from this callback"** — repeated on all
  `Io_user`/`Vcd_user` callbacks: the engine cannot be re-entered from inside a handler.

There are two control planes: **in-process C++** (tutorial 17 — the one we use)
and **remote/proxy** (`GvsocConf::proxy_socket` + `vp/proxy.hpp`, socket-based
control from a separate process). For DPI lockstep the **in-process** one is the
right choice (zero socket latency).

---

## 4. Engine lifecycle — what we use of `gv::Gvsoc`

Single entry point: `gvsoc_engine_init()`, called from `rvviRefInit()`. Three phases:

1. **create** — `g_conf.api_mode = Api_mode_sync`; `gv::gvsoc_new(&g_conf)`;
   `g_gvsoc->bind(&g_user)` (registers `BridgeUser`, a subclass of `gv::Gvsoc_user`).
2. **open & start** — `g_gvsoc->open()` (elaborates the component graph) →
   `g_gvsoc->start()`.
3. **acquire core** — `comp = g_gvsoc->get_component("soc/core")` →
   `g_wrapper = static_cast<IssWrapper*>(comp)`; `build_csr_map(...)`; CSR reset
   forcing (see §6).

Shutdown (`gvsoc_engine_shutdown` from `rvviRefShutdown`), each call wrapped in
try/catch (DPI does not propagate exceptions): `stop() → quit(0) → join() → close()`.

`BridgeUser` overrides **only** two callbacks: `has_ended(status)` (the exit device
ended the sim → sets `g_finished`) and `has_stopped()` (no-op: we drive stepping by
hand). All the other callbacks (`handle_step_end`, `was_updated`,
`handle_syscall_stop`) are **not** needed in synchronous mode.

**Mapping onto the `gv::Gvsoc` API** (what we use / what we don't):

| API | Use | Note |
|---|---|---|
| `gvsoc_new`, `bind`, `open`, `start`, `stop`, `quit`, `join`, `close` | ✅ used | lifecycle conforms to the tutorial |
| `step(duration_ps)` | ⚠️ used "at the edge" | not to "advance by N ps" but as the primitive of a retire poll-loop (§5) |
| `get_component(path)` | ⚠️ used for reach-in | the only channel to reach ISS state (§6) |
| free-running `run`/`stop`, async `step`+`handle_step_end` | ❌ not used | rightly so: incompatible with retire-by-retire determinism |
| `get_time`, `get_next_event_time`, `update`, `wait_runnable` | ❌ not used | we do not synchronize on time; `get_next_event_time` is a lever though (§13) |
| `wire_bind`, `io_bind` | ❌ not used | `io_bind`=inbound MMIO (N/A); `wire_bind`=IRQ (lever §13) |
| `lock`/`unlock`, `flush`, `terminate` | ❌ not used | `lock/unlock` "high cost"; `terminate` is a robustness lever (§13) |
| `Vcd`, `Power`, `Testbench`, proxy | ❌ not used | out of scope for RVVI co-sim |

---

## 5. Step semantics — retire-based on top of a time-based API

`rvviRefEventStep` → `gvsoc_engine_step()`. The model:

```
loop up to STEP_MAX_CYCLES (=2000):
    pre_pc = iss.exec.current_insn          // current_insn IS the PC, not the opcode
    g_gvsoc->step(g_clock_ps)               // advance ONE clock (g_clock_ps = 20000 ps = 50 MHz)
    post_pc = iss.exec.current_insn
    if post_pc != pre_pc:  return RETIRE     // the PC change is the retire signal
```

Why the PC change and not `instret`: in GVSOC `csr.instret` does not auto-increment,
so it is not a reliable retire counter. The `current_insn` transition is the retire
signal available.

Handled corner cases:
- **Branch-to-self** (`beq x,x,0`): `post_pc == pre_pc` would look like a stall →
  recognized via the `stall_cycles 0→>0` transition (branch penalty) at constant
  PC, and counted as a retire.
- **Timeout** (`STEP_MAX_CYCLES` cycles without a retire): returns "no retire" + diagnostics.
- **Runaway detector**: counts consecutive timeouts with unchanged, non-WFI PC; at
  `RUNAWAY_THRESHOLD` (=16) it latches `g_runaway` (sticky, cleared only by init). A clean
  retire resets the counter; WFI is excluded. Exposed as bit `0x20` in the batch call.

This loop is **outside the intended contract** of `step()` (meant as "advance by N
ps", not "advance until retire"). It is correct and loses no retires, but it is
CPU-intensive (per-cycle polling) and needs the runaway detector as a safety net.
Lever §13 (`get_next_event_time`) aims precisely at making it adaptive.

---

## 6. ISS state readback — the reach-in (and why it is unavoidable)

The `gv::Gvsoc` API has **no** method to read PC/GPR/CSR/FPR: it only exposes **Io**
(memory), **Wire** (signals), **Vcd** (trace), **Power** bindings. Architectural
step-and-compare needs the state, so the only way is the reach-in:

`get_component("soc/core")` → down-cast to `IssWrapper*` → access to the **public
struct members** of the ISS.

| State | Symbol | Note |
|---|---|---|
| PC | `iss.exec.current_insn` | it is the current PC |
| GPR | `iss.regfile.regs[i]` | x0 always 0 |
| FPR | `iss.regfile.fregs[i]` / `regs[i]` under ZFINX | `#ifdef ISS_SINGLE_REGFILE` |
| CSR | `g_csr_value_map[addr]` → `&csr.<reg>.value` | map built by `build_csr_map` (~110 CSRs) |
| mip | `iss.csr.mip.value` | |
| WFI / stall | `iss.exec.wfi`, `iss.exec.stall_cycles` | |

**Explicit contract**: only the public data members are accessed, **never** the ISS
methods (`get_csr()`, `access()`, …). Those methods are compiled into the model
`.so` files, dynamically loaded by GVSOC, and are **not linkable at DPI time** when
the simulator loads `libgvsoc_rvvi.so`. This is the constraint that cascades into
everything else:

- **Re-implemented read fixups**: reading `.value` directly bypasses the ISS CSR
  access callbacks, so the read fixups must be reproduced by hand in the getter
  (e.g. mstatus: rebuilding the SD bit from FS/XS, forcing MPP=M; mtval/tdata2
  forced to 0 because they are read-only in the RTL).
- **Re-implemented write masks**: `gvsoc_engine_set_csr` applies the CV32E40P write
  mask by hand (e.g. mstatus `0x7888`).
- **Post-acquire reset forcing**: some CSRs (depc, mepc, mcause, …) are not brought
  back to the RTL reset values by `Csr::reset()` → the wrapper forces them after the
  acquire (defense-in-depth against an ISS "reset-scope hole").

Consequence: the reach-in is **not a convenience hack** — it is imposed by the
absence of an introspection API. It is, however, the source of the ABI
fragility (§12).

---

## 7. RVVI-API → engine mapping (excerpt)

| RVVI function | Engine action |
|---|---|
| `rvviRefInit` | env + `init_net_map` + `create_temp_config` (injects the ELF into the JSON) + `gvsoc_engine_init` |
| `rvviRefShutdown` | `gvsoc_engine_shutdown` + performance/profile dump |
| `rvviRefProgramLoad` | **stub** (the ELF is already in the config) |
| `rvviRefNetIndexGet` / `rvviRefNetSet` | name→index lookup; `gvsoc_engine_set_irq` + `settle_irq` |
| `rvviRefEventStep` | `gvsoc_engine_step` + resync/realign logic |
| `rvviRefPcGet`/`GprGet`/`FprGet`/`CsrGet` | readback from the ISS (§6) |
| `rvviRefPcCompare`/`GprsCompare`/`CsrsCompare`/`FprsCompare` | DUT state vs ISS readback |
| `rvviRefCsrSetVolatile`/`Mask` | configure the compare (CSRs the ISS does not model cycle-by-cycle) |
| `rvviRefGprSet`/`FprSet`/`CsrSet` | **direct injection into the ISS** (used in `ref_init`, e.g. mtvec) |
| `rvviRefRetireAndCompare` | **GVSOC-only batch** (§10) |
| `rvviRefInjectIrq` / `rvviRefSetInformedIrq` | informed-injection (§8) |
| ~25 vector/memory/conn functions | **stub** (N/A for CV32E40P) |

> State crosses the DPI boundary **not** as a packed struct, but through scalar
> function signatures + the SV `rvviTrace` interface read field-by-field by the
> `rvvi_trace2api.sv` driver.

---

## 8. IRQ injection — two mechanisms

CV32E40P models interrupts combinationally in the DECODE FSM; GVSOC evaluates them
post-execute → intrinsic ~1-cycle desync on entry. The bridge offers two paths.

**(A) Reactive (default).** `rvviRefNetSet` → `gvsoc_engine_set_irq`: maps the RVVI
net index onto the `mip` bit (net0→bit3 MSW, net1→bit7 MTimer, net2→bit11 MExt,
net3..18→bit16..31 local) and writes `iss.csr.mip.value` directly. It cannot call
`irq.check_interrupts()` (not linkable) — the ISS sees `mip` in its natural loop.
`settle_irq()` runs a few drain cycles so the pre-fetch guard fires in the same
cycle as the RTL. Since `skip_irq_check` is asserted (the ISS does not take IRQs on
its own), the **force-resync** path in `rvviRefEventStep` realigns PC + CSR + GPR/FPR
from the DUT when the DUT enters an async trap (RVFI does not set `rvfi_trap` for
async traps).

**(B) Informed — OVPSim "deferint" style** (opt-in, `+rvvi_informed_irq` plusarg).
`rvviRefInjectIrq` detects the vectored entry (`DUT-PC == mtvec_base + cause*4`, with
ISS-MIE still 1) and calls `gvsoc_engine_take_irq_for_one_step(cause)`: masks `mip`
**down to the single cause** (CV32E40P ranks the fast local IRQs above MEI, whereas
the generic `Irq::check()` would use the standard RISC-V priority), lowers
`skip_irq_check` for **one** step, and lets **the ISS compute the entry** (mepc,
mstatus.MIE→MPIE, mcause=(1<<31)|id, current_insn=vector). Then it restores `mip`
and the flags. The DUT is the oracle for *which/when*; the ISS is the calculator; the
bridge compares in the normal step-and-compare.

> A **trap = 2 ISS steps**: GVSOC models the entry as an extra step → on the retire of
> a trap the bridge issues ONE extra silent `rvviRefEventStep` and **skips** the compare.
> This is intentional and must not be "fixed". Same spirit: pipeline-flush retires with
> PC=0 are skipped (they are not divergences).

---

## 9. Debug / haltreq — the DPI linkability limit

`haltreq` is net index 19. In `gvsoc_engine_set_irq` it lands on a special branch that
writes `iss.irq.req_debug = true` and sets `dcsr.cause`. But the ISS `debug_req()`
method — the one that performs the real side effects (WFI exit, switch to full-exec) —
is **not linkable at DPI time**, so it is never invoked. Debug single-stepping
therefore remains **partial** (state set, but not the debug-entry state machine). It
is the same limit as `irq.check_interrupts()`: the bridge can write public structs,
never invoke ISS logic.

---

## 10. RVVI standard conformance

| RVVI component | Status | Assessment |
|---|---|---|
| **RVVI-API** (init/step/compare/csr/gpr/fpr) | ~60 fn implemented, ~25 vector/mem stubs | ✅ good alignment on the core; honest stubs where not needed |
| **RVVI-TRACE** (the `rvviTrace` interface consumed by `rvvi_trace2api.sv`) | consumed correctly on the SV side | ✅ see the separate TRACE conformance report |
| **Custom functions** | see below | ⚠️ justified, to be tracked |
| **RVVI-TEXT** (textual / golden trace logging) | **implemented and validated** — 3 modes (RTL-only, bridge-emit, dual-trace) from a single formatter `rvvi_text_writer.{hpp,cpp}` | ✅ see `RVVI_TEXT_TRACING.md` |
| **RVVI-VVP** | not used | N/A for our case |

**The custom functions — not in the (vendored) RVVI standard, but justified:**

| Function | Rationale | Verdict |
|---|---|---|
| `rvviRefRetireAndCompare` | **1 DPI crossing** instead of 5 (`EventStep`+`PcCompare`+`GprsCompareWritten`+`CsrsCompare`+`FprsCompare`) + runaway bit. Pure performance | **keep** (documented as a local optimization) |
| `rvviRefInjectIrq` + `rvviRefSetInformedIrq` | IRQ parity with OVPSim: without them, false mismatches on every async IRQ (±1 desync, RVFI does not mark rvfi_trap for async) | **keep** (closes a real architectural limit) |
| `rvviRefIsFinished` | end-of-simulation polling from the `gvsoc_wrap` watchdog (the RVVI API does not expose the "ISS finished" state) | **keep** |
| `rvviBridgeSetMode` / `rvviBridgeSetRefOnly` / `rvviBridgeSetFlen` | RVVI-TEXT emission: per-retire MODE on the ref side, ref-only switch in dual-trace mode, FLEN from the CFG for the `PARAMS` header. The `rvviBridge*` prefix avoids collisions with the vendor `rvviDut*`/`rvviRef*` namespaces | **keep** |
| `rvviText*` family (`rvviTextOpen`/`SetGpr`/`SetFpr`/`SetCsr`/`SetMode`/`Write`/`Close`) | DPI shim for the SV tracer (RTL-only and dual-trace modes) on `librvvi_text.so`, zero GVSOC dependency | **keep** |

The "reduce the custom functions" direction is only partly feasible: the batch call
could in theory disappear by having the TB call the 5 standard functions (losing the
performance win); the informed-IRQ functions **cannot** go away until RVVI
standardizes a deferint mechanism.

---

## 11. Comparison with the official dpi-wrapper and the other in-tree examples

GVSOC ships **its own** DPI wrapper (`gvsoc/engine/dpi-wrapper/src/dpi.cpp`) and
several launchers. They all belong to a **time-driven** family; we belong to a
**retire-driven** one. The two solve different problems.

| In-tree example | Sync model | Technique |
|---|---|---|
| `dpi-wrapper/src/dpi.cpp` (official DPI) | time-driven | `step_until(SV_time)` + `get_next_event_time()` + sleep until the next event; ISS = black box; stimulus via `wire_bind`/`io_bind` |
| `engine/src/main_systemc.cpp` | time-driven (SystemC) | `SC_THREAD`: `step_until(sc_time)` + `get_next_event_time()` + `wait()`; `was_updated`/`has_ended` callbacks |
| `engine/src/launcher.cpp`, `main.cpp` | run-to-completion / multi-client state machine | `open→start→run` (event-driven), no ISS introspection |
| `tutorial 17 solution/launcher.cpp` | time-stepped | coarse-grained `step(duration)` + `io_bind` MMIO |
| **`rvvi_api2gvsoc.cpp` + `gvsoc_engine.cpp` (us)** | **retire-driven** | poll `step(clock_ps)` until a PC change + reach-in into the ISS structs |

**What the official drivers do better** (and what we can borrow *without* changing
model): they use `get_next_event_time()` to **sleep until the next event** instead of
polling every cycle, and `wire_bind()` as a first-class channel for stimulus (IRQs
included). They are production-vetted and never touch ISS state.

**Why we cannot simply "become" the official dpi-wrapper:** they do NOT read
architectural state (they do no compare) and they do NOT stop at retires — between
two `step_until(t)` calls the ISS may retire N instructions and they only see time.
RVVI step-and-compare needs exactly what they don't: instruction granularity +
GPR/CSR reads. GVSOC exports neither an "after-retire" callback nor a
`step_until_retire()` nor a state API — so **no in-tree example solves our problem**,
and our deviation is forced, not an avoidable choice.

> Honesty note: the reach-in would **not** disappear by moving to `step_until` (a
> common misconception). The reach-in is imposed by the absence of a state-read API,
> **orthogonal** to how time is advanced. `step_until`/`get_next_event_time` improve
> *how we advance*, not *how we observe*.

---

## 12. Known fragilities (prioritized)

**P0 — certain breakage on submodule bumps / ISS refactors**
- **Reach-in via down-cast + ISS struct access** (§6): depends on the **memory
  layout** of the ISS structs. A field reorder or a type change in the model →
  wrong reads or silent SIGSEGV. The `#error` guard on `CONFIG_GVSOC_ISS_CV32E40P`
  protects against the missing *define*, **not** against *layout drift*.
- **ABI coupling through defines / ZFINX**: the bridge TU must compile with the same
  defines as the ISS `.so`; no runtime check that the two layouts match.
- **Path coupling on `get_component("soc/core")`**: if the config hierarchy changes
  the core path → `nullptr` → crash at acquire.

**P1 — behavioral (false mismatches / stalls)**
- PC-change retire detection (branch-to-self heuristic + runaway detector): new
  "constant PC but retiring" cases could require ad-hoc patches.
- Hand re-implemented CSR read fixups and write masks: every CSR semantics change in
  the ISS must be mirrored manually in the bridge → drift guaranteed over time.
- `skip_irq_check` re-asserted at every step and post-acquire reset forcing: they
  depend on ISS side effects that might change.

**P2 — localized**
- Trap-CSR snapshot + 2-step realign: sensitive to the number of steps per trap entry.
- Partial debug entry (§9): a permanent functional limit while staying DPI.

---

## 13. Future improvements / alignment

> **Deep dive**: the GVSOC-side analysis in [`GVSOC_ENGINE.md`](GVSOC_ENGINE.md) §12-§13 refines
> this section. Correction note: the GVSOC gdbserver is **not** a drop-in replacement for the
> reach-in — the `gv::Gdbserver_core` interface exists in the header but the ISS implementation
> is unsuitable for step-and-compare (it exposes only GPR+PC, **zero CSRs**; single `reg_get` is
> UNIMPLEMENTED; `stepi` is async over a socket) and it is not even instantiated on the cv32e40p
> platform. The reach-in remains the right way to get **synchronous access to PC+GPR+CSR**.
> Real levers: `get_next_event_time()` (today, anti-runaway), `wire_bind` IRQ (desync caveat),
> engine bump for `gv_api_version()`.

| Lever | What it buys | Cost | Priority |
|---|---|---|---|
| **ABI layout canary + fail-fast** at first deref | turns silent SIGSEGVs from struct drift into diagnosable errors at submodule bump | low | **P0** |
| **`/verify-gvsoc` as a pre-merge gate** for every submodule bump | catches drift before the merge (cf. rule `cross-target-iss-validation.md`) | low | **P0** |
| **`get_next_event_time()` + `was_updated`** → adaptive stepping | in the step loop, jump to the next event instead of polling 2000 cycles → kills the root cause of the runaway on WFI/far events | medium (risk: breaking the retire-detect if mis-calibrated → prototype + mandatory FAST2 regression) | **P1** |
| **`wire_bind()` on the IRQ line** | would replace the direct `mip` write with the GVSOC channel intended for "raw signals (pads, interrupts, registers)" | medium — **to be validated, caveat** ↓ | **P1 (spike)** |
| **CSR read-fixup/write-mask as a data table** | centralizes the ISS↔bridge CSR drift, today scattered inline | low | **P1** |
| **`terminate()` before `join()`** in error paths | robust shutdown, does not hang if the sim wedges | low | **P2** |
| **RVVI-TEXT for trace logging / golden** | standard textual trace output → comparison against external goldens + interop with RVVI tools | — | **DONE** — 3 modes (RTL-only, bridge-emit, dual-trace) implemented and validated, see `RVVI_TEXT_TRACING.md` |
| shadowed RVVI `state[]`, RVVI-VVP, `step_until` time-sync, proxy socket | "purity" alignment | high / no value for us | **P3 — skip** |

> **Critical caveat on `wire_bind` for IRQs.** All the reactive logic (force-set of
> `mip` + `settle` + `skip_irq_check` + force-resync) and the informed-injection exist
> *precisely* to handle the ±1-cycle desync between GVSOC (evaluates the IRQ post-execute)
> and the RTL (combinational in DECODE). Injecting via `wire_bind` would have the IRQ
> handled at **GVSOC's timing** → it could **reintroduce** the very desync the informed-
> injection was built to eliminate. It must therefore be treated as a **spike** (prototype
> on a single line, e.g. MTimer, with parity validation against the current force-set
> and a FAST2 regression), **not** as a direct refactor. It also remains to be verified
> that the CV32E40P ISS model exposes an IRQ interface bindable by name.

---

## 14. References

**Code (`gvsoc_rvvi` submodule)**
- `rvvi_trace2api.sv` — SV driver: consumes `rvviTrace`, calls the RVVI API (RVVI-TRACE → RVVI-API)
- `rvvi_api2gvsoc.cpp` — RVVI DPI front-end (RVVI-API → engine)
- `gvsoc_engine.cpp` / `gvsoc_engine.hpp` — engine wrapper (pure-C firewall)

**GVSOC API and documentation**
- `gvsoc/engine/engine/include/gv/gvsoc.hpp` — `gv::Gvsoc` API (authoritative)
- `gvsoc/core/docs/developer_manual/tutorials.rst` §17 + `…/17_how_to_control_gvsoc_from_an_external_simulator/solution/launcher.cpp`
- `gvsoc/core/docs/user_manual/{remote_control,target_control}.rst`
- `gvsoc/engine/dpi-wrapper/src/dpi.cpp` — official DPI wrapper (time-driven model)

**RVVI standard**
- `RVVI/RVVI-API/`, `RVVI/RVVI-TRACE/` (v1.7), `RVVI/RVVI-TEXT/`, `RVVI/RVVI-VVP/`

**Testbench (parent repo core-v-verif)**
- `cv32e40p/tb/uvmt/uvmt_cv32e40p_gvsoc_wrap.sv` — GVSOC wrap (`ref_init`, WFI watchdog, FPU/ZFINX)
- `cv32e40p/tb/uvmt/uvmt_cv32e40p_iss_wrap_common.svh` — shared RVFI→RVVI wiring (GVSOC + Imperas)
- `cv32e40p/tb/uvmt/gvsoc.flist` — file list for the GVSOC path

**Discipline rules** (parent repo)
- `.claude/rules/tb-cv32e40p-discipline.md`, `gvsoc-development-workflow.md`, `cross-target-iss-validation.md`
