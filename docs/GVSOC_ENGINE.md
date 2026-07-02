# The GVSOC system — engine, `gv::Gvsoc` API, ISS, and internals

> Deep-dive document on the **GVSOC side** (complementary to [`ARCHITECTURE.md`](ARCHITECTURE.md),
> which describes *our* bridge). Covered here: how GVSOC builds and runs a system, the time
> engine + the clock engine, the `gvsoc.hpp` API and its implementation, how the ISS hooks in
> and runs, the service subsystems (trace/power/interconnect/semihosting), **why we did not use
> the gdbserver**, and what could be improved on the GVSOC side.
>
> Every claim is anchored to a `file:line` read directly from source, and the central points
> went through an adversarial cross-check. Line numbers are indicative of these states: engine
> `a8c57439`, core `d4f37777`. Where a detail is framework-level / not inspected it is marked
> **[to verify]**.

---

## 0. TL;DR

1. GVSOC is a **multi-repo discrete-event simulator**: the **engine** (`gvsoc/gvsoc-engine`,
   *upstream, not forked*) has the time engine + the control API; **core**/**pulp** (our
   forks) hold the models + the ISS. The simulated system is a **tree of C++ components**
   loaded from `.so` files (`dlopen`+`gv_new`), described by a Python config (gvrun/gapy →
   JSON + "compiled platform tree").
2. The **time engine** (`TimeEngine`) is a list of clients (`vp::Block`) ordered by timestamp;
   it executes events by advancing time (time-warp). Each **clock domain** is a `ClockEngine`
   (a client of the TimeEngine) that converts cycles↔ps (`period = 1e12/freq`) and executes
   that cycle's `ClockEvent`s — including the ISS's **`instr_event`**, a *permanent* event
   re-executed once per cycle.
3. The `gv::Gvsoc` API is a **vtable-based abstract interface**; the concrete class is
   `gv::ControllerClient` behind the `gv::Controller` singleton. `step(duration)` = posts a
   *stop event* and runs the engine up to that timestamp. In **synchronous mode** the engine
   runs inline in the caller's thread (for us, the DPI thread).
4. The **ISS** (CV32E40P, **TIMED** model) runs one instruction per `instr_event` activation:
   `exec.current_insn` is the PC, a retire = its mutation (the `handler` returns the next PC);
   the public API exposes **no** state readback, hence our reach-in.
5. **Why not the gdbserver** (correcting an earlier revision of this analysis): the
   `gv::Gdbserver_core` interface exists in the header, **but the ISS implementation is
   unsuitable** for step-and-compare (it exposes only GPR+PC, **zero CSRs**; single `reg_get`
   is UNIMPLEMENTED; `stepi` is async over an RSP socket) **and** the component is not even
   instantiated on the cv32e40p platform. The reach-in was not an avoidable shortcut: it was
   the only way to get **synchronous access to PC+GPR+CSR**.

---

## 1. Multi-repo topology + recency

Since commit `d107dd0` ("engine: moved engine out of core to a new repo") GVSOC is split:

| submodule | SHA | nature | contents |
|---|---|---|---|
| `engine` | `a8c57439` | **upstream `gvsoc/gvsoc-engine`, NOT forked** | TimeEngine, ClockEngine, `gv::Gvsoc` API, DPI/SystemC/proxy drivers, gdbserver-engine |
| `core` | `d4f37777` | fork `FondazioneChipsIT/gvsoc-core` | HW models + ISS + gdbserver-iss |
| `pulp` | `9d9835a` | fork `FondazioneChipsIT/gvsoc-pulp` | PULP targets/platforms |

The vendored engine is **~46 commits behind** upstream (§15). Our CV32E40P changes live in
`core` (ISS) and in the bridge, **not** in the engine.

---

## 2. How GVSOC builds and runs a system

### 2.1 Before the engine: gvrun / gapy (Python)

Two Python tools (`gvsoc/gvrun/`, `gvsoc/gapy/`) generate the **config tree** as **two artifacts**:
1. **The JSON** (`gvsoc_config.json`): component hierarchy, parameters (reset value/mask,
   frequencies), ports, bindings, and the `target/gvsoc` section (`include_dirs`,
   `platform_tree`). Generated, not versioned.
2. **The "compiled platform tree"** — a per-target `.so` (`libplatform_tree_cv32e40p_standalone.so`)
   exporting `vp_get_platform_tree()` → a **statically compiled** `ComponentTreeNode` tree
   (hierarchy + module names + bindings + typed config structs).

→ There are **two instantiation paths** in the C++: from the compiled tree (preferred) or from
the raw JSON (fallback). [finding: our tree uses the compiled platform tree as the primary source]

### 2.2 Build: from the config to the component tree

The `vp::Top::Top(config_path)` constructor (`top.cpp:28`): parses the JSON (`:30`), `dlopen`s the
`platform_tree` and resolves `vp_get_platform_tree` (`:39-53`), creates the global engines (`TimeEngine`,
`TraceEngine`, `PowerEngine`, `StatsEngine`, `MemCheck`, `:55-59`), and instantiates the root via
`Component::load_component("**/target", ...)` (`:61-63`).

**Plugin loading of a model** (`Component::load_component`, `component.cpp:284`): resolves the
`<inc_dir>/<module>.so` path from the `vp_component` name, performs
**`dlopen(..., RTLD_NOW|RTLD_GLOBAL|RTLD_DEEPBIND)`** (`:328`), resolves the **`gv_new`** factory
via `dlsym` (`:352`) and calls it. → **one model = one `.so` exporting
`extern "C" gv_new(ComponentConf&)`**: this is the plugin contract. `DEEPBIND` isolates each
`.so`'s symbols (relevant because a model's methods are not visible outside its `.so` —
cf. §12).

A `Component`'s constructor (`component.cpp:438`) registers with its parent and calls `create_comps()`
(`:194`), which **recurses** over children (compiled-tree or JSON) → the whole tree is instantiated
depth-first. Config-struct fields marked `Runtime` are overridden from the JSON via offsets
(`apply_runtime_overrides`, `:393-435`).

### 2.3 Lifecycle: open → build_all → bind → reset → start

| Phase | Where | What |
|---|---|---|
| `init` | `launcher.cpp:121` | `new vp::Top(...)` → the whole build of §2.2 |
| `open` → **`build_all`** | `launcher.cpp:176`, `component.cpp:84` | `bind_comps → pre_start_all → start_all → final_bind` |
| `bind` | `launcher.cpp:179` | `time_engine->bind_to_launcher(user)` (wires the `Gvsoc_user` callbacks) |
| `start` | `launcher.cpp:185-189` | `handler->start()` + **`reset_all(true); reset_all(false)`** (reset assert/deassert pulse) + `check_run()` |
| `close` | `launcher.cpp:211` | flush traces, `stop_all`, dump stats, `unbuild_all`, delete |

`build_all` (`component.cpp:84`): **`bind_comps()`** resolves the master→slave port bindings
(`master_port->bind_to_virtual(slave_port)`, `:545`); **`start_all()`** calls `start()` on every
model. `reset_all(active)` (`block.cpp:103`) recurses over children→objects→signals→registers; the
double `true`/`false` call is the hardware reset pulse. **Note**: reset is *separate* from `build_all`.

### 2.4 Memory map: how an address is routed

The `MappingTree` (`mapping_tree.cpp`) is a **BST over the `[base, base+size)` intervals**:
`get(base, size, is_write)` (`:133`) walks down from the root (`right` if `base >= entry->base`,
otherwise `left`), checks the range, and on a miss returns the `default_entry`. A core access
resolves to the target component in O(log N). [to verify: the precise runtime call site of
`MappingTree::get()` lives in the router model (a `.so`), not in the engine]

---

## 3. The `vp::` component model

- **`vp::Block`** is the schedulable base: it holds the `vp::BlockTime time` member (`block.hpp:310`)
  and a `virtual int64_t exec()` (`block.hpp:445`) — it is the unit the TimeEngine calls back.
- **`vp::Component : public vp::Block`** adds ports/interfaces/sub-blocks.
- **`vp::Top`** is the root; it exposes `build_all`/`reset_all`/`start_all`/`stop_all`/`pause_all`/`flush_all`.
- **Ports/interfaces**: components connect through typed master/slave ports. An **`IoReq`**
  travels from master to slave (`itf/io.hpp`): 4 return states — `IO_REQ_OK` (sync), `IO_REQ_INVALID`,
  `IO_REQ_DENIED`, `IO_REQ_PENDING` (deferred response via the `grant()`/`resp()` callbacks,
  `io.hpp:244-250,385-392`); latency accumulates monotonically (`set_latency` with `std::max`, `:108`).
  The bus also carries **AMO** ops (`LR/SC/SWAP/ADD/XOR/AND/OR/MIN/MAX...`, `io.hpp:41-56`), not just R/W.

---

## 4. The time engine: `TimeEngine`

`engine/engine/include/vp/time/time_engine.hpp` + `src/time/time_engine.cpp`. It is the event scheduler.

- **Queue**: not a heap, but a **linked list of clients (`vp::Block`) ordered by increasing
  `next_event_time`** (`time_engine.hpp:117-120`, `first_client` = the most imminent). `enqueue`
  inserts in order (`time_engine.cpp:176`).
- **`exec()`** (`:38`): takes `first_client`, **advances global time** to the event's timestamp
  (`this->time = next_event_time`, `:48` — time-warp, no idle ticks), calls `current->exec()`
  which returns the **delta (in ps)** to the client's next event; re-enqueues it in order.
  Terminates on an empty queue or on `stop_req` at a timestamp boundary (`:108`).
- **`run()`** (`:134`) = `exec()` + `top->pause_all()` + returns the time of the next event (or -1).
  **`pause()`** (`:225`) sets `stop_req` (lock-free).
- **Hook**: when `enqueue` changes the first client, it calls `launcher->was_updated()` (`:210-213`).

---

## 5. The clock engine and timed events

**Two-level** model: one global `TimeEngine` (ps) + N `ClockEngine`s (one per clock domain, in cycles).

### 5.1 `ClockEvent` vs `TimeEvent`

- **`vp::ClockEvent`** (`clock_event.hpp:51`): *cycle*-based callback inside a clock domain. `cycle`
  field (absolute; `-1` = "permanent"). **Two modes** (`clock_event.hpp:43-49`): *repeated-enqueue*
  (queued each time, flexible/slow) or *enable/disable* (permanent: executes **every cycle** while
  enabled, fast). The ISS uses the **permanent** one.
- **`vp::TimeEvent`** (`time_event.hpp:43`): callback at a *timestamp in ps*, clock-independent.

### 5.2 `ClockEngine` is a client of the `TimeEngine`

`ClockEngine : public vp::Component` (`clock_engine.hpp:43`), registered as a time client
(`clock_engine.cpp:552`). `Block::exec()` is virtual and **overridden** by `ClockEngine::exec()`
(`clock_engine.cpp:385`) — this is where the polymorphic dispatch happens. `ClockEngine::exec()`:
- **permanent** branch: for each cycle `cycles++` (`:399`), runs all the permanent callbacks
  (`current->meth(...)`, `:408`); if it remains the only client, it advances `engine->time += period`
  and keeps going (cycle-by-cycle shortcut, `:422`);
- **returns the delta in ps**: `period` if there are permanent events (`:465`), `(cycle_diff)*period`
  for the next delayed one (`:472`), or `-1` (no events, `:478`).
- Conversion: **`period = 1e12 / frequency`** (`:185`). Multiple frequencies coexist as distinct
  TimeEngine clients on the common ps time axis.

### 5.3 The exact chain (what "makes the ISS run")

```
TimeEngine::exec()                          time_engine.cpp:54  current->exec()   [current = the core's ClockEngine, via virtual Block::exec()]
  → ClockEngine::exec()                     clock_engine.cpp:385
      → cycles++                            clock_engine.cpp:399
      → instr_event.meth(_this, &event)     clock_engine.cpp:408  [callback = Exec::exec_instr]  → executes ONE instruction
      → returns `period` (ps to the next cycle)
  → TimeEngine advances time and reschedules  time_engine.cpp:59-98
```

The ISS `instr_event` is a **permanent** `ClockEvent`: `instr_event.enable()` when not stalled,
`disable()` when stalled (`exec_inorder_implem.hpp:181,198`). It is **re-executed once per cycle
by the ClockEngine** — *not* manually re-enqueued each cycle.

---

## 6. The public `gvsoc.hpp` API (reference)

`engine/engine/include/gv/gvsoc.hpp` (897 lines). **Purely virtual abstract** interface: the
implementation lives in `libpulpvp.so`; the header is a vtable shared cross-`.so`.

### `class Gvsoc` (`:614`) — control

| Method | Line | What it does | Use |
|---|---|---|---|
| `open`/`start`/`close` | :630/:657/:648 | lifecycle (main controller only) | lifecycle |
| `bind(Gvsoc_user*)` | :638 | registers the event callbacks | wiring |
| `run()` / `stop()` | :673/:685 | runnable / stop at the current timestamp | free-running |
| `step(dur,wait,data)` / `step_until(ts,...)` | :742/:761 | run+stop after a duration / until a timestamp | **lockstep** |
| `update(ts)` / `wait_runnable()` | :720/:817 | sync with external time | time-sync |
| `get_time()` / `get_next_event_time()` | :826/:837 | current time / next event | introspection |
| `get_component(path)`→**`void*`** | :794 | component descriptor (or NULL) | direct-access |
| `lock`/`unlock`, `flush`, `terminate`, `quit`, `join` | :803/:782/:695/:708/:772 | — | lifecycle/misc |

> **Correctness note**: `get_component` returns **`void*`** (internally a `vp::Block*`, not a
> `vp::Component*`); the cast to the concrete type is the caller's responsibility.

### `class Gvsoc_user` (`:560`) — callbacks

`has_ended(status)` (:571), `has_stopped()` (:579), `was_updated()` (:587), `handle_step_end(data)`
(:597, async step), `handle_syscall_stop()` (:606). **Rule: no GVSOC API from inside a callback.**

### Bindings + `GvsocConf` + `gvsoc_new`

`Io`/`Wire`/`Vcd`/`Power` (+ `_user`/`_binding`): interaction channels (memory, raw signals
including IRQs, trace, power). `GvsocConf` (:848): `config_path`, `proxy_socket`, `api_mode`.
`gvsoc_new` (:895): the first client is the **main controller** (the only one allowed to
`open`/`close`).

> **What is NOT there**: no architectural state readback (PC/GPR/CSR/FPR) — grep `get_reg`/`get_csr`/
> `get_pc` = 0 matches. This is the gap our reach-in stems from.

---

## 7. How the engine implements the API

`gvsoc_new` (`launcher.cpp:649`) returns a **`gv::ControllerClient`** (this is the user's `gv::Gvsoc`;
`controller.hpp:202`) or a `Gvsoc_proxy_client` (if `proxy_socket != -1`). Behind it sits the
**`gv::Controller`** singleton (`Controller::get()`, `controller.hpp:56-59`) owning `vp::Top` +
`TimeEngine` + the client list + `run_count`/`lock_count`. (A global `gv::controller` also exists
but is **unused**.)

`step_sync(duration)` → `step_until_sync(now+duration)` (`launcher.cpp:371`): enqueues
`client->step_event` (a pre-allocated `vp::TimeEvent` whose callback is `time_engine->pause()`) at
the target timestamp, then loops `run_sync()` until `get_time() >= end_time` (`:407-426`).

**Threading** (`open`, `launcher.cpp:151`): `is_async` from `api_mode` (`:114`). Async → dedicated
`engine_routine` thread (`while(1) run_sync()`). **Sync → no engine thread**: it runs inline in the
caller's thread. **We use `Api_mode_sync`** → the engine runs in the thread of Questa's DPI call, in
lockstep (deterministic). Runnable: `run_count == clients.size() && lock_count == 0` (`:469`).

---

## 8. How the ISS hooks in and runs

### 8.1 IssWrapper as a component

`class IssWrapper : public vp::Component` (`cores/cv32e40p/class.hpp:90`) contains `Iss iss` by value
(`:100`). The `Iss` (`class.hpp:59-88`) is **not** a tree of sub-components: it is an **aggregate of
public by-value member sub-blocks**, in this order: `regfile, exec, insn_cache, timing, core,
prefetcher, decode, irq, gdbserver, lsu, dbgunit, syscalls, trace, csr, exception, memcheck`. *This*
is what makes the reach-in possible (direct public fields). The path is `soc/core`;
`get_component("soc/core")` returns it as `void*`, the bridge casts it to `IssWrapper*`. [to verify:
where `soc/core` is defined in the Python config]

### 8.2 CV32E40P configuration

`pulp_cores.py:184` — `cv32e40p` is **TIMED** (`scoreboard=True, timed=True, riscv_exceptions=True`).
Hence: real fetch through the prefetcher, modeled stalls. (`EXEC_SCOREBOARD` is **off** instead — the
class does not pass `modules`, default `[ExecInOrder()]` with `scoreboard=False`.)

### 8.3 Fast vs slow handler

Execution is driven by the `instr_event` (§5). Two callbacks:
- **FAST** `Exec::exec_instr` (`exec_inorder.cpp:165`);
- **SLOW** `Exec::exec_instr_check_all` (`:305`) — the default at boot (`:28`); it additionally does:
  pending cache flush, **take-exception** (`if (has_exception) current_insn = exception_pc`, `:320`),
  IRQ check, slow handler, `timing.insn_account()`, dbg step.

Switching: **SLOW→FAST** if `can_switch_to_fast_mode()` (`:330`) — blocked by an active
gdbserver/trace/HW counter (`exec_inorder_implem.hpp:158-170`). **FAST→SLOW** (`switch_to_full_mode`)
on icache flush, exception raise, etc.

### 8.4 Insn cache + prefetcher + execution

- **`get_insn(pc)`** (`insn_cache.hpp:63`): page-based fast path; **lazy decode** — a fresh slot has
  `handler = iss_decode_pc_handler`, and the *first* execution of that PC decodes and caches it (`:74-82`).
- **Prefetcher** (TIMED): `fetch(addr)` (`prefetch_single_line_implem.hpp:29`) may issue a memory
  `IoReq` and **stall** the core on an async refill (`stalled_inc`/`fetch_response`).
- **Execution**: `current_insn = insn->fast_handler(iss, insn, pc)` (`exec_inorder.cpp:233`) — the
  handler's return value **is the next PC**. A retire = a mutation of `current_insn`. `csr.instret`
  is a plain `CsrReg` that does **not auto-increment** (which is why the bridge uses the
  `current_insn` change, not `instret`, as the retire signal).

### 8.5 Internal traps and IRQs

- **2-phase traps**: `Exception::raise` (`exception.cpp:41`) computes the vector/`mcause`, saves
  `mepc`, updates `mstatus`, but does **not redirect immediately**: it sets
  `has_exception`+`exception_pc`. The *take* happens the next cycle, in the SLOW handler
  (`current_insn = exception_pc`, `exec_inorder.cpp:320`). (This is the "trap = 2 ISS steps" the
  bridge handles.)
- **Combinational DECODE-stage IRQ**: for CV32E40P there is an `irq.check()` *before fetch* both in
  FAST (`exec_inorder.cpp:171-179`) and in SLOW (`:337`), under `#ifdef CONFIG_GVSOC_ISS_CV32E40P` —
  the documented CV32E40P divergence. **`Irq::check()` runs inside the ISS run loop**, never called
  by the bridge.

---

## 9. Service subsystems

### 9.1 Trace / VCD
`engine/src/trace/`. Registration (`reg_trace`) → **asynchronous** dumping in a thread (`vcd_routine`) →
backend. **Key point**: if an external `Vcd_user` is bound (`vcd_bind`, `gvsoc.hpp:354`), GVSOC
**delegates the dump** to the consumer (`event_update_logical`) and does NOT write the file — this is
how a host captures waveforms.

### 9.2 Power
`engine/src/power/`. Hierarchical **activity-based** model: `PowerSource` (per-event quantum +
background + leakage), interpolated T/V/F characterization tables (`power_table.cpp`),
window-by-window energy integration (`power_trace.cpp`), `parent->inc_*` propagation up the tree.
Report to `power_report.csv`. [headers confirmed; activity-based semantics as declared, implementation
only partially read]

### 9.3 Stats
`engine/src/stats.cpp`. `StatScalar` (counters) + `StatBw` (derived bandwidth = bytes/duration). Behind
`#ifdef CONFIG_GVSOC_STATS_ACTIVE` — without the define they degrade to no-op stubs (zero overhead).

### 9.4 Interconnect / memory
**Router** `core/models/interco/router/router.cpp`: `handle_req` (`:240`) → stats + bandwidth limit →
`mapping_tree.get()` (`:267`) → translates the address (`addr - remove_offset + add_offset`) →
`req_forward` (`:304`). Requests straddling multiple mappings are split and reassembled (`:306-455`).
**Memory** `core/models/memory/memory.cpp`: final target, with memcheck and atomics.

### 9.5 Semihosting / exit → `has_ended` → our watchdog
Three termination triggers converge on `TimeEngine::quit(status)`:
- our **exit device** at `0x2000_0000` (`cv32e40p_exit_device.cpp` — a *model*, outside the engine):
  PASSED = magic `0x075BCD15` at offset 0x00 (**the same magic word as the UVM virtual peripheral**),
  exit code via offset 0x04;
- **HTIF** tohost (`cmd & 1` → `quit(cmd >> 1)`, `htif.cpp:104`);
- **semihosting** `0x10D` (program-driven pause).

Chain (the engine-internal part, verified): `TimeEngine::quit` (`time_engine.cpp:218`) sets
`finished=true` → `Controller::run_sync` sees `finished_get()` → `sim_finished` → `user->has_ended()`
(`launcher_client.cpp:134`) → **`BridgeUser::has_ended`** (`gvsoc_engine.cpp:118`) sets `g_finished`,
exposed to SV as `rvviRefIsFinished()` (the **WFI watchdog** that forces `$finish`, see
`tb-cv32e40p-discipline.md §4`). [the exit-device→quit *trigger* lives in a model `.so`, outside
the engine; only the `quit→has_ended` propagation is engine-internal]

---

## 10. How others drive the engine

| Driver | file | model |
|---|---|---|
| **Official DPI** | `engine/dpi-wrapper/src/dpi.cpp` | **time-driven** (`step_until` + `get_next_event_time` + `wait_runnable` + sleep); `wire_bind` for signals; **CPU state never touched** |
| **SystemC** | `engine/engine/src/main_systemc.cpp` | same pattern in an `SC_THREAD` |
| **Proxy (remote)** | `engine/engine/src/proxy.cpp` | text protocol over a socket: `run`/`step <ns>`/`step_cycles <dom> <n>`/`get_component`/`get_clock_domains`/`trace`/`event` |
| **gdbserver** | `core/models/gdbserver/` + `core/.../iss/src/gdbserver.cpp` | RSP/GDB over TCP (opt-in; see §12) |

The generic drivers (DPI/SystemC) are **time-driven** and inject stimuli via `wire_bind`/`io_bind`,
**without reading architectural state**. None of them does step-and-compare. This is why our bridge
deviates (retire-based + reach-in) — see [`ARCHITECTURE.md`](ARCHITECTURE.md) §11.

---

## 11. How WE implement `gvsoc.hpp` (summary)

Details in [`ARCHITECTURE.md`](ARCHITECTURE.md). In short: `Api_mode_sync`; `gvsoc_new → bind(BridgeUser)
→ open → start → get_component("soc/core") → cast IssWrapper*`; a `step(g_clock_ps=20000ps)` loop with
PC-change retire detection; reach-in on `iss.regfile/csr/exec/irq`; **never** `step_until`/
`get_next_event_time`/`wire_bind`. The workarounds (CSR read fixups, write masks, force-reset,
`skip_irq_check` re-assert, runaway detector, IRQ via direct `mip`) all descend from the absence of a
state API (§6) and the non-linkability of the ISS methods (§12).

---

## 12. Why we did not use the gdbserver

> A fair question, since GVSOC *does* ship a gdbserver with `reg_get`/`reg_set`/`stepi`. The honest
> answer is **multi-factor**, and corrects the enthusiasm of an earlier revision of this analysis:
> the gdbserver **was not a viable route**, on three independent fronts.

**(a) The reach-in was not chosen *against* the gdbserver.** The bridge's design comment
(`gvsoc_engine.cpp:35-39`) is explicit: state is read only through *public struct fields*, never
through *ISS methods* (`get_csr`, `access`, …), because those methods are compiled into the model's
`.so` and are **not linkable at DPI time** (`libgvsoc_rvvi.so` is loaded via DPI; `RTLD_DEEPBIND`
isolates the model's symbols). Root cause = a linking barrier, not an anti-gdb decision. The bridge
was born in a single commit (`42d2781`) with the reach-in from the start; **zero** mentions of gdb in
its code.

**(b) The gdbserver is not there on the cv32e40p platform.** It is an opt-in `vp::Component` to be
instantiated on the systree (explicitly created only on pulp_open/siracusa/cheshire/magia).
`cv32e40p-standalone.py` does not create it (0 occurrences) and neither does `pulp_cores.py`.
Per-target auto-instantiation is in the ~46 upstream commits we are missing. → at the time (and now)
**there was no `Gdbserver_core` on `soc/core` to talk to**.

**(c) Even if it were there, the interface is structurally unsuitable for step-and-compare** — this
is the new finding (verified by reading the ISS implementation, `core/models/cpu/iss/src/gdbserver.cpp`):
- **`gdbserver_regs_get`** exposes **33 registers = 32 GPRs + `reg[32]`=`current_insn` (PC), and ZERO
  CSRs** (`:135-167`). RVVI step-and-compare compares ~19 CSRs (mstatus/mtvec/mcause/mepc/…): the
  interface does not provide them.
- **`gdbserver_reg_get`** (single) is **UNIMPLEMENTED** (prints and returns 0, `:127-131`); `reg_set`
  only handles the PC.
- **`gdbserver_stepi`** (`:203-218`) does **not execute** the instruction: it raises
  `step_mode`/releases the halt and returns; the advance happens in the event-driven loop and
  completion is notified **over the RSP socket** (`signal` → `Rsp::signal_from_core`) — an
  **asynchronous** handshake, not a synchronous in-process `step()`.
- The API speaks **GDB register indices** (CSRs would require a qXfer target-description XML that is
  not wired up).

**Verdict.** The reach-in into public fields directly provides **PC + GPR + CSR with synchronous
access**, which is exactly what step-and-compare requires. Adopting the gdbserver would have required:
instantiating the component, bypassing the RSP, **extending the interface with CSR access and a
synchronous step**, and handling the async signal — a rewrite, not a reuse. "It was not available"
**and** "it would not have been the right tool" are both true and independent.

---

## 13. What we could use better / improvements

> **Correction to an earlier revision of this document**: the gdbserver had been presented as a
> near "drop-in" solution for state readback + per-instruction stepping. Verification (§12c) shows
> the ISS implementation is unsuitable. The actually actionable levers are therefore different.

| # | Lever | What it buys | Status / feasibility |
|---|---|---|---|
| **L1** | **`get_next_event_time()` for adaptive stepping** (used by dpi.cpp/systemc) | kills the blind 2000-cycle poll and the root cause of the runaway on WFI/far events | **ADOPTABLE TODAY** — the API is there. Risk: calibration vs retire-detect → prototype + FAST2 regression |
| **L2** | **`wire_bind()` for IRQ/debug** (the official dpi.cpp does this) | would replace the direct `mip` write + the `settle`, and would make haltreq complete | **ADOPTABLE** but with the caveat ↓ |
| **L3** | **`gv_api_version()` + engine bump** | fail-fast version check instead of a SIGSEGV from a stale `.so` | **EXISTS UPSTREAM**, we lack it (§15) — adoptable with the bump |
| **L4** | **Layout canary / fail-fast** on the reach-in | silent SIGSEGV → diagnosable error at submodule bump | adoptable today (our side) |
| **L5** | **(upstream) extend the ISS gdbserver** with CSRs + `reg_get` + a synchronous step | would *then* make the gdbserver a valid route (removing the reach-in + the ABI fragility) | **high cost, upstream** — not a reuse, see §12c |

**Caveat L2 (`wire_bind` IRQ)**: architecturally right (it is what the official dpi-wrapper does)
**but** it could **reintroduce** the ±1-cycle desync that the informed-injection was built to
eliminate (the IRQ would be handled at GVSOC's timing), and it does **not** solve the
CV32E40P-specific local-IRQ priority. → a spike, not a direct refactor.

**If only ONE thing**: **L1** (`get_next_event_time`) — adoptable without depending on upstream and
with the best immediate return (kills the runaway).

---

## 14. Other findings of interest

- **Asynchronous `io_req`** (`itf/io.hpp`): the bus supports `IO_REQ_PENDING` (deferred response via
  callback) and cumulative latency. Our exit device always returns a synchronous `IO_REQ_OK` —
  anyone extending toward devices with back-pressure must handle PENDING.
- **AMO on the bus**: the `IoReq` carries atomic opcodes (`LR/SC/SWAP/ADD/…`, `io.hpp:41-56`);
  `testandset.cpp` emulates test-and-set at the interconnect level (read → forward → write `-1`).
- **`bus_watchpoint.cpp` is also a RISC-V syscall handler** (semihosting host I/O: `getcwd/fcntl/mkdirat/
  unlinkat/…`) — this explains how bare-metal programs do host I/O without a real UART.
- **Proxy text protocol** (`proxy.cpp`): a second step-by-step control channel besides DPI
  (per-domain `step_cycles`, `get_component`, `trace`/`event`). Not active on our target, but it exists.
- **Shared magic word**: the exit device's PASSED word `0x075BCD15` is the same one used by the UVM
  virtual peripheral — it is the RTL/UVM ↔ standalone-GVSOC coupling point.
- **Dead code**: the legacy C function `handle_syscall(Iss*, …)` calls `iss_exit()` which **is not
  defined anywhere in the tree** → not compiled in the active flow (the live path is the
  `Syscalls::` class + HTIF).

**Not present / not verified**: checkpoint/restart is not in the vendored engine (the upstream
`restart()` is in the missing commits); `iss/` vs `iss_v2/` — our path references
`cpu/iss/src/cv32e40p/` (the "v1" ISS) but the accuracy difference has not been proven [to verify];
the exact `prefetcher_size` for the `cv32e40p` class and where the branch penalty lives [to verify].

---

## 15. Recency & maintenance

The vendored engine (`a8c57439`, upstream not forked) is **~46 commits behind** `gvsoc/gvsoc-engine`
main. Among the missing ones: `gv_api_version()` (`99a3ffc7` — lever L3), per-target gdbserver
auto-instantiation (`b93bf7b5`), in-process `restart()`, exact `step_cycles`, SystemC launcher split.
API delta: we lack `Gvsoc::restart()`, `GvsocConf::proxy_enabled`, `gv_api_version()`. **Bump risk**:
the auto-gdbserver changes the component graph → validate that it neither moves `soc/core` nor breaks
the reach-in (rule `cross-target-iss-validation.md` + `/verify-gvsoc`).

> Honesty note: the exact content of `b93bf7b5` is not readable in this tree (missing commit); the
> **absence** of auto-instantiation, however, is verified directly on our code. The "46 commits"
> figure is today's snapshot via `git fetch`.

---

## 16. References

**Engine** (`gvsoc/engine/engine/`): `include/gv/gvsoc.hpp` · `src/launcher.cpp` + `include/vp/controller.hpp`
· `time/time_engine.{hpp,cpp}` · `clock/{clock_engine,clock_event,block_clock}.{hpp,cpp}` ·
`src/{top,component,block,mapping_tree}.cpp` · `src/{trace,power}/*` · `src/stats.cpp` ·
`include/vp/gdbserver/gdbserver_engine.hpp` · `dpi-wrapper/src/dpi.cpp` · `src/proxy.cpp`

**Core / ISS** (`gvsoc/core/models/`): `cpu/iss/include/cores/cv32e40p/class.hpp` ·
`cpu/iss/{src/exec/exec_inorder.cpp,include/exec/exec_inorder*.hpp}` · `cpu/iss/include/insn_cache.hpp` ·
`cpu/iss/include/prefetch/*` · `cpu/iss/src/{exception,syscalls,htif,gdbserver}.cpp` · `interco/router/router.cpp`
· `interco/{bus_watchpoint,testandset}.cpp` · `memory/memory.cpp` · `gdbserver/{gdbserver,rsp}.cpp`

**Platform** (`gvsoc/pulp/`): `cv32e40p-standalone.py` · `pulp/cpu/iss/pulp_cores.py` ·
`pulp/cv32e40p_exit/cv32e40p_exit_device.cpp`

**Our side**: `gvsoc_engine.{cpp,hpp}` · see [`ARCHITECTURE.md`](ARCHITECTURE.md)

**External**: [GVSoC paper — arXiv:2201.08166](https://arxiv.org/pdf/2201.08166) ·
[gvsoc/gvsoc (GitHub)](https://github.com/gvsoc/gvsoc) · [user docs](https://gvsoc.readthedocs.io/) ·
[developer docs](https://gvsoc-developer.readthedocs.io/) ·
[GAP SDK GDB Server](https://greenwaves-technologies.com/manuals_gap9/gap9_sdk_doc/html/source/tools/docs/gvsoc/gdb_server.html)
