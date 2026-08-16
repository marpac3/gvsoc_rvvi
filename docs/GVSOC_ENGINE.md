# GVSOC internals — engine, `gv::Gvsoc` API, and the ISS

Notes on the GVSOC side of the co-simulation: how GVSOC builds and runs a
system, the time and clock engines, the `gvsoc.hpp` control API and its
implementation, how the ISS hooks in, why we did not use the gdbserver, and
what could be improved. The companion document
[`ARCHITECTURE.md`](ARCHITECTURE.md) describes our bridge; this one describes
what the bridge sits on.

Line references are against engine `a8c57439` and core `d4f37777`; prefer the
symbol names, the line numbers drift.

## Repository layout

Since commit `d107dd0` ("engine: moved engine out of core to a new repo")
GVSOC is split across three submodules:

| submodule | SHA | nature | contents |
|---|---|---|---|
| `engine` | `a8c57439` | upstream `gvsoc/gvsoc-engine`, not forked | TimeEngine, ClockEngine, `gv::Gvsoc` API, DPI/SystemC/proxy drivers, gdbserver-engine |
| `core` | `d4f37777` | fork of `FondazioneChipsIT/gvsoc-core` | hardware models, the ISS frameworks (the original `iss/` and the modular `iss_v2/` the co-sim runs on), gdbserver-iss |
| `pulp` | `9d9835a` | fork of `FondazioneChipsIT/gvsoc-pulp` | PULP targets and platforms; the out-of-tree CV32E40P iss_v2 personality (`cpu/iss_v2/`) |

Our CV32E40P changes live in `core` (the shared iss_v2 framework), in
`pulp` (the iss_v2 personality, the v2 platforms and the IRQ injector) and
in the bridge, not in the engine. The vendored engine lags upstream main by roughly 77 commits as of
July 2026 — see "Upstream drift" at the end.

## From config to component tree

The simulated system is a tree of C++ components loaded from shared objects
and described by a Python configuration. Two Python tools (`gvsoc/gvrun/`,
`gvsoc/gapy/`) generate the configuration in two artifacts:

1. `gvsoc_config.json` — component hierarchy, parameters (reset value/mask,
   frequencies), ports, bindings, plus the `target/gvsoc` section
   (`include_dirs`, `platform_tree`). Generated, not versioned.
2. The "compiled platform tree" — a per-target shared object
   (`libplatform_tree_cv32e40p_v2_standalone.so`) exporting
   `vp_get_platform_tree()`, which returns a statically compiled
   `ComponentTreeNode` tree: hierarchy, module names, bindings, typed config
   structs.

The C++ side can instantiate from either artifact; the compiled tree is the
preferred path and the one our target uses, with the raw JSON as fallback.

The `vp::Top` constructor parses the JSON, `dlopen`s the platform tree and
resolves `vp_get_platform_tree`, creates the global engines (`TimeEngine`,
`TraceEngine`, `PowerEngine`, `StatsEngine`, `MemCheck`), and instantiates the
root through `Component::load_component("**/target", ...)`.

Model loading follows a plugin contract: one model = one `.so` exporting
`extern "C" gv_new(ComponentConf&)`. `Component::load_component` resolves
`<inc_dir>/<module>.so` from the `vp_component` name, calls
`dlopen(..., RTLD_NOW | RTLD_GLOBAL | RTLD_DEEPBIND)`, and resolves the
`gv_new` factory with `dlsym`. `RTLD_DEEPBIND` isolates each model's symbols —
a model's methods are not visible outside its own `.so`, which matters later
(see "Why we did not use the gdbserver"). Each `Component` constructor
registers with its parent and calls `create_comps()`, which recurses over the
children, so the whole tree comes up depth-first. Config-struct fields marked
`Runtime` are overridden from the JSON by field offset
(`apply_runtime_overrides`).

### Lifecycle

| Phase | Where | What happens |
|---|---|---|
| `init` | `launcher.cpp` | `new vp::Top(...)` — the whole build above |
| `open` | `Component::build_all` | `bind_comps → pre_start_all → start_all → final_bind` |
| `bind` | `launcher.cpp` | `time_engine->bind_to_launcher(user)`, wiring the `Gvsoc_user` callbacks |
| `start` | `launcher.cpp` | `handler->start()`, then `reset_all(true); reset_all(false)` — the reset pulse — then `check_run()` |
| `close` | `launcher.cpp` | flush traces, `stop_all`, dump stats, `unbuild_all`, delete |

`bind_comps()` resolves the master→slave port bindings
(`master_port->bind_to_virtual(slave_port)`); `start_all()` calls `start()` on
every model. `reset_all(active)` recurses over children, objects, signals and
registers; the assert/deassert double call is the hardware reset pulse. Reset
is deliberately separate from `build_all`.

### Memory map routing

The `MappingTree` (`mapping_tree.cpp`) is a binary search tree over
`[base, base+size)` intervals. `get(base, size, is_write)` walks down from the
root (right if `base >= entry->base`, else left), checks the range, and on a
miss returns the `default_entry`, so a core access resolves to its target
component in O(log N). The runtime call site lives in the router model, not in
the engine.

## The `vp::` component model

`vp::Block` is the schedulable base: it owns the `vp::BlockTime time` member
and a `virtual int64_t exec()` — the unit the TimeEngine calls back.
`vp::Component : public vp::Block` adds ports, interfaces and sub-blocks.
`vp::Top` is the root and exposes `build_all` / `reset_all` / `start_all` /
`stop_all` / `pause_all` / `flush_all`.

Components connect through typed master/slave ports. An `IoReq`
(`itf/io.hpp`) travels from master to slave and can end in four states:
`IO_REQ_OK` (synchronous), `IO_REQ_INVALID`, `IO_REQ_DENIED`, or
`IO_REQ_PENDING` — a deferred response completed later through the `grant()` /
`resp()` callbacks. Latency accumulates monotonically (`set_latency` keeps the
max). The bus also carries atomic operations (`LR/SC/SWAP/ADD/XOR/AND/OR/
MIN/MAX...`), not just plain reads and writes.

## The time engine

`TimeEngine` (`engine/engine/{include/vp/time,src/time}/time_engine.*`) is the
event scheduler. The queue is not a heap: it is a linked list of clients
(`vp::Block`) kept ordered by increasing `next_event_time`, with
`first_client` the most imminent; `enqueue` inserts in order.

`TimeEngine::exec()` takes `first_client`, advances global time straight to
the event's timestamp (time-warp — no idle ticks), and calls
`current->exec()`, which returns the delta in picoseconds to that client's
next event; the client is then re-enqueued in order. The loop terminates on an
empty queue or on `stop_req` at a timestamp boundary. `run()` is `exec()` plus
`top->pause_all()`, and returns the time of the next event (or -1). `pause()`
sets `stop_req` without taking a lock. When `enqueue` changes the first
client, the engine notifies the launcher through `was_updated()`.

## Clock engines and timed events

Timing is two-level: one global `TimeEngine` in picoseconds, plus one
`ClockEngine` per clock domain counting cycles.

Two event types exist. `vp::ClockEvent` is a cycle-based callback inside a
clock domain, with two modes: repeated-enqueue (queued each time — flexible,
slow) and enable/disable (a "permanent" event that executes every cycle while
enabled — fast). `vp::TimeEvent` is a callback at an absolute timestamp in
picoseconds, independent of any clock. The ISS uses a permanent `ClockEvent`.

`ClockEngine` is itself a `vp::Component` registered as a time-engine client;
`Block::exec()` is virtual and `ClockEngine::exec()` overrides it — that is
where the polymorphic dispatch happens. On each activation it increments
`cycles`, runs all the permanent callbacks, and — if it is the only client —
keeps advancing `engine->time += period` cycle by cycle as a shortcut. Its
return value is the delta in ps: `period` while permanent events exist,
`cycle_diff * period` for the next delayed event, or -1 with no events.
Conversion is `period = 1e12 / frequency`; multiple frequencies coexist as
distinct clients on the shared picosecond axis.

The exact chain that makes the ISS run:

```
TimeEngine::exec()                              current->exec()  [current = the core's ClockEngine]
  -> ClockEngine::exec()
       -> cycles++
       -> instr_event.meth(_this, &event)       [callback = Exec::exec_instr] executes ONE instruction
       -> returns period (ps to the next cycle)
  -> TimeEngine advances time and reschedules
```

The ISS `instr_event` is a permanent `ClockEvent`: `enable()` while not
stalled, `disable()` while stalled (`exec_inorder_implem.hpp`). The
ClockEngine re-executes it once per cycle; nothing re-enqueues it manually.

## The `gv::Gvsoc` control API

`engine/engine/include/gv/gvsoc.hpp` (897 lines) is a purely virtual
interface; the implementation lives in `libpulpvp.so` and the header acts as a
vtable shared across `.so` boundaries.

`class Gvsoc` — control:

| Method | Purpose |
|---|---|
| `open` / `start` / `close` | lifecycle (main controller only) |
| `bind(Gvsoc_user*)` | register the event callbacks |
| `run()` / `stop()` | free-running mode / stop at the current timestamp |
| `step(duration, ...)` / `step_until(ts, ...)` | run and stop after a duration / at a timestamp — the lockstep primitives |
| `update(ts)` / `wait_runnable()` | synchronization with external time |
| `get_time()` / `get_next_event_time()` | introspection |
| `get_component(path)` | component descriptor, or NULL |
| `lock` / `unlock`, `flush`, `terminate`, `quit`, `join` | lifecycle and miscellany |

`get_component` returns `void*` — internally a `vp::Block*`, not a
`vp::Component*` — and the cast to the concrete type is entirely the caller's
responsibility.

`class Gvsoc_user` — callbacks: `has_ended(status)`, `has_stopped()`,
`was_updated()`, `handle_step_end(data)` (async step), `handle_syscall_stop()`.
One rule applies: no GVSOC API calls from inside a callback.

The binding classes (`Io`, `Wire`, `Vcd`, `Power`, plus their `_user` /
`_binding` counterparts) are the interaction channels: memory accesses, raw
signals (including IRQs), traces, power. `GvsocConf` carries `config_path`,
`proxy_socket` and `api_mode`; `gvsoc_new` creates a client, and the first
client is the main controller — the only one allowed to `open`/`close`.

What is *not* there: any architectural state readback. No `get_reg`,
`get_csr`, `get_pc` anywhere in the header. That gap is where our reach-in
comes from.

## How the engine implements the API

`gvsoc_new` (`launcher.cpp`) returns a `gv::ControllerClient` — that object is
the user's `gv::Gvsoc` — or a `Gvsoc_proxy_client` when `proxy_socket` is set.
Behind it sits the `gv::Controller` singleton, which owns `vp::Top`, the
`TimeEngine`, the client list and the `run_count`/`lock_count` bookkeeping.
(A global `gv::controller` symbol also exists but is unused.)

`step_sync(duration)` reduces to `step_until_sync(now + duration)`: it
enqueues the client's pre-allocated `step_event` — a `vp::TimeEvent` whose
callback is `time_engine->pause()` — at the target timestamp, then loops
`run_sync()` until `get_time() >= end_time`.

Threading is decided in `open` from `api_mode`. Async mode spawns a dedicated
`engine_routine` thread (`while(1) run_sync()`). Sync mode creates no engine
thread at all: the engine runs inline in the caller's thread. We use
`Api_mode_sync`, so the engine runs in the thread of Questa's DPI call, in
deterministic lockstep. The engine is runnable when
`run_count == clients.size() && lock_count == 0`.

## How the ISS hooks in and runs

The co-simulation runs on `iss_v2` (`core/models/cpu/iss_v2/`), a modular
ISS. There is no wrapper component: the component at path `soc/core` —
returned by `get_component("soc/core")` as `void*` — is the `Iss` object
itself, an aggregate of public by-value sub-blocks (`regfile`, `exec`,
`timing`, `csr`, `irq`, `lsu`, ...). Those public fields are what make the
bridge's reach-in possible. The subsystems are compile-time slots that a
core personality fills by substituting classes — the shared iss_v2 sources
carry no core-specific `#ifdef`. The CV32E40P personality lives out of
tree in the pulp repo (`cpu/iss_v2/{include,src}/cores/cv32e40p/`),
assembled by the recipe `pulp/cpu/iss/cv32e40p_v2.py` (slots: Csr, Irq,
Core, Exception, Regfile, Events, Exec, priv); the `Events` slot maintains
the architectural commit stream the engine wrapper consumes — see "The v2
bridge" in [`ARCHITECTURE.md`](ARCHITECTURE.md).

The CV32E40P target runs the timed configuration: real fetch through the
prefetcher, modeled stalls.

### Fast and slow handlers

Execution is driven by the `instr_event` with two callbacks: the fast one,
`Exec::exec_instr`, and the slow one, `Exec::exec_instr_check_all`, which is
the default at boot. The slow handler additionally handles pending cache
flushes, exception take (`if (has_exception) current_insn = exception_pc`),
the IRQ check, timing accounting (`timing.insn_account()`) and debug stepping.
The engine switches slow→fast through `can_switch_to_fast_mode()` — blocked
while a gdbserver, tracing or a hardware counter is active — and drops back
fast→slow (`switch_to_full_mode`) on icache flush, exception raise, and
similar events.

### Instruction cache, prefetcher, execution

`get_insn(pc)` (`insn_cache.hpp`) is a page-based fast path with lazy decode:
a fresh slot starts with `handler = iss_decode_pc_handler`, and the first
execution of that PC decodes and caches the instruction. In the TIMED
configuration the prefetcher's `fetch(addr)` may issue a memory `IoReq` and
stall the core on an asynchronous refill (`stalled_inc` / `fetch_response`).

Execution itself is `current_insn = insn->fast_handler(iss, insn, pc)`: the
handler's return value is the next PC, so a retire is precisely a mutation of
`exec.current_insn`. Note that `csr.instret` is a plain `CsrReg` that does not
auto-increment — it cannot serve as a retire counter, which is why the
personality records retires in the commit stream the bridge consumes.

### Traps and IRQs inside the ISS

Traps are two-phase. `Exception::raise` computes the vector and `mcause`,
saves `mepc`, updates `mstatus`, but does not redirect immediately: it sets
`has_exception` and `exception_pc`, and the take happens on the next cycle in
the slow handler (`current_insn = exception_pc`). This is the "trap = 2 ISS
steps" shape the bridge has to handle.

For CV32E40P there is a decode-stage IRQ check before fetch in both the
fast and slow handlers; the CV32E40P-specific ranking comes from the
personality's `Irq` slot. The check runs inside the ISS run loop; the
bridge never calls it.

## Service subsystems

Trace/VCD (`engine/src/trace/`): trace registration feeds an asynchronous
dump thread (`vcd_routine`) and a backend. If an external `Vcd_user` is bound
through `vcd_bind`, GVSOC delegates the dump to the consumer
(`event_update_logical`) and does not write the file itself — that is how a
host captures waveforms.

Power (`engine/src/power/`): a hierarchical activity-based model —
`PowerSource` with per-event quantum, background and leakage components,
interpolated temperature/voltage/frequency tables (`power_table.cpp`),
window-by-window energy integration (`power_trace.cpp`), propagation up the
tree via `parent->inc_*`. Reports to `power_report.csv`. (Headers confirmed;
the implementation only partially read.)

Stats (`engine/src/stats.cpp`): `StatScalar` counters and `StatBw` derived
bandwidth, behind `#ifdef CONFIG_GVSOC_STATS_ACTIVE`; without the define they
compile to no-op stubs.

Interconnect and memory (`core/models/`): the router's `handle_req` applies
stats and the bandwidth limit, resolves the target through
`mapping_tree.get()`, translates the address
(`addr - remove_offset + add_offset`) and forwards. Requests straddling
multiple mappings are split and reassembled. `memory.cpp` is the final
target, with memcheck and atomics support.

Termination: three triggers converge on `TimeEngine::quit(status)` — our exit
device at `0x2000_0000` (`cv32e40p_exit_device.cpp`, a model outside the
engine; PASSED is the magic word `0x075BCD15` at offset 0x00, the same magic
the UVM virtual peripheral uses, exit code at offset 0x04); HTIF tohost
(`cmd & 1` → `quit(cmd >> 1)`); and semihosting `0x10D`. From there the
engine-internal chain is `quit` → `finished=true` → `Controller::run_sync`
sees it → `has_ended()` on the bound user → the bridge's
`BridgeUser::has_ended` sets the flag exposed to SV as `rvviRefIsFinished()`,
which the testbench-side watchdog polls to force `$finish`.

## How other hosts drive the engine

| Driver | Where | Model |
|---|---|---|
| Official DPI wrapper | `engine/dpi-wrapper/src/dpi.cpp` | time-driven: `step_until` + `get_next_event_time` + `wait_runnable` + sleep; signals via `wire_bind`; never touches CPU state |
| SystemC | `engine/engine/src/main_systemc.cpp` | same pattern inside an `SC_THREAD` |
| Proxy (remote) | `engine/engine/src/proxy.cpp` | text protocol over a socket: `run`, `step <ns>`, `step_cycles <domain> <n>`, `get_component`, `get_clock_domains`, `trace`, `event` |
| gdbserver | `core/models/gdbserver/` + the ISS side | GDB RSP over TCP, opt-in |

The generic drivers are time-driven and inject stimuli through
`wire_bind`/`io_bind` without ever reading architectural state; none of them
does step-and-compare. That is why our bridge deviates — retire-based
stepping plus the reach-in. The full comparison is in
[`ARCHITECTURE.md`](ARCHITECTURE.md).

Our own usage, in one line: `Api_mode_sync`;
`gvsoc_new → bind(BridgeUser) → open → start → get_component("soc/core")`;
one `step(20000 ps)` per clock, gated by the personality's commit stream
(one pop per retire); IRQs driven over `wire_bind` into the
`cv32e40p_irq_injector` component; reach-in on `iss.regfile/csr/exec/irq`;
no `step_until`, no `get_next_event_time`. The bridge workarounds — CSR
read fixups, write masks, force-reset, `skip_irq_check` re-assert, the
runaway detector — all descend from the missing state API and from the
symbol isolation described next.

## Why we did not use the gdbserver

GVSOC ships a gdbserver with `reg_get`/`reg_set`/`stepi`, so it is a fair
question. It was not a viable route, for three independent reasons.

First, the reach-in was never a choice *against* the gdbserver: it was forced
by a linking barrier. The bridge reads state only through public struct
fields, never through ISS methods (`get_csr`, `access`, ...), because those
methods are compiled into the model's `.so` and are not linkable from a
DPI-loaded library — `RTLD_DEEPBIND` isolates the model's symbols. The
bridge was born in a single commit (`42d2781`) with the reach-in already in
place.

Second, the component is simply not there on our platform. The gdbserver is
an opt-in `vp::Component` instantiated explicitly only on pulp_open,
siracusa, cheshire and magia; the `cv32e40p-v2-standalone*` platforms do
not create it. (Upstream later added per-target
auto-instantiation — see "Upstream drift".) There was, and is, no
`Gdbserver_core` on `soc/core` to talk to.

Third, even if it were instantiated, the ISS implementation
(`core/models/cpu/iss_v2/src/gdbserver.cpp`) is structurally unsuitable for
step-and-compare:

- `gdbserver_regs_get` exposes 33 registers — 32 GPRs plus
  `reg[32] = current_insn` (the PC) — and zero CSRs, while the compare needs
  the full enabled CSR set (mstatus, mtvec, mcause, mepc, the FP and debug
  CSRs, the retired-instruction counters, ...);
- the single-register `gdbserver_reg_get` is unimplemented (prints and
  returns 0), and `reg_set` only handles the PC;
- `gdbserver_stepi` does not execute the instruction: it raises the step
  mode, releases the halt and returns; the advance happens later in the
  event loop and completion is signalled back over the RSP socket — an
  asynchronous handshake, not a synchronous in-process step;
- the API speaks GDB register indices; CSRs would need a qXfer
  target-description XML that is not wired up.

The reach-in gives synchronous access to PC, GPRs and CSRs — exactly what
step-and-compare needs. Adopting the gdbserver would have meant instantiating
the component, bypassing RSP, extending the interface with CSR access and a
synchronous step, and handling the async signalling: a rewrite, not a reuse.

## Improvements worth pursuing

Adaptive stepping through `get_next_event_time()`. The official DPI and
SystemC drivers already use it; for us it would kill the blind 2000-cycle
poll and with it the root cause of the runaway detector firing on WFI and
far-future events. Adoptable today — the API is already in the vendored
engine. The one risk is calibration against retire detection, so it should
land as a prototype plus a full regression pass. If we pursue only one item
on this list, it is this one.

IRQ over `wire_bind()` — done. Architecturally the right thing (it is what
the official dpi-wrapper does): `gvsoc_engine_v2.cpp` binds the 19 lines
of the `cv32e40p_irq_injector` component (pulp repo,
`pulp/cv32e40p_irq_injector/`) at init and drives them from
`rvviRefNetSet`. The desync worry proved manageable because the settle
logic and the informed injection were kept on top of the wires — see
"IRQ injection" in [`ARCHITECTURE.md`](ARCHITECTURE.md).

Engine bump plus `gv_api_version()`. Upstream now versions the control API;
adopting it turns a stale-`.so` SIGSEGV into a fail-fast version error. See
"Upstream drift" for what the bump involves.

A layout canary on the reach-in. Purely on our side and cheap: turn the
silent SIGSEGV after a core bump into a diagnosable startup error.

Upstream state-readback API. A `reg_get`/`csr_get`/`pc_get` per hart — for
instance a `vp::CoreStateIf` returned by `vp::Block`, the same pattern as
upstream's `debug_mem_if()` — with corresponding `gv::` vtable slots would
remove the reach-in entirely: the bridge would compile against `gv/gvsoc.hpp`
alone and survive core bumps. Moderate ask: the ISS getters already exist
(`get_reg_untimed`, `exec.current_insn`, the csr objects); the real work is
CSR-by-address dispatch and multi-hart semantics. Extending the gdbserver
instead would cost more, for the reasons above.

Upstream per-instruction step or retire hook. A `step_instructions(n)`
built on a retire budget that pauses the time engine, or a registrable retire
callback, gated the way the gdbserver already gates fast mode (zero cost when
off). The pause-on-event mechanism and the fast-mode gating both exist
already; the design work is the retire semantics around traps and IRQ entry.
The commit stream already gives the bridge most of this without an engine
change — the CV32E40P personality's `Events` slot records commits in a ring
the bridge pops one per retire, with a bounded cycle budget and the runaway
net as safety checks (a timeout means "no new commit showed up") — but an
upstream retire hook would still be the cleaner engine-level version of the
same idea: one engine crossing per retire instead of clocking cycle-by-cycle
while the ring is empty.

Runtime trace control and the debug-memory backdoor. Both exist upstream and
come with the engine bump: `trace_level_set`/`trace_subscribe` make ISS
instruction/LSU/IRQ tracing switchable while the simulation runs (turn it on
only around a divergence window), and `vp::DebugMemIf` gives zero-time,
out-of-band memory access while the sim is paused — useful for test-image
loading, divergence-point memory dumps, or targeted ref←dut resyncs. The
debug-mem path additionally needs `debug_mem_access` implemented in the pulp
memory model, a small change in our fork.

## Odds and ends

- The bus supports deferred responses (`IO_REQ_PENDING` with `grant`/`resp`
  callbacks) and cumulative latency. Our exit device always answers a
  synchronous `IO_REQ_OK`; anyone extending toward devices with back-pressure
  must handle PENDING.
- `testandset.cpp` emulates test-and-set at the interconnect level (read,
  forward, write -1) on top of the AMO opcodes the `IoReq` already carries.
- `bus_watchpoint.cpp` doubles as a RISC-V syscall handler for semihosting
  host I/O (`getcwd`, `fcntl`, `mkdirat`, `unlinkat`, ...) — that is how
  bare-metal programs do host I/O without a UART.
- The proxy text protocol is a second step-by-step control channel besides
  DPI (per-domain `step_cycles`, `get_component`, `trace`/`event`). Not
  active on our target, but it exists.
- The exit device's PASSED magic `0x075BCD15` is the same word the UVM
  virtual peripheral uses — the coupling point between the RTL/UVM flow and
  standalone GVSOC.
- Dead code: the legacy C function `handle_syscall(Iss*, ...)` calls
  `iss_exit()`, which is not defined anywhere in the tree, so it cannot be
  part of the active flow; the live path is the `Syscalls::` class plus HTIF.

Not verified: checkpoint/restart is absent from the vendored engine (the
upstream `restart()` is in the missing commits); the exact `prefetcher_size`
for the `cv32e40p` class and where the branch penalty is accounted. The
`iss_v2` accuracy question is settled empirically: the bridge passes the
full no-pulp regression.

## Upstream drift and what a bump would take

As of July 2026 the vendored engine (`a8c57439`) is about 77 commits behind
`gvsoc/gvsoc-engine` main. Upstream now versions the public API —
`GV_API_VERSION = 10` plus an `extern "C" int gv_api_version()` accessor; our
snapshot predates the versioning entirely. Additions we lack:

- `gv_api_version()` (`99a3ffc7`, `ea216770`) — the fail-fast compatibility
  check;
- runtime trace control (`trace_subscribe`/`trace_unsubscribe` at v9,
  `trace_level_set` at v10) — `--trace`/`--trace-level` become switchable at
  runtime;
- the debug-memory backdoor (`9a73ab40`, `20400ea7`): `vp::DebugMemIf`,
  zero-time out-of-band memory access on a paused sim, with a flat
  `DebugMemMap` built at first access and an optional host-pointer fast
  path; terminal memories must implement `debug_mem_access`, which the
  vendored pulp memory model does not yet;
- in-process `restart()` (v3), `GvsocConf::proxy_enabled` (v5), a
  console-output channel (v6), `get_memcheck_fault()` (v7), exact proxy-side
  `step_cycles` (`23d670b4`);
- per-target gdbserver auto-instantiation (`b93bf7b5`): Python-only (about
  40 lines across the two runners), it adds an inert `gdbserver` node at the
  tree root unless the target already declares one. It does not move or
  rename `soc/core`, and it only takes effect when `gvsoc_config.json` is
  regenerated with the new runners — an engine-only bump reusing a
  pre-generated config does not instantiate it.

A bump looks moderate. Checked directly on upstream main: the sync-mode
contract is unchanged (`Controller::start()` still takes the engine mutex
when `!is_async`, `stop()` still requires the engine locked — our shutdown
workaround remains both valid and necessary), and `Gvsoc_user` keeps the same
five callbacks with empty defaults. The real risk is skew between a new
engine and our older core/pulp forks: upstream reworked the port signatures
(`9968edfd`, class-based for all ports) and the platform-tree generation, and
legacy string-signature support has already needed a follow-up fix
(`7aa44582`). The API changelog documents breaking layout changes at v2, v4,
v5 and v6, but they only affect `Vcd` consumers, which this bridge is not.
One watch-item: sync `join()` now waits for every client's `has_quit`; we are
single-client and call `quit(0)` first, so this should be fine, but it wants
a smoke test. Bump order: rebuild the bridge against the new headers,
regenerate the config, then `make check-rvvi` plus a dual-trace
`illegal_instr_test` and `hello-world` with `USE_ISS=YES` before trusting any
regression score.

One expectation to keep straight for any core bump: the reach-in is still
there. The iss_v2 gdbserver exposes no CSR readback, so the bridge acquires
the core through `get_component` and reads public struct members — the
component is the `Iss` object itself — with the `static_assert` layout
tripwires and the runtime canary as the mitigation. The Makefile builds the
engine wrapper against the generated `isa_cv32e40p_v2_*` ISA headers, so a
core bump must be followed by a full library rebuild before any validation
run.

## References

Engine (`gvsoc/engine/engine/`): `include/gv/gvsoc.hpp` · `src/launcher.cpp`
and `include/vp/controller.hpp` · `time/time_engine.{hpp,cpp}` ·
`clock/{clock_engine,clock_event,block_clock}.{hpp,cpp}` ·
`src/{top,component,block,mapping_tree}.cpp` · `src/{trace,power}/*` ·
`src/stats.cpp` · `include/vp/gdbserver/gdbserver_engine.hpp` ·
`dpi-wrapper/src/dpi.cpp` · `src/proxy.cpp`

Core / ISS (`gvsoc/core/models/`):
`cpu/iss_v2/` (the shared framework) ·
`cpu/iss_v2/{src/exec/exec_inorder.cpp,include/exec/exec_inorder*.hpp}` ·
`cpu/iss_v2/include/insn_cache.hpp` · `cpu/iss_v2/include/prefetch/` ·
`cpu/iss_v2/src/{exception,syscalls,htif,gdbserver}.cpp` ·
`interco/router/router.cpp` · `interco/{bus_watchpoint,testandset}.cpp` ·
`memory/memory.cpp` · `gdbserver/{gdbserver,rsp}.cpp`

Platform (`gvsoc/pulp/`):
`cv32e40p-v2-standalone{,-fpu,-zfinx,-nopulp}.py` ·
`pulp/cpu/iss/cv32e40p_v2.py` (the personality recipe) ·
`cpu/iss_v2/{include,src}/cores/cv32e40p/` (the personality) ·
`pulp/cv32e40p_irq_injector/cv32e40p_irq_injector.{cpp,py}` ·
`pulp/cv32e40p_exit/cv32e40p_exit_device.cpp`

Our side: `gvsoc_engine.{cpp,hpp}`, `gvsoc_engine_v2.cpp` and
[`ARCHITECTURE.md`](ARCHITECTURE.md).

External: [the GVSoC paper (arXiv:2201.08166)](https://arxiv.org/pdf/2201.08166) ·
[gvsoc/gvsoc on GitHub](https://github.com/gvsoc/gvsoc) ·
[user docs](https://gvsoc.readthedocs.io/) ·
[developer docs](https://gvsoc-developer.readthedocs.io/) ·
[GAP SDK GDB server](https://greenwaves-technologies.com/manuals_gap9/gap9_sdk_doc/html/source/tools/docs/gvsoc/gdb_server.html)
