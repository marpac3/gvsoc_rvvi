# Architecture of gvsoc_rvvi

gvsoc_rvvi makes the CV32E40P model of GVSOC the reference model of a CV32E40P RTL simulation. This repo has the GVSOC
targets of the model, a client (cosim_client.cpp) that runs a target inside another program and steps the model one
instruction at a time, the RVVI API and an extension of it to bring the interrupt and debug decision points from the TB
to the GVSOC model.
The bridge (rvvi_api.cpp, built as libcv32e40p_rvvi.so) implements the RVVI API and its extension on the client: after
every instruction retired by the RTL, it steps the model and compares the state of the DUT with the state of the model.

```
RTL testbench
  RVFI -> RVVI-TRACE -> rvvi_trace2api.sv
  decision monitor (interrupts, debug)
      |
      |  RVVI-API (DPI) + decision-point extension
      v
libcv32e40p_rvvi.so (rvvi_api.cpp)
      |
      v
Cv32e40pCosimClient (cosim_client.cpp)
      |
      v
cv32e40p_cosim* platform (CV32E40P model, cosim.hpp)
```

The model decides to take interrupts and debug requests by itself, and the DUT testbench only gives it the signals as
input.
The reference state is never written from the DUT.

## GVSOC targets (`gvsoc/`)

A GVSOC target is a Python file that describes a platform: the models it contains and how they are connected.
`gvsoc/` holds the targets of the CV32E40P model.

GVSOC is not part of this repository, and must be installed on the machine. Clone the GVSOC repository
(https://github.com/gvsoc/gvsoc) and prepare it as its README says, with its submodules and Python requirements.
Its gvsoc-pulp submodule must have the CV32E40P model (`pulp/cv32e40p/`). Then run the build of GVSOC in its
clone, with the targets of this repository. `<gvsoc>` is the path of the GVSOC clone, and `<this repository>` the
path of this repository:

```
cd <gvsoc> && make build \
    MODULES=<this repository>/gvsoc \
    TARGETS="cv32e40p_cosim;cv32e40p_cosim_pulp;..."
```

`MODULES` makes GVSOC look for targets in `<this repository>/gvsoc` too, and `TARGETS` names the targets whose
models GVSOC compiles. The build installs GVSOC and these models in `<gvsoc>/install`, or in the directory given
with `INSTALLDIR`.

Five RTL parameters of the core, FPU, ZFINX, COREV_PULP, COREV_CLUSTER and NUM_MHPMCOUNTERS, change the code of the core
model: for example which instructions it decodes and which CSRs it has. The model is compiled with fixed values of these
parameters, and they cannot change at run time. So there is one target for each combination of values that core-v-verif
uses. The suffixes of the name give the values: `_pulp`, `_cluster`, `_fpu` and `_zfinx` set COREV_PULP, COREV_CLUSTER,
FPU and ZFINX to 1, and `_mhpm29` sets NUM_MHPMCOUNTERS to 29. A parameter without its suffix is 0, except
NUM_MHPMCOUNTERS, which is 1.

| target | core-v-verif configurations |
|---|---|
| `cv32e40p_cosim` | default, no_pulp |
| `cv32e40p_cosim_pulp` | pulp |
| `cv32e40p_cosim_pulp_fpu` | pulp_fpu |
| `cv32e40p_cosim_pulp_fpu_zfinx` | pulp_fpu_zfinx |
| `cv32e40p_cosim_pulp_cluster` | pulp_cluster |
| `cv32e40p_cosim_pulp_cluster_fpu` | pulp_cluster_fpu |
| `cv32e40p_cosim_pulp_cluster_fpu_zfinx` | pulp_cluster_fpu_zfinx |
| `cv32e40p_cosim_mhpm29` | num_mhpmcounter_29 |

The `_1cyclat` and `_2cyclat` configurations of core-v-verif use the target of their configuration without the
suffix: the FPU latency changes the timing of the RTL, not the model.

The targets instantiate the `cv32e40p_testbench` SoC of gvsoc-pulp, which has the memory map and the virtual
peripherals of the uvmt_cv32e40p testbench. They do not print the output of the test program: in co-simulation
the UVM testbench prints it. They have three parameters:

- `binary`, the ELF of the test program;
- `stop_on_exit`, true by default. The test program reports its end by writing a register of the testbench (test
  status, exit or signature). With `true`, GVSOC stops at that write, so a standalone run such as `cosim_run`
  ends there. With `false`, the model keeps running after the write, like the RTL core, which runs until the UVM
  testbench ends the test. core-v-verif passes `stop_on_exit=false`, so that the model also executes the
  instructions that the RTL core retires after the write;
- `mtvec_addr`, which drives the `mtvec_addr_i` input of the core, so that each test passes its plusarg without
  rebuilding the platform.

`gvrun ... prepare` writes the platform configuration of a test program, `gvsoc_config.json`, which the client
opens.

## Co-simulation client (`bridge/cosim_client.*`)

`Cv32e40pCosimClient` runs GVSOC in the same process through the public engine API (`gv::Gvsoc`), in synchronous
mode: the engine runs inside `step()`, in the caller thread. It uses only that API and `cosim.hpp` of gvsoc-pulp,
and depends on nothing of the core layout.

- `open()` opens `gvsoc_config.json`, and the runtime configuration next to it when present, and takes the
  `cv32e40p_cosim` interface of `soc/core`. The engine loads models with `RTLD_GLOBAL`, so the client finds the
  symbol of the CV32E40P model with `dlsym`.
- `input()` sets the levels of the `irq_i[31:0]` and `debug_req_i` pins, `sample()` reports that the RTL sampled
  the pins of one domain, interrupts or debug, and `opportunity()` reports a decision point. The model applies them
  in call order, right before the instruction that the decision point names: see the decision-point extension
  below.
- `step()` lets the model execute one more instruction and returns its commit record, with the boundary events
  published before it (interrupt taken, debug entry, wake-up). The decision points of that instruction must be
  reported first: the model treats an instruction without a decision point as an error. The client advances the
  engine one clock period at a time, and fails the step after 1,000,000 core cycles without a record, so that a
  model that spins or sleeps fails instead of hanging the caller.
- `volatile_csr()` declares a CSR whose value depends on timing that the model does not reproduce, and
  `volatile_read()` gives the model the value that the RTL read from it, before the instruction that reads it
  executes.
- `external_region()` declares a memory range whose loads take their data from the testbench, through the
  `external_data` callback of `step()`.
- `read_gpr()`, `read_fpr()` and `read_csr()` read the state of the model: the reset state before the first step,
  then the current state.

## cosim_run (`test/`)

`cosim_run [-v] <gvsoc_config.json> [max_records]` runs a target through the client alone, without a DUT. It
sends no input, and reports one DISPATCH decision point before every instruction. It prints one line per commit
record in the instruction trace format of GVSOC (pc, opcode), so that the output can be compared with the
instruction trace of a standalone run of the target; `-v` adds the boundary events and the register and CSR
writes.

## Decision-point extension (`bridge/rvviDecisionApi.h`, `sv/rvviDecisionApiPkg.sv`)

An interrupt or a debug request arrives at some random time during the execution of some instruction. The instruction
before which the DUT takes it depends on its pipeline: when it registers the input, and in which states its controller
looks at it. A reference model has no pipeline, so it cannot find that instruction by itself. With the base RVVI-API a
testbench delivers the input nets with `rvviRefNetSet()`, and the reference either takes an event as soon as it sees the
net, or waits for the DUT to report that it took one. The first is too early whenever the DUT registers the net later,
or looks at it only in some states; the second lets the DUT choose for the reference, so a wrong choice goes unseen.

The extension lets the DUT report the moments of its decisions, and never the decisions themselves:

| call | what the DUT reports |
|---|---|
| `rvviRefNetSet(net, value, when)` (base API) | a new level on an input net |
| `rvviRefNetGroupSample(group)` | it copied the nets of a group into its registers: its decisions now see these levels |
| `rvviRefDecisionPoint(hartId, kind, order)` | its controller checked for interrupts and debug, before the instruction that retires with RVFI order `order` |

The reference then decides by itself, at the same points and with the same sampled levels, whether it takes an
interrupt, enters debug mode or wakes up. It never learns what the DUT decided. When the two decide differently,
the next retired instruction differs, for example the first instruction of a handler against the next instruction
of the program, and the compare of that retire fails.

`rvviDecisionApiPkg` imports the two calls in SystemVerilog, through DPI.

### Net groups and samples

A net group is the set of nets that one register of the DUT copies at the same clock edge, and a sample is that
copy. The base RVVI-API places nets in groups (`rvviRefNetGroupSet()`) without saying what a group is: the
extension gives groups this meaning. Between two samples of its group, a new level of a net is known to the
reference but not used by its decisions, as in the DUT, whose controller reads the register and not the pin.

For CV32E40P the groups follow the RTL, and only this placement is accepted:

| group | nets | register in the RTL |
|---|---|---|
| 1 | `MSWInterrupt`, `MTimerInterrupt`, `MExternalInterrupt`, `LocalInterrupt0..15` (irq_i bits 3, 7, 11, 16..31) | `irq_q` in `cv32e40p_int_controller.sv`, read as `mip`, on the gated clock: no sample while the core sleeps |
| 2 | `haltreq` (debug_req_i) | `debug_req_q` in `cv32e40p_controller.sv`, on the free-running clock: set while the pin is high, cleared in debug mode once the pin is low |

The testbench reports a sample at each clock edge where the register loads: `irq_q` at every edge of the gated
clock, `debug_req_q` at the edges where the pin is high or the core is in debug mode. A sample of group 2 at
another edge would clear the request in the reference, while the RTL keeps it.

Two rules of the RTL use the pins as well: the wake-up from WFI looks at the interrupt pins enabled in `mie`, since
the gated clock is stopped, and a debug request is pending while the pin or `debug_req_q` is set.

### Decision points

A decision point is a cycle in which the controller of the DUT checks for interrupts and debug requests, and could
take one. `order` is the RVFI order of the next instruction that retires. When the DUT takes an interrupt or enters
debug mode at the point, it drops the instruction it was about to issue, and the first instruction of the handler
retires with that same order: `order` names the place of the decision, whatever the decision is. The reference
numbers its commit records in the same way, one per retired instruction.

The kinds, `RVVI_DECISION_<kind>` in `rvviDecisionApi.h`, are the states in which the CV32E40P controller
(`cv32e40p_controller.sv`) checks:

| kind | state of the RTL controller | what it checks |
|---|---|---|
| `DISPATCH` | `DECODE` or `DECODE_HWLOOP`, with a valid instruction in ID | outside debug mode, a debug request or a trigger, then the interrupts |
| `FIRST_FETCH` | `FIRST_FETCH`, before the first instruction after reset or a wake-up | the interrupts, unless a debug request is pending or the core is in debug mode |
| `SLEEP` | `SLEEP`, and `FLUSH_WB` after a WFI that sleeps | a debug request, which enters debug mode; in `SLEEP`, also an enabled interrupt pin, which wakes up the core |
| `BOOT` | `BOOT_SET`, out of reset | a debug request, which enters debug mode before the first instruction |

In `DECODE` a taken branch in EX, a data error and a fetch error come first: in such a cycle the controller does
not check, and there is no decision point. A WFI sleeps only outside debug mode and single step, with no debug
request pending, no trigger match and without COREV_CLUSTER; otherwise it is a nop, with no SLEEP point.

With COREV_CLUSTER the controller also checks in the two states of `cv.elw`: `ELW_EXE` for debug requests and
triggers, `IRQ_FLUSH_ELW` for interrupts. The extension has no kind for them, so an interrupt or a debug request
taken during a `cv.elw` is not covered.

Several points can come before the same instruction: for example one for every cycle in which the instruction
waits in DECODE, or one for every cycle of sleep. They are decided in turn, each with the samples reported before
it.

### Rules

- The testbench makes the calls in the order of RTL time. Within a clock cycle: the new pin levels, then the
  decision point of the cycle, then the samples taken at the clock edge that ends the cycle, which the next
  decision sees.
- Every instruction has at least one decision point before it. The CV32E40P model treats an instruction without
  one as an error.
- A decision point for an instruction that the reference has already executed is rejected.
- While the reference sleeps in WFI it accepts only SLEEP points: a point of another kind means that the DUT woke
  up and the reference did not, which is an error.
- The reference applies the calls when the instruction named by `order` is its next one, so the testbench reports
  them as they happen and steps the reference on retire. Since the DUT decides on an instruction before it retires
  it, all the decision points of an instruction have been reported when the testbench steps the reference for it.
- A testbench may skip a sample that does not change the levels of its group, and a decision point that repeats
  the previous one (same kind and same `order`) with no new level or sample in between: neither changes the
  decision.

### In the reference

The CV32E40P model of GVSOC queues the calls and applies them in call order, right before it executes the
instruction that the decision point names. A new level updates its pins, a sample of group 1 copies the interrupt
pins into `mip`, a sample of group 2 copies `haltreq` into its `debug_req_q`, and a decision point runs the check
of its kind. When the check takes an interrupt, enters debug mode or wakes up the core, the model publishes a
boundary event before the next commit record.

### Example

A timer interrupt, with `mstatus.MIE` and `mie.MTIE` set:

1. `irq_i[7]` rises: `rvviRefNetSet(MTimerInterrupt, 1, ...)`. The reference stores the level; its decisions do
   not see it yet.
2. At the next edge of the gated clock `irq_q` copies the pin: `rvviRefNetGroupSample(1)`.
3. The controller is in DECODE, before the next instruction to retire, number 42:
   `rvviRefDecisionPoint(0, RVVI_DECISION_DISPATCH, 42)`.
4. When the testbench steps the reference for retire 42 (`rvviRefEventStep()`), the reference applies the three
   calls in order: it sets `mip.MTIP`, sees the enabled interrupt and takes it. Instruction 42 is the first
   instruction of the handler, as in the DUT. With `mie.MTIE` clear, instruction 42 would be the next instruction
   of the program, in both.

### Finding the moments in the RTL

A testbench finds these moments inside the RTL: the pins `irq_i` and `debug_req_i`; the enable of the clock gate,
for the samples of `irq_q`; the condition that loads `debug_req_q`; the state of the controller (`ctrl_fsm_cs`),
for the decision points. It computes `order` by counting the instructions issued from ID. In core-v-verif this is
`uvmt_cv32e40p_gvsoc_decision_monitor`.

## Bridge (`bridge/rvvi_api.cpp`, `libcv32e40p_rvvi.so`)

The bridge implements the RVVI-API of the RVVI submodule and the extension on the client. It keeps two shadows of
the architectural state: the DUT shadow, which the testbench writes with `rvviDut*Set` after every retired
instruction, and the reference shadow, written only by the commit records of the model.

**Start.** `rvviRefInit()` reads `GVSOC_CONFIG`, the path of `gvsoc_config.json` (required), and
`GVSOC_RVVI_TRACE` (optional), and opens the client. Then it reads the reset state of the model: the GPRs, the
FPRs when the core has F registers (no FPU, or Zfinx, means none), and the CSRs. This is the only direct read of
the model state: from here on the reference shadow follows the commit records. It declares to the model the
volatile CSRs and the volatile memory ranges set before, and sends the pin levels set before the model existed.

**Every retire.** For each instruction that the DUT retires, the testbench passes the registers and CSRs that the
instruction wrote (`rvviDutGprSet`, `rvviDutFprSet`, `rvviDutCsrSet`), then calls `rvviDutRetire`, or `rvviDutTrap`
if the instruction trapped. `rvviRefEventStep()` steps the model by one
commit record. The CSR writes of the boundary events before the record (interrupt taken, debug entry) go to the
reference shadow first, then the register and CSR writes of the record. A write to `fcsr`, `frm` or `fflags`
changes the other two, since `fcsr` is {frm, fflags}. The instret counters change on every retire and are not in
the CSR list of the record: the bridge takes them from its `minstret` field.

**Compares.** After the step the testbench calls the compares:

- PC and instruction. For a compressed instruction the RVFI of the core reports the fetched word, whose upper
  half is the next parcel, so only the lower half is compared;
- trap and debug mode. `rvviDutTrap` carries no debug mode, and the bridge cannot take it from the instruction
  before, which can be a `dret`: for an instruction that trapped the debug mode is not compared, and the next
  instruction shows where the trap went;
- all GPRs and all FPRs;
- every compare-enabled, non-volatile CSR that the model implements and the DUT reports, with the masks of
  `rvviRefCsrCompareMask`. A CSR the model does not implement has no reference state: an access to it traps in
  the model, and the PC and trap compares catch that.

Each compare category prints its first 10 mismatches in detail.

**Reset state.** The DUT shadow starts from the RTL reset state, with all registers zero, and the reference
shadow from the reset state of the model, so the first compare checks the reset state of the model.

**State setters.** The reference is never written from the DUT: `rvviRefCsrSet`, `rvviRefGprSet`, ... are
rejected, except a CSR set to the value the model already holds at reset.

**Volatile CSRs.** `rvviRefCsrSetVolatile` declares the CSR to the model and takes it out of the CSR compare. When
the DUT retires a CSR instruction that reads a volatile CSR into rd != x0, the value it wrote to rd goes to the
model before the step.

The testbench declares the HPM counters volatile (core-v-verif does), because they can count timing events (cycles,
stalls, fetch misses), which the model does not count. The model does count the instruction events: retired, load,
store, jump, branch, branch taken and compressed instructions (`hpm_events` in `cv32e40p_cs_registers.sv`). When the
selector of a counter has only these events in the model, or none, the counter of the model counts as the one of the
RTL. A read of it (mhpmcounter3..31, its upper half or a user alias) is then compared: the value the DUT read
against the counter of the model, which has not executed the read yet. Then the model takes the DUT value, as for
every volatile CSR.

**Volatile memory.** A range set with `rvviRefMemorySetVolatile` becomes an external region of the model. A load
from it takes the value of its destination register in the DUT, and the model applies the access width and the
extension. This is what volatile memory means in RVVI-API: a test that reads the random-number generator of the
testbench, for example, needs it.

**Stores.** Stores are compared on the data bus. The testbench reports each write accepted on the DUT data port with
`rvviDutBusWrite` (byte enables relative to the address). The bridge cuts the DUT writes and the stores of the model
into word beats, and compares the two sequences in program order: address, byte enables and enabled bytes. The DUT
writes the bus before it retires the store, while the reference writes it when the store retires, so a beat waits on
one side until the other side has it.

At the end of the test, in `rvviRefShutdown()`, a store of the model without its DUT beats is a mismatch. DUT beats
left over are logged with the retire they followed: they belong to a store the DUT had not retired when the
simulation stopped, which the reference never stepped. A spurious DUT write in the middle of a run shifts the
sequence and fails the compare of the next store; only a spurious write after the last store of the run is not a
mismatch: it is logged as a store the DUT had not retired.

**Shutdown.** `rvviRefShutdown()` runs at the end of the test, before the testbench reads the metrics. It logs the
metrics and the compare-enabled CSRs that were never checked, because the model does not implement them or the
DUT never reported them.

**Not provided.** The functions that would write the reference state (`rvviRefPcSet`, `rvviRefGprSet`,
`rvviRefMemoryWrite`, ...), load a program (it comes from the platform configuration) or handle vector registers
fail with "... is not supported by the GVSOC reference": the reference state is only written by its own
execution. `rvviDutCycleCountSet` is accepted and ignored, since cycle-accurate comparison is not part of the
functional compare.

## rvvi_trace2api (`sv/rvvi_trace2api.sv`)

`rvvi_trace2api` is the SystemVerilog module that connects the testbench to the bridge. The testbench instantiates it
and connects it to its RVVI-TRACE interface (`rvviTrace`).

At every clock edge the interface holds the instructions that the core retired in the cycle. For each one it has:

- `valid`, `pc_rdata` and `insn`: the instruction retired, its PC and its encoding;
- `trap`: the instruction trapped;
- `debug_mode`: the core is in debug mode;
- `x_wdata` and `f_wdata`: the values written to the GPRs and to the FPRs; `csr`: the values of the CSRs;
- `x_wb`, `f_wb` and `csr_wb`: a flag for each register and CSR, set when the instruction wrote it. The core-v-verif
  testbench sets the flag of a CSR when its value changes, also when a trap or a counter changes it.

At every clock edge, for each retired instruction, `rvvi_trace2api` calls the bridge through DPI:

1. it passes the registers and the CSRs whose flag is set (`rvviDutGprSet`, `rvviDutFprSet`, `rvviDutCsrSet`);
2. with `NETS` = 1, it passes the interrupt and debug nets that changed (`rvviRefNetSet`);
3. it closes the instruction with `rvviDutRetire`, or with `rvviDutTrap` if it trapped;
4. it makes the model execute its next instruction (`rvviRefEventStep`), and compares the PC, the instruction, the GPRs,
   the FPRs and the CSRs of the DUT with those of the model.

If 50 consecutive instructions fail a compare, `rvvi_trace2api` stops the simulation.
`+rvvi_max_consecutive_mismatch=<n>` sets a different number.

The parameter `NETS` says who passes the interrupt and debug nets to the model:

- `NETS` = 1: `rvvi_trace2api` passes the nets that changed, taken from the interface. It does not pass samples or
  decision points, but the model needs a decision point before every instruction: with `rvvi_trace2api` alone the
  co-simulation stops at the first instruction. This comes from reading the code; it has not been tried.
- `NETS` = 0, the default: the testbench passes the nets, their samples and the decision points itself
  (`rvviDecisionApiPkg`). core-v-verif does this, with `uvmt_cv32e40p_gvsoc_decision_monitor`.

`sv/gvsoc_rvvi.f` lists the SystemVerilog files of RVVI and of the bridge, from `GVSOC_BRIDGE_HOME`, so that a testbench
includes them with one `-f`. The `rvvi.f` of RVVI takes its files from an Imperas installation (`IMPERAS_HOME`).

## Debugging

- `GVSOC_RVVI_TRACE=first:last` prints the commit records and boundary events of the model for those retires,
  with their register writes and memory accesses, and the CSR changes the DUT reports.
- `rvviErrorGet()` returns the last error, with the message of the model when the model failed.
- `cosim_run` runs the model of a target alone, to check it without the RTL.
