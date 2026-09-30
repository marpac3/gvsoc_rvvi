# Architecture of gvsoc_rvvi

gvsoc_rvvi makes the CV32E40P model of GVSOC the reference model of a CV32E40P RTL simulation. This repo has the GVSOC
targets of the model, a client (cosim_client.cpp) that runs a target inside another program and steps the model one
instruction at a time, the RVVI API and an extension of it to bring the interrupt and debug decision points from the TB
to the GVSOC model.

```
test/cosim_run --> Cv32e40pCosimClient --> cv32e40p_cosim* platform
                   (cosim_client.cpp)      CV32E40P model, cosim.hpp
```

The model decides to take interrupts and debug requests by itself, and the DUT testbench only gives it the signals as
input.

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
- `read_gpr()` and `read_csr()` read the state of the model: the reset state before the first step, then the
  current state.

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
