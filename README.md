# gvsoc_rvvi: CV32E40P reference model for RISC-V co-simulation, on GVSOC

This repository compares a CV32E40P RTL simulation in lock step with the CV32E40P model of
[GVSOC](https://github.com/gvsoc/gvsoc), through the
[RVVI](https://github.com/riscv-verification/RVVI) API.

| directory | content |
|---|---|
| `gvsoc/` | GVSOC module: `cv32e40p_cosim*` targets on the memory map of the core-v-verif testbench |
| `bridge/` | `libcv32e40p_rvvi.so`: the RVVI-API on the co-simulation interface of the core model |
| `sv/` | `rvvi_trace2api.sv`: from an RVVI-TRACE interface to RVVI-API calls, one step-and-compare per retire; `rvviDecisionApiPkg.sv`; `gvsoc_rvvi.f`, the file list for the simulator (`GVSOC_BRIDGE_HOME` = this repository) |
| `test/` | `cosim_run`: runs a target through the interface alone and prints the commit records |
| `RVVI/` | RVVI headers and SystemVerilog packages (submodule) |

## Build

```
git submodule update --init
cd <gvsoc> && make build MODULES=<this repository>/gvsoc INSTALLDIR=<install> \
    TARGETS="cv32e40p_cosim;cv32e40p_cosim_mhpm29;cv32e40p_cosim_pulp;cv32e40p_cosim_pulp_fpu;cv32e40p_cosim_pulp_fpu_zfinx;cv32e40p_cosim_pulp_cluster;cv32e40p_cosim_pulp_cluster_fpu;cv32e40p_cosim_pulp_cluster_fpu_zfinx"
make GVSOC_HOME=<gvsoc> GVSOC_INSTALL=<install>
```

## Use

1. For each test program, write the platform configuration:
   `gvrun --target-dir=<this repository>/gvsoc --target=<target> --parameter binary=<elf> --work-dir=<dir> prepare`,
   with `<target>` one of `cv32e40p_cosim[_pulp[_cluster][_fpu[_zfinx]]]` or `cv32e40p_cosim_mhpm29`
   (NUM_MHPMCOUNTERS=29)
2. Run the simulator with `GVSOC_CONFIG=<dir>/gvsoc_config.json`, `-sv_lib <build>/libcv32e40p_rvvi`,
   the files of `sv/gvsoc_rvvi.f` and `rvvi_trace2api` bound to the `rvviTrace` interface of the testbench.

## Compare semantics

After every retire, the bridge compares the DUT state, written by `rvviDut*Set` from the RVFI rows, with the
reference state, written only by the commit records of the model: PC, instruction, trap and debug mode, all
GPRs, all FPRs, and every compare-enabled, non-volatile CSR that the model implements and the DUT reports. The
DUT state starts from the RTL reset state (all registers zero) and the reference state from the model reset
state, so the first compare checks the reset state of the model.

Stores are compared on the data bus. The testbench reports each write accepted on the DUT data port with
`rvviDutBusWrite` (byte enables relative to the address). The bridge cuts each store of the model records into
the same word beats and compares the two sequences in program order: address, byte enables and enabled bytes.
The DUT writes the bus before it retires the store, so a beat waits on one side until the other side has it.
The testbench shuts the reference down at the end of the test, before it reads the metrics. At that point a
store of the model without its DUT beats is a mismatch. DUT beats left over are logged with the retire they
followed, since they belong to a store the DUT had not retired when the simulation stopped, which the
reference never stepped. A spurious DUT write in the middle of a run shifts the sequence and fails the compare
of the next store; only a spurious write after the last store of the run goes unseen.

The reference is never written from the DUT. The DUT only gives the model its inputs:
- the interrupt lines and the debug request, with the instants the RTL samples them and the decision points
  where its controller evaluates them (`bridge/rvviDecisionApi.h`, an additive RVVI-API extension); the model
  takes interrupts and debug entries by itself;
- the value read from a volatile CSR (`rvviRefCsrSetVolatile`, e.g. the performance counters);
- the data loaded from a volatile memory range (`rvviRefMemorySetVolatile`), an external region of the model.

State setters (`rvviRefCsrSet`, `rvviRefGprSet`, ...) are rejected, except a CSR set to the value the model
already holds at reset.
