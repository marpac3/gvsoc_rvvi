# gvsoc_rvvi: CV32E40P reference model for RISC-V co-simulation, on GVSOC

This repository compares a CV32E40P RTL simulation in lock step with the CV32E40P model of
[GVSOC](https://github.com/gvsoc/gvsoc), through the
[RVVI](https://github.com/riscv-verification/RVVI) API.
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) explains how it works.

| directory | content |
|---|---|
| `gvsoc/` | GVSOC module with the `cv32e40p_cosim*` targets. They instantiate the `cv32e40p_testbench` SoC of gvsoc-pulp, which has the memory map and virtual peripherals of the core-v-verif testbench, configured for co-simulation. |
| `bridge/` | `libcv32e40p_rvvi.so`, the RVVI-API implemented on the co-simulation interface of the core model. |
| `sv/` | `rvvi_trace2api.sv` turns an RVVI-TRACE interface into RVVI-API calls, with one step and compare per retire. `rvviDecisionApiPkg.sv` is the SystemVerilog package of the decision-point extension, and `gvsoc_rvvi.f` is the file list for the simulator, with `GVSOC_BRIDGE_HOME` set to this repository. |
| `test/` | `cosim_run` runs a target through the interface alone and prints the commit records. |
| `docs/` | `ARCHITECTURE.md`, the architecture of the bridge and how a co-simulation runs. |
| `RVVI/` | RVVI headers and SystemVerilog packages (submodule) |

## Build

GVSOC is not part of this repository, and must be installed on the machine. Clone the
[GVSOC repository](https://github.com/gvsoc/gvsoc) and prepare it as its README says, with its submodules and
Python requirements. Its gvsoc-pulp submodule must have the CV32E40P model (`pulp/cv32e40p/`).

In the commands below `<this repository>` is the path of this repository, `<gvsoc>` the path of the GVSOC clone
and `<install>` the directory where GVSOC is installed. The second command runs the build of GVSOC in its clone:
it compiles GVSOC with the targets of `gvsoc/` and installs it in `<install>`. The third one builds the bridge
(`libcv32e40p_rvvi.so`) and `cosim_run` in this repository, in `build/`.

```
cd <this repository> && git submodule update --init
cd <gvsoc> && make build MODULES=<this repository>/gvsoc INSTALLDIR=<install> \
    TARGETS="cv32e40p_cosim;cv32e40p_cosim_mhpm29;cv32e40p_cosim_pulp;cv32e40p_cosim_pulp_fpu;cv32e40p_cosim_pulp_fpu_zfinx;cv32e40p_cosim_pulp_cluster;cv32e40p_cosim_pulp_cluster_fpu;cv32e40p_cosim_pulp_cluster_fpu_zfinx"
cd <this repository> && make GVSOC_HOME=<gvsoc> GVSOC_INSTALL=<install>
```

## Use

1. For each test program, write the platform configuration:
   `<install>/bin/gvrun --target-dir=<this repository>/gvsoc --target=<target> --parameter binary=<elf> --work-dir=<dir> prepare`,
   with `<target>` one of `cv32e40p_cosim[_pulp[_cluster][_fpu[_zfinx]]]` or `cv32e40p_cosim_mhpm29`
   (NUM_MHPMCOUNTERS=29)
2. Run the simulator with `GVSOC_CONFIG=<dir>/gvsoc_config.json`, `-sv_lib <this repository>/build/libcv32e40p_rvvi`,
   the files of `sv/gvsoc_rvvi.f` and `rvvi_trace2api` bound to the `rvviTrace` interface of the testbench.

## Compare semantics

After every retire, the bridge compares the DUT state, which the testbench writes with `rvviDut*Set`, with the
reference state, written only by the commit records of the model: PC, instruction, trap and debug mode, all
GPRs, all FPRs, and every compare-enabled, non-volatile CSR that the model implements and the DUT reports. The
DUT state starts from the RTL reset state (all registers zero) and the reference state from the model reset
state, so the first compare checks the reset state of the model.

The reference is never written from the DUT. The DUT only gives the model its inputs:
- the interrupt lines and the debug request, with the instants the RTL samples them and the decision points
  where its controller evaluates them (`bridge/rvviDecisionApi.h`, an additive RVVI-API extension). The model
  takes interrupts and debug entries by itself.
- the value read from a volatile CSR (`rvviRefCsrSetVolatile`, such as the performance counters).
- the data loaded from a volatile memory range (`rvviRefMemorySetVolatile`), an external region of the model.

State setters (`rvviRefCsrSet`, `rvviRefGprSet`, ...) are rejected, except a CSR set to the value the model
already holds at reset.
