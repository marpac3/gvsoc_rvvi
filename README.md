# gvsoc_rvvi: CV32E40P reference model for RISC-V co-simulation, on GVSOC

This repository compares a CV32E40P RTL simulation in lock step with the CV32E40P model of
[GVSOC](https://github.com/gvsoc/gvsoc), through the
[RVVI](https://github.com/riscv-verification/RVVI) API.

| directory | content |
|---|---|
| `gvsoc/` | GVSOC module: `cv32e40p_cosim*` targets on the memory map of the core-v-verif testbench |
| `bridge/` | `libcv32e40p_rvvi.so`: the RVVI-API on the co-simulation interface of the core model |
| `sv/` | `rvvi_trace2api.sv`: from an RVVI-TRACE interface to RVVI-API calls, one step-and-compare per retire |
| `test/` | `cosim_run`: runs a target through the interface alone and prints the commit records |
| `RVVI/` | RVVI headers and SystemVerilog packages (submodule) |

## Build

```
git submodule update --init
cd <gvsoc> && make build TARGETS="cv32e40p_cosim;cv32e40p_cosim_mhpm29;cv32e40p_cosim_pulp;cv32e40p_cosim_pulp_fpu;cv32e40p_cosim_pulp_fpu_zfinx" \
    MODULES=<this repository>/gvsoc INSTALLDIR=<install>
make GVSOC_HOME=<gvsoc> GVSOC_INSTALL=<install>
```

## Use

1. For each test program, write the platform configuration:
   `gvrun --target-dir=<this repository>/gvsoc --target=<target> --parameter binary=<elf> --work-dir=<dir> prepare`,
   with `<target>` one of `cv32e40p_cosim[_pulp[_fpu[_zfinx]]]` or `cv32e40p_cosim_mhpm29` (NUM_MHPMCOUNTERS=29)
2. Run the simulator with `GVSOC_CONFIG=<dir>/gvsoc_config.json`, `-sv_lib <build>/libcv32e40p_rvvi`,
   the RVVI packages and `sv/rvvi_trace2api.sv` bound to the `rvviTrace` interface of the testbench.

## Compare semantics

The DUT state (written by `rvviDut*Set` from the RVFI rows) and the reference state (written only by the
commit records of the model) start from the model reset state and are compared after every retire:
PC, instruction, trap and debug mode, all GPRs, all FPRs, and every compare-enabled, non-volatile CSR
implemented by the model and reported by the DUT. The reference is never written from the DUT; the only DUT-to-model inputs are the
interrupt lines and the debug request. State setters (`rvviRefCsrSet`, `rvviRefGprSet`, ...) are
rejected, except a CSR set to the value the model already holds at reset.
