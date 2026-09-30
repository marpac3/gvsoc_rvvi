# SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Marco Paci (marco.paci@chips.it)

from typing import Annotated

import gvsoc.systree
import gvsoc.signature
from config_tree import Config, cfg_field
from gvrun.runtime import Runtime

# Line names and their interrupt numbers, index-aligned, in RVVI net order
# (MSWInterrupt, MTimerInterrupt, MExternalInterrupt, LocalInterrupt0..15).
IRQ_LINES: tuple = ('msi', 'mti', 'mei',
                    *(f'external_irq_{i}' for i in range(16, 32)))
IRQ_NUMBERS: tuple = (3, 7, 11, *range(16, 32))


class Cv32e40pExitDeviceConfig(Config):

    # Runtime: set per run (gvrun --parameter stop_on_exit=...) without
    # rebuilding the platform.
    stop_on_exit: Annotated[bool, Runtime] = cfg_field(default=True, dump=True, desc=(
        "Stop the simulation when the software reports its end. False keeps the "
        "core running after the report, like the testbench (co-simulation)"
    ))

    def __post_init__(self):
        super().__post_init__()


class Cv32e40pExitDevice(gvsoc.systree.Component):
    """UVM virtual peripheral at 0x20000000: test status and exit value."""

    def __init__(self, parent, name, config: Cv32e40pExitDeviceConfig):
        super().__init__(parent, name, config=config)
        self.add_sources(['cv32e40p_platform/exit_device.cpp'])

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'input', signature=gvsoc.signature.IoV2Sync())


class Cv32e40pStrapsConfig(Config):

    # Runtime: set per test (gvrun --parameter mtvec_addr=...) without
    # rebuilding the platform.
    mtvec_addr: Annotated[int, Runtime] = cfg_field(default=0, fmt="hex", dump=True, desc=(
        "mtvec base at boot (RTL mtvec_addr_i)"
    ))

    def __post_init__(self):
        super().__post_init__()


class Cv32e40pStraps(gvsoc.systree.Component):
    """Static configuration inputs of the core, driven at reset."""

    def __init__(self, parent, name, config: Cv32e40pStrapsConfig):
        super().__init__(parent, name, config=config)
        self.add_sources(['cv32e40p_platform/straps.cpp'])

    def o_MTVEC_ADDR(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('mtvec_addr', itf, signature='wire<uint32_t>')


class Cv32e40pSparseMem(gvsoc.systree.Component):
    """Catch-all memory: never-written bytes read 0, like the UVM testbench."""

    def __init__(self, parent, name):
        super().__init__(parent, name)
        self.add_sources(['cv32e40p_platform/sparse_mem.cpp'])

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'input', signature=gvsoc.signature.IoV2Sync())


class Cv32e40pIrqInjector(gvsoc.systree.Component):
    """Core input wires driven by an external gv:: client (gv::wire_bind)."""

    def __init__(self, parent, name):
        super().__init__(parent, name)
        self.add_sources(['cv32e40p_platform/irq_injector.cpp'])

    def o_LINE(self, name: str, itf: gvsoc.systree.SlaveItf):
        self.itf_bind(name, itf, signature='wire<bool>')
