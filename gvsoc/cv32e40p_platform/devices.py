# SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Marco Paci (marco.paci@chips.it)

import gvsoc.systree
import gvsoc.signature

# Line names and their interrupt numbers, index-aligned, in RVVI net order
# (MSWInterrupt, MTimerInterrupt, MExternalInterrupt, LocalInterrupt0..15).
IRQ_LINES: tuple = ('msi', 'mti', 'mei',
                    *(f'external_irq_{i}' for i in range(16, 32)))
IRQ_NUMBERS: tuple = (3, 7, 11, *range(16, 32))


class Cv32e40pExitDevice(gvsoc.systree.Component):
    """UVM virtual peripheral at 0x20000000: test status and exit value."""

    def __init__(self, parent, name):
        super().__init__(parent, name)
        self.add_sources(['cv32e40p_platform/exit_device.cpp'])

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'input', signature=gvsoc.signature.IoV2Sync())


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
