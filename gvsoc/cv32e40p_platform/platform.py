# SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Marco Paci (marco.paci@chips.it)

"""CV32E40P co-simulation platform, shared by the cv32e40p_cosim* targets.

It instantiates the cv32e40p_testbench SoC of gvsoc-pulp, which has the memory
map and the virtual peripherals of the core-v-verif UVM testbench
(uvmt_cv32e40p), configured like an RTL configuration. In co-simulation the UVM
testbench prints and ends the test, so the model does neither.
"""

import vp.clock_domain
import gvsoc.systree
from gvrun.parameter import TargetParameter
from pulp.cv32e40p.cv32e40p_testbench import Cv32e40pTestbench
from pulp.cv32e40p.cv32e40p_testbench_config import Cv32e40pTestbenchConfig


class Cv32e40pCosim(gvsoc.systree.Component):

    def __init__(self, parent, name=None, fpu: bool=False, zfinx: bool=False,
                 corev_pulp: bool=False, corev_cluster: bool=False,
                 num_mhpmcounters: int=1):
        super().__init__(parent, name)

        binary = TargetParameter(
            self, name='binary', value=None, description='ELF binary to simulate'
        ).get_value()

        stop_on_exit = TargetParameter(
            self, name='stop_on_exit', value=True, cast=bool,
            description='Stop when the software reports its end (False for co-simulation)'
        ).get_value()

        mtvec_addr = TargetParameter(
            self, name='mtvec_addr', value=0, cast=int,
            description='mtvec base at boot (RTL mtvec_addr_i)'
        ).get_value()

        config = Cv32e40pTestbenchConfig('soc', fpu=fpu, zfinx=zfinx, corev_pulp=corev_pulp,
                                         corev_cluster=corev_cluster,
                                         num_mhpmcounters=num_mhpmcounters)
        config.tb_mem.stop_on_exit = stop_on_exit
        config.tb_mem.print_stdout = False

        # The SoC declares its own binary and mtvec_addr parameters, so pass the
        # values on.
        if binary is not None:
            self.set_parameter('soc/binary', binary)
        self.set_parameter('soc/mtvec_addr', mtvec_addr)

        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=50000000)
        soc = Cv32e40pTestbench(self, 'soc', config)
        clock.o_CLOCK(soc.i_CLOCK())
