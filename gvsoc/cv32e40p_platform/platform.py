# SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Marco Paci (marco.paci@chips.it)

"""CV32E40P co-simulation platform: the memory map of the core-v-verif UVM
testbench (uvmt_cv32e40p), shared by the cv32e40p_cosim* targets.

    0x00000000  4MB   Main RAM        (boot address 0x80)
    0x10000000  256B  Virtual STDOUT  (write-only sink)
    0x15000000  256B  Virtual TIMER   (write-only sink)
    0x1A110800  4KB   Debug ROM       (linker script `dbg` region)
    0x20000000  256B  Virtual EXIT    (terminates the simulation)
    elsewhere         sparse memory   (reads 0 until written)
"""

import vp.clock_domain
import gvsoc.systree
from gvrun.parameter import TargetParameter
from config_tree import Config, cfg_field
from memory.memory_v3 import Memory, MemoryV3Config
from interco.router_v2 import Router, RouterConfig, RouterMapping
from utils.loader.loader_v2 import ElfLoader
from pulp.cpu.iss.cv32e40p import Cv32e40p
from pulp.cpu.iss.cv32e40p_config import Cv32e40pConfig
from cv32e40p_platform.devices import (Cv32e40pExitDevice, Cv32e40pExitDeviceConfig,
                                    Cv32e40pStraps, Cv32e40pStrapsConfig, Cv32e40pSparseMem,
                                    Cv32e40pIrqInjector, IRQ_LINES, IRQ_NUMBERS)


class Cv32e40pCosimConfig(Config):

    boot_addr: int = cfg_field(default=0x80, fmt="hex", dump=True, desc=(
        "Boot address (RTL boot_addr_i)"
    ))

    ram_latency: int = cfg_field(default=0, dump=True, desc=(
        "Main RAM response latency in cycles"
    ))

    fpu: bool = cfg_field(default=False, dump=True, desc=(
        "RTL FPU parameter"
    ))

    zfinx: bool = cfg_field(default=False, dump=True, desc=(
        "RTL ZFINX parameter"
    ))

    corev_pulp: bool = cfg_field(default=False, dump=True, desc=(
        "RTL COREV_PULP parameter"
    ))

    core: Cv32e40pConfig = cfg_field(init=False, desc=(
        "CV32E40P core configuration"
    ))

    mem: MemoryV3Config = cfg_field(init=False, desc=(
        "Main RAM configuration"
    ))

    stdout: MemoryV3Config = cfg_field(init=False, desc=(
        "Virtual STDOUT sink configuration"
    ))

    timer: MemoryV3Config = cfg_field(init=False, desc=(
        "Virtual TIMER sink configuration"
    ))

    debug_rom: MemoryV3Config = cfg_field(init=False, desc=(
        "Debug ROM configuration"
    ))

    exit: Cv32e40pExitDeviceConfig = cfg_field(init=False, desc=(
        "Virtual EXIT device configuration"
    ))

    straps: Cv32e40pStrapsConfig = cfg_field(init=False, desc=(
        "Static configuration inputs of the core"
    ))

    router: RouterConfig = cfg_field(init=False, desc=(
        "Router configuration"
    ))

    mem_mapping: RouterMapping = cfg_field(init=False, desc=(
        "Main RAM address range"
    ))

    stdout_mapping: RouterMapping = cfg_field(init=False, desc=(
        "Virtual STDOUT address range"
    ))

    timer_mapping: RouterMapping = cfg_field(init=False, desc=(
        "Virtual TIMER address range"
    ))

    debug_rom_mapping: RouterMapping = cfg_field(init=False, desc=(
        "Debug ROM address range"
    ))

    exit_mapping: RouterMapping = cfg_field(init=False, desc=(
        "Virtual EXIT device address range"
    ))

    background_mapping: RouterMapping = cfg_field(init=False, desc=(
        "Sparse memory catch-all route"
    ))

    def __post_init__(self):
        super().__post_init__()
        if self.ram_latency < 0:
            raise ValueError('ram_latency must be nonnegative')
        # ZFINX keeps the F opcodes in the decoder (on the integer register
        # file); the core recipe disables the FP loads, stores and moves.
        isa = 'rv32imfc' if (self.fpu or self.zfinx) else 'rv32imc'
        self.core = Cv32e40pConfig(isa=isa, zfinx=self.zfinx, corev_pulp=self.corev_pulp,
                                   boot_addr=self.boot_addr)
        # init=False: never-written bytes read 0, like the testbench memory.
        self.mem       = MemoryV3Config('mem', size=0x0040_0000, atomics=False,
                                        latency=self.ram_latency, init=False)
        self.stdout    = MemoryV3Config('stdout', size=0x100, atomics=False, latency=0,
                                        init=False)
        self.timer     = MemoryV3Config('timer', size=0x100, atomics=False, latency=0,
                                        init=False)
        self.debug_rom = MemoryV3Config('debug_rom', size=0x1000, atomics=False, latency=0,
                                        init=False)
        self.exit = Cv32e40pExitDeviceConfig('exit')
        self.straps = Cv32e40pStrapsConfig('straps')
        self.router = RouterConfig(kind='bandwidth')
        self.mem_mapping       = RouterMapping(name='mem_mapping',
                                               base=0x0000_0000, size=0x0040_0000)
        self.stdout_mapping    = RouterMapping(name='stdout_mapping',
                                               base=0x1000_0000, size=0x100)
        self.timer_mapping     = RouterMapping(name='timer_mapping',
                                               base=0x1500_0000, size=0x100)
        self.debug_rom_mapping = RouterMapping(name='debug_rom_mapping',
                                               base=0x1A11_0800, size=0x1000)
        self.exit_mapping      = RouterMapping(name='exit_mapping',
                                               base=0x2000_0000, size=0x100)
        # Catch-all (size=0): absolute addresses are forwarded so the sparse
        # memory is indexed like the testbench one.
        self.background_mapping = RouterMapping(name='background_mapping',
                                                base=0x0000_0000, size=0,
                                                remove_base=False)


class Cv32e40pCosimSoc(gvsoc.systree.Component):

    def __init__(self, parent, name, config: Cv32e40pCosimConfig, binary):
        super().__init__(parent, name, config=config)

        mem    = Memory             ( self, 'mem'           , config=config.mem       )
        stdout = Memory             ( self, 'stdout'        , config=config.stdout    )
        timer  = Memory             ( self, 'timer'         , config=config.timer     )
        dbgrom = Memory             ( self, 'debug_rom'     , config=config.debug_rom )
        exit_d = Cv32e40pExitDevice ( self, 'exit'          , config=config.exit      )
        bg_mem = Cv32e40pSparseMem  ( self, 'background_mem'                          )
        ico    = Router             ( self, 'ico'           , config=config.router    )
        core   = Cv32e40p           ( self, 'core'          , config=config.core      )
        loader = ElfLoader          ( self, 'loader'        , binary=binary           )

        ico.o_MAP ( mem.i_INPUT()   , mapping=config.mem_mapping        )
        ico.o_MAP ( stdout.i_INPUT(), mapping=config.stdout_mapping     )
        ico.o_MAP ( timer.i_INPUT() , mapping=config.timer_mapping      )
        ico.o_MAP ( dbgrom.i_INPUT(), mapping=config.debug_rom_mapping  )
        ico.o_MAP ( exit_d.i_INPUT(), mapping=config.exit_mapping       )
        ico.o_MAP ( bg_mem.i_INPUT(), mapping=config.background_mapping )

        # o_ENTRY is not bound: like the RTL, the core boots at boot_addr,
        # not at the ELF entry.
        loader.o_OUT   ( ico.i_INPUT(0)   )
        loader.o_START ( core.i_FETCHEN() )

        core.o_FETCH ( ico.i_INPUT(1) )
        core.o_DATA  ( ico.i_INPUT(2) )

        # Core input wires, driven by the co-simulation client.
        irq_inj = Cv32e40pIrqInjector(self, 'irq_injector')
        for name, irq in zip(IRQ_LINES, IRQ_NUMBERS):
            irq_inj.o_LINE(name, core.i_IRQ(irq))
        irq_inj.o_LINE('haltreq', gvsoc.systree.SlaveItf(
            core, itf_name='haltreq', signature='wire<bool>'))

        straps = Cv32e40pStraps(self, 'straps', config=config.straps)
        straps.o_MTVEC_ADDR(gvsoc.systree.SlaveItf(
            core, itf_name='mtvec_addr', signature='wire<uint32_t>'))


class Cv32e40pCosim(gvsoc.systree.Component):

    def __init__(self, parent, name=None, fpu: bool=False, zfinx: bool=False,
                 corev_pulp: bool=False):
        super().__init__(parent, name)

        binary = TargetParameter(
            self, name='binary', value=None, description='ELF binary to simulate'
        ).get_value()

        ram_latency = TargetParameter(
            self, name='ram_latency', value=0, cast=int,
            description='Main RAM response latency in cycles'
        ).get_value()

        stop_on_exit = TargetParameter(
            self, name='stop_on_exit', value=True, cast=bool,
            description='Stop when the software reports its end (False for co-simulation)'
        ).get_value()

        mtvec_addr = TargetParameter(
            self, name='mtvec_addr', value=0, cast=int,
            description='mtvec base at boot (RTL mtvec_addr_i)'
        ).get_value()

        config = Cv32e40pCosimConfig('soc', fpu=fpu, zfinx=zfinx, corev_pulp=corev_pulp,
                                     ram_latency=ram_latency)
        config.exit.stop_on_exit = stop_on_exit
        config.straps.mtvec_addr = mtvec_addr

        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=50000000)
        soc = Cv32e40pCosimSoc(self, 'soc', config, binary)
        clock.o_CLOCK(soc.i_CLOCK())
