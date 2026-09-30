# SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Marco Paci (marco.paci@chips.it)

import gvsoc.runner
from cv32e40p_platform.platform import Cv32e40pCosim


class Model(Cv32e40pCosim):

    def __init__(self, parent, name=None):
        super().__init__(parent, name)


class Target(gvsoc.runner.Target):

    description = "CV32E40P co-simulation platform"
    model = Model
    name = "cv32e40p_cosim"
