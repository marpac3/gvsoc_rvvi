// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

// SystemVerilog files of the bridge. GVSOC_BRIDGE_HOME is the root of this repository.

// RVVI (submodule)
${GVSOC_BRIDGE_HOME}/RVVI/source/host/rvvi/rvviApiPkg.sv
${GVSOC_BRIDGE_HOME}/RVVI/source/host/rvvi/rvviTrace.sv

// RVVI-API extension: decision points of asynchronous events
${GVSOC_BRIDGE_HOME}/sv/rvviDecisionApiPkg.sv

// RVVI-TRACE to RVVI-API driver
${GVSOC_BRIDGE_HOME}/sv/rvvi_trace2api.sv
