// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

// RVVI-API extension: decision points of asynchronous events (bridge/rvviDecisionApi.h).

`ifndef _RVVI_DECISION_API_PKG__
`define _RVVI_DECISION_API_PKG__

package rvviDecisionApiPkg;

parameter RVVI_DECISION_API_VERSION = 1;

typedef enum {
    RVVI_DECISION_DISPATCH = 0,
    RVVI_DECISION_FIRST_FETCH = 1,
    RVVI_DECISION_SLEEP = 2,
    RVVI_DECISION_BOOT = 3
} rvviDecisionE;

import "DPI-C" context function byte rvviRefNetGroupSample(
    input int group);

import "DPI-C" context function byte rvviRefDecisionPoint(
    input int hartId,
    input int kind,
    input longint order);

endpackage

`endif
