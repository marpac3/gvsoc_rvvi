/*
 * SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Authors: Marco Paci (marco.paci@chips.it)
 */

#pragma once

/*! \file rvviDecisionApi.h
 *  \brief RVVI-API extension: decision points of asynchronous events.
 *
 *  With the base RVVI-API a testbench delivers interrupt and debug-request nets with
 *  rvviRefNetSet(), and the reference either takes an event as soon as it sees the net,
 *  or waits for the DUT to report that it took one. The first is too early whenever the
 *  DUT samples the net later, or decides only at some pipeline states; the second lets
 *  the DUT choose for the reference.
 *
 *  This extension adds the two facts the reference misses:
 *  - when the DUT sampled a group of nets (rvviRefNetGroupSample());
 *  - where the DUT evaluated its asynchronous events (rvviRefDecisionPoint()), named by
 *    the RVFI order of the instruction it was about to issue.
 *
 *  The reference then decides by itself, at the same points and with the same sampled
 *  values, whether it takes an interrupt, enters debug mode or wakes up; the DUT never
 *  tells it what it decided. Both calls follow the rvviRefNetSet() calls in the order
 *  the DUT saw the events.
 *
 *  The net groups, which value a decision uses (sampled or current), and the kinds of
 *  decision point a DUT reports are documented with the reference model, like the net
 *  names of rvviRefNetIndexGet().
**/

#include "rvviApi.h"

#define RVVI_DECISION_API_VERSION 1

typedef enum {
    RVVI_DECISION_DISPATCH = 0,     /*!< before issuing the instruction `order` */
    RVVI_DECISION_FIRST_FETCH = 1,  /*!< before the first instruction after reset or a wake-up */
    RVVI_DECISION_SLEEP = 2,        /*!< while the hart waits for an event (WFI) */
    RVVI_DECISION_BOOT = 3,         /*!< out of reset, before fetching */
} rvviDecisionE;

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Notify the reference that the DUT sampled the nets of a group.
 *
 *  The values set with rvviRefNetSet() so far become the values the DUT's decision logic
 *  sees for the nets of this group, until the next sample of the group.
 *
 *  \param group The net group, as placed with rvviRefNetGroupSet().
 *
 *  \return RVVI_TRUE if the reference accepted the sample else RVVI_FALSE.
**/
extern bool_t rvviRefNetGroupSample(
    uint32_t group);

/*! \brief Notify the reference of a DUT decision point for asynchronous events.
 *
 *  The DUT evaluated its interrupts and debug requests here, before issuing the
 *  instruction that retires with RVFI order `order`. The reference takes its own
 *  decision at the same point, from its own state and the sampled nets. A DUT can
 *  report several points before the same instruction; each one is decided in turn.
 *
 *  \param hartId The hart that made the decision.
 *  \param kind The kind of decision point (rvviDecisionE).
 *  \param order RVFI order of the next instruction the hart retires.
 *
 *  \return RVVI_TRUE if the reference accepted the point, RVVI_FALSE if it is not valid
 *          (for example, the reference has already executed that instruction).
**/
extern bool_t rvviRefDecisionPoint(
    uint32_t hartId,
    uint32_t kind,
    uint64_t order);

#ifdef __cplusplus
}  // extern "C"
#endif
