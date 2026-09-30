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
 *  The reference takes interrupts and debug requests by itself. The DUT tells it when
 *  its registers sample the input nets (rvviRefNetGroupSample()) and when its controller
 *  checks for interrupts and debug (rvviRefDecisionPoint()).
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
 *  The decisions then use the values set so far with rvviRefNetSet().
 *
 *  \param group The net group, as placed with rvviRefNetGroupSet().
 *
 *  \return RVVI_TRUE if the reference accepted the sample else RVVI_FALSE.
**/
extern bool_t rvviRefNetGroupSample(
    uint32_t group);

/*! \brief Notify the reference of a DUT decision point for asynchronous events.
 *
 *  The DUT decided on interrupts and debug before issuing the instruction with RVFI
 *  order `order`. The reference decides at the same point.
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
