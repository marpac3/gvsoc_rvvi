/*
 * Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and
 *                    University of Bologna
 * Copyright (C) 2026 Fondazione Chips-it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Marco Paci, Fondazione Chips-it (marco.paci@chips.it)
 */

/*
 * GVSOC engine wrapper - compilation firewall.
 *
 * Pure-C interface between the DPI bridge (rvvi_api2gvsoc.cpp, compiled with
 * Questa's svdpi.h) and the GVSOC engine internals (gv::Gvsoc, IssWrapper).
 * The implementation in gvsoc_engine.cpp includes the heavy GVSOC/ISS
 * headers; rvvi_api2gvsoc.cpp only includes this lightweight header, so the two
 * sets of headers never meet in the same translation unit.
 */

#ifndef GVSOC_ENGINE_HPP
#define GVSOC_ENGINE_HPP

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Lifecycle ---- */

/**
 * Instantiate and start the GVSOC engine in synchronous mode.
 *
 * @param config_path  Path to gvsoc_config.json (with the ELF injected).
 * @return 0 on success, non-zero on failure.
 */
int gvsoc_engine_init(const char *config_path);

/** Shut down the GVSOC engine and release resources. */
void gvsoc_engine_shutdown(void);

/** @return true if the engine is initialized and running. */
bool gvsoc_engine_is_running(void);

/* ---- Stepping ---- */

/**
 * Step GVSOC until exactly one instruction retires (or the simulation ends).
 *
 * Steps one clock at a time and detects a retire from a change in
 * exec.current_insn (the ISS PC): GVSOC's instret does not auto-increment, so
 * the PC transition is the retire signal. A zero-offset branch-to-self is
 * recognised via its branch-penalty stall so it is not mistaken for a stall.
 *
 * @return 1 if an instruction retired, 0 if the simulation ended or timed out.
 */
int gvsoc_engine_step(void);

/** @return true once the simulation has ended (exit device triggered). */
bool gvsoc_engine_finished(void);

/**
 * Runaway detector (DPI co-sim).
 *
 * @return true once the ISS is detected as permanently diverged and stuck (a
 *         run of consecutive cycle-budget-exhausting step timeouts with the
 *         ISS PC unchanged). Sticky once latched; reset only by
 *         gvsoc_engine_init(). Never asserts on a passing test (a clean retire
 *         resets the underlying counter) nor on WFI.
 */
bool gvsoc_engine_is_runaway(void);

/**
 * Commits still queued after the last gvsoc_engine_step().
 *
 * The v2 engine retires from a deferred-commit stream: on a multi-commit
 * burst the ISS register/CSR state has already advanced to the NEWEST
 * queued commit, so while this returns > 0 a state compare against the
 * retire just served would read future values. 0 = state is exact for the
 * served retire. The v1 engine steps per instruction and always returns 0.
 */
uint64_t gvsoc_engine_pending_commits(void);

/**
 * 1 when retires are served from an architectural commit stream, 0 for
 * per-instruction stepping.
 *
 * On a commit-stream engine (v2) a refused instruction (illegal) never
 * commits, so its DUT trap row has no ISS step to consume: stepping there
 * would eat the first handler commit and shift the comparison by one
 * retire per trap. An instruction that executes and then traps
 * (ecall/ebreak) does commit — materialize_commit tells the two apart by
 * PC. A per-instruction engine (v1) always consumes the faulting step.
 */
int gvsoc_engine_commit_stream(void);

/**
 * 1 when the ISS architectural state matches the last served retire, 0 when
 * a trap redirect happened after that instruction executed (it was still
 * held in the commit FIFO behind an in-flight memory op when the next
 * instruction trapped): the sampled CSR/GPR state then already includes
 * the trap entry, and state compares on this retire must be skipped. The
 * comparison re-arms on the first commit stamped after the redirect. The
 * v1 engine steps per instruction and always returns 1.
 */
int gvsoc_engine_state_current(void);

/**
 * Ensure at least one commit is queued (advancing the engine if needed) and
 * return its PC WITHOUT consuming it.  0 = *pc valid, -1 = timeout/finished
 * or per-instruction engine.
 *
 * Used on DUT trap rows to decide whether the row has a commit to consume:
 * an instruction that executes and then traps (ecall/ebreak) commits, so its
 * PC equals the trapped PC; a refused instruction (illegal) never commits and
 * the queued head is already the handler entry.
 */
int gvsoc_engine_materialize_commit(uint32_t *pc);

/* ---- State query: PC and instruction ---- */

uint32_t gvsoc_engine_get_pc(void);
uint32_t gvsoc_engine_get_insn(void);

/**
 * Force the ISS program counter.
 *
 * Sets current_insn to the given value and flushes the instruction prefetcher
 * so the next fetch comes from the new address (it also clears any residual
 * pipeline stall and a pending WFI). Used by the force-resync path to align
 * the ISS PC with the DUT after an IRQ trap.
 */
void gvsoc_engine_set_pc(uint32_t pc);

/**
 * Enable/disable ISS-side IRQ checking.
 *
 * In DPI mode the DUT drives interrupt flow. Setting skip=true prevents the
 * ISS from taking interrupts independently, avoiding the one-retire IRQ timing
 * desync between ISS and DUT. Traps are handled by the force-resync path
 * (or by informed injection) instead.
 */
void gvsoc_engine_skip_irq(bool skip);

/** @return true if the ISS is in WFI (stalled waiting for an interrupt). */
bool gvsoc_engine_is_wfi(void);

/* ---- State query: registers ---- */

/** Read a GPR. Index 0-31 (x0 always returns 0). */
uint32_t gvsoc_engine_get_gpr(uint32_t index);

/** Read an FPR. Index 0-31 (shares the GPR file under ZFINX). */
uint32_t gvsoc_engine_get_fpr(uint32_t index);

/* ---- State query: CSRs ---- */

/**
 * Read a CSR by 12-bit address (e.g. 0x300 = mstatus).
 *
 * Reads the value through the public struct member and reproduces the read
 * fixups the ISS would apply (mstatus SD/MPP, the M-mode read-only zeros).
 *
 * @param csr_addr  CSR address.
 * @param value     Output: CSR value.
 * @return 1 if the CSR is mapped, 0 otherwise.
 */
int gvsoc_engine_get_csr(uint32_t csr_addr, uint32_t *value);

/* ---- State injection: registers, CSR, and IRQ ---- */

/**
 * Write a GPR into the ISS (direct regfile write).
 *
 * @param index  GPR index (0-31). x0 writes are ignored.
 * @param value  Value to write.
 */
void gvsoc_engine_set_gpr(uint32_t index, uint32_t value);

/**
 * Write an FPR into the ISS (direct write; shares the GPR file under ZFINX).
 *
 * @param index  FPR index (0-31).
 * @param value  Value to write.
 */
void gvsoc_engine_set_fpr(uint32_t index, uint32_t value);

/**
 * Write a CSR into the ISS (direct struct write).
 *
 * Used by rvviRefCsrSet() to inject DUT CSR values (e.g. mtvec during
 * ref_init). For mstatus it applies the CV32E40P write mask; other CSRs are
 * written verbatim. Only CSRs present in the internal map are written.
 *
 * @param csr_addr  12-bit CSR address (e.g. 0x305 = mtvec).
 * @param value     Value to write.
 * @return 1 if the CSR was found and written, 0 otherwise.
 */
int gvsoc_engine_set_csr(uint32_t csr_addr, uint32_t value);

/**
 * Set or clear an IRQ line in the ISS.
 *
 * Maps RVVI net indices (assigned by init_net_map in rvvi_api2gvsoc.cpp) to mip
 * bit positions:
 *   0     = MSWInterrupt       -> mip bit 3
 *   1     = MTimerInterrupt    -> mip bit 7
 *   2     = MExternalInterrupt -> mip bit 11
 *   3..18 = LocalInterrupt0..15 -> mip bits 16..31
 *   19    = haltreq            -> debug request (not an IRQ, handled apart)
 *
 * Delivery is engine-specific: the v1 engine writes mip directly; the v2
 * engine drives the platform irq-injector wires (gv::wire_bind), so mip and
 * the WFI wake-up follow the model's own interrupt path.
 *
 * @param net_index  RVVI net index.
 * @param value      1 = assert, 0 = deassert.
 */
void gvsoc_engine_set_irq(uint64_t net_index, int value);

/**
 * IRQ settle - run a few drain cycles after an mip assertion so the pre-fetch
 * IRQ guard can fire before the next gvsoc_engine_step(). Call right after
 * gvsoc_engine_set_irq() for an assert. A no-op when no settle is pending.
 */
void gvsoc_engine_settle_irq(void);

/**
 * Informed IRQ injection (OVPSim-style "deferint" oracle handoff).
 *
 * Tell the ISS to TAKE the interrupt with mcause exception code @p
 * mcause_irq_id now, and let Irq::check() COMPUTE the trap entry itself
 * (mepc = interrupted PC, mstatus.MIE->MPIE, mcause = (1<<31)|id,
 * current_insn = the trap-vector entry, vectored for CV32E40P). No CSR/GPR/FPR
 * is copied from the DUT - the ISS is the calculator and the bridge compares
 * the result in the normal step-n-compare.
 *
 * Lowers exec.skip_irq_check (and the engine-level g_dpi_skip_irq re-assert)
 * for exactly one gvsoc_engine_step(), then restores both. Used only on the
 * informed-injection path (plusarg-gated in the SV bridge).
 *
 * @param mcause_irq_id  interrupt cause id (0..31), i.e. mcause[30:0].
 * @return 1 if a retire occurred (handler entry taken), 0 otherwise.
 */
int gvsoc_engine_take_irq_for_one_step(int mcause_irq_id);

#ifdef __cplusplus
}
#endif

#endif /* GVSOC_ENGINE_HPP */
