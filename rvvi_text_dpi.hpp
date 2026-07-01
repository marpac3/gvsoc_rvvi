/* ==========================================================================
 * rvvi_text_dpi.hpp — C/DPI entry points for the RVVI-TEXT formatter
 *
 * Used by a SystemVerilog tracer (uvmt_cv32e40p_rvvi_text_tracer) to emit
 * dut.rvvi directly from the DUT in an RTL-only sim — NO GVSOC, NO ISS.
 *
 * The SV side accumulates one retire's write-set via the rvviTextSet* calls,
 * then rvviTextWrite emits one RET/TRAP line and clears the set.  SV imports
 * these via `import "DPI-C"`; C++ callers (the unit test) include this header.
 * ========================================================================== */
#ifndef RVVI_TEXT_DPI_HPP
#define RVVI_TEXT_DPI_HPP

#include <cstdint>

extern "C" {

/* Open the trace file and write the 3-line header.  flen/vlen come from the
 * CFG (flen 0 = no FPU).  Returns 1 on success (also when already open), 0
 * when path is null or the file cannot be created -- the writer then stays
 * disabled and every later call is a safe no-op, so the caller must raise
 * the error itself (a silently missing trace looks like a passing run). */
int rvviTextOpen(const char *path, uint32_t ilen, uint32_t xlen, uint32_t flen,
                 uint32_t vlen, uint32_t nhart, uint32_t retire);

/* Accumulate one written register / CSR / privilege mode for the current retire.
 * value is uint64_t for DPI compatibility (SV longint unsigned); the high 32 bits
 * are discarded -- correct for XLEN=32 (CV32E40P) and FLEN=32 (F/Zfinx, not D). */
void rvviTextSetGpr(uint32_t idx, uint64_t value);
void rvviTextSetFpr(uint32_t idx, uint64_t value);
void rvviTextSetCsr(uint32_t addr, uint64_t value);
void rvviTextSetMode(uint32_t mode);

/* Emit one line (RET, or TRAP when isTrap != 0) and reset the write-set. */
void rvviTextWrite(uint64_t pc, uint64_t insn, uint8_t isTrap);

/* Flush and close the trace file. */
void rvviTextClose(void);

} /* extern "C" */

#endif /* RVVI_TEXT_DPI_HPP */
