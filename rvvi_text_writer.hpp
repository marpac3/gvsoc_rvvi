/* ==========================================================================
 * rvvi_text_writer.hpp — standalone RVVI-TEXT v0.4 line formatter
 *
 * Pure C++/stdio, ZERO GVSOC dependency. The caller (DPI bridge, SV tracer via
 * a DPI shim, or a GVSOC-only headless runner) builds a plain-data write-set
 * and this formats ONE canonical RVVI-TEXT line. The same formatter is used for
 * dut.rvvi and ref.rvvi, so the two files are byte-format-identical (only the
 * values/PC differ) and `diff dut.rvvi ref.rvvi` localises a divergence 1:1.
 *
 * Conformance target: RVVI/source/host/rvvi/rvviTextChecker.py
 *   - VERSION must be the first line.
 *   - VENDOR string must start alpha and use only [a-zA-Z0-9_ ] (NO hyphen).
 *   - hex tokens (PC / insn / CSR addr+val / MODE) are 0x-prefixed.
 * ========================================================================== */
#ifndef RVVI_TEXT_WRITER_HPP
#define RVVI_TEXT_WRITER_HPP

#include <cstdint>
#include <cstdio>
#include <vector>

/* Header parameters (PARAMS line). Emitted once per file via write_header.
 * FLEN is a parameter (not hardcoded) so the caller can derive it from the CFG
 * (0 = no FPU, 32 = single, 64 = double); VLEN 0 when there is no vector unit. */
struct RvviTextParams {
    const char *vendor;   /* e.g. "gvsoc_rvvi" — no hyphen (checker STRING rule) */
    uint32_t    ilen;     /* instruction length, bits (32) */
    uint32_t    xlen;     /* GPR width (32 or 64) */
    uint32_t    flen;     /* FPR width: 0 (no FPU), 32, 64, 128 */
    uint32_t    vlen;     /* vector width: 0 if none */
    uint32_t    nhart;    /* number of harts (1) */
    uint32_t    retire;   /* retirements per cycle (1) */
};

/* One CSR delta (already resolved addr + value) for the current retire. */
struct RvviTextCsr {
    uint32_t addr;
    uint32_t value;
};

/* Per-retire write-set: everything needed to format one RET/TRAP line. The
 * caller fills this from its own source; the formatter never reads GVSOC or DUT
 * globals directly. GPR/FPR are reported as a bitmask + value array: only the
 * indices whose mask bit is set are emitted (the architectural write-set). */
struct RvviTextWriteSet {
    uint32_t pc;
    uint32_t insn;
    bool     is_trap;          /* TRAP vs RET */

    bool     has_mode;         /* emit the trailing MODE column? */
    uint32_t mode;             /* privilege mode value (emitted 0x-prefixed) */

    uint32_t gpr_mask;         /* bit i set -> GPR i is in this write-set */
    uint32_t gpr[32];

    uint32_t fpr_mask;         /* bit i set -> FPR i in write-set (f0-f31; fpr[]
                                * holds the low 32 bits — ok for F/Zfinx, D later) */
    uint32_t fpr[32];

    std::vector<RvviTextCsr> csr;   /* CSR deltas this retire (addr + value) */
};

/* Write the 3-line RVVI-TEXT header (VERSION / VENDOR / PARAMS). Call once per
 * file, before any line. No-op if fp is null. */
void rvvi_text_write_header(FILE *fp, const RvviTextParams &p);

/* Format and write one RET/TRAP line from the write-set. No-op if fp is null.
 * Column order: PC insn, then X (GPR), F (FPR), C (CSR), then MODE (if any). */
void rvvi_text_write_line(FILE *fp, const RvviTextWriteSet &ws);

#endif /* RVVI_TEXT_WRITER_HPP */
