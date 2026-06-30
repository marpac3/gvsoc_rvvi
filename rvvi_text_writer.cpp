/* ==========================================================================
 * rvvi_text_writer.cpp — RVVI-TEXT v0.4 line formatter.
 * See rvvi_text_writer.hpp for the contract. Pure stdio, no GVSOC dependency.
 * ========================================================================== */
#include "rvvi_text_writer.hpp"

void rvvi_text_write_header(FILE *fp, const RvviTextParams &p)
{
    if (!fp)
        return;

    /* VERSION must be the first line (checker requirement). "0 1" = this
     * emitter's output revision; the format itself follows RVVI-TEXT v0.4. */
    fprintf(fp, "VERSION 0 1\n");
    /* VENDOR string: quoted, no hyphen (checker STRING rule rejects '-'). */
    fprintf(fp, "VENDOR \"%s\" 0 1\n", p.vendor);
    /* PARAMS <count> then <key val> pairs — 6 pairs. */
    fprintf(fp, "PARAMS 6 ILEN %u XLEN %u FLEN %u VLEN %u NHART %u RETIRE %u\n",
            p.ilen, p.xlen, p.flen, p.vlen, p.nhart, p.retire);
}

void rvvi_text_write_line(FILE *fp, const RvviTextWriteSet &ws)
{
    if (!fp)
        return;

    fprintf(fp, "%s 0x%x 0x%x", ws.is_trap ? "TRAP" : "RET", ws.pc, ws.insn);

    /* GPR deltas: only indices flagged in gpr_mask. */
    for (int i = 0; i < 32; i++)
        if (ws.gpr_mask & (1u << i))
            fprintf(fp, " X %d 0x%x", i, ws.gpr[i]);

    /* FPR deltas: only indices flagged in fpr_mask. */
    for (int i = 0; i < 32; i++)
        if (ws.fpr_mask & (1u << i))
            fprintf(fp, " F %d 0x%x", i, ws.fpr[i]);

    /* CSR deltas: already resolved addr + value pairs. */
    for (const RvviTextCsr &c : ws.csr)
        fprintf(fp, " C 0x%x 0x%x", c.addr, c.value);

    /* Trailing MODE column (privilege mode), 0x-prefixed. Optional: callers
     * that do not track privilege mode keep the plain line format. */
    if (ws.has_mode)
        fprintf(fp, " MODE 0x%x", ws.mode);

    fputc('\n', fp);
}
