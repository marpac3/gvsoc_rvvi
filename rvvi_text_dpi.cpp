/* ==========================================================================
 * rvvi_text_dpi.cpp — DPI shim exposing the RVVI-TEXT formatter
 * to a SystemVerilog tracer, for RTL-only trace generation (no GVSOC/ISS).
 *
 * Builds into librvvi_text.so together with rvvi_text_writer.o — ZERO GVSOC
 * dependency, so an RTL-only sim links it without the embedded engine.  All
 * state is a single FILE* plus the write-set accumulated for the current retire.
 *
 * No C++ exception may cross the DPI boundary back into the simulator — the
 * only allocating call (CSR push_back) is contained.
 * ========================================================================== */
#include "rvvi_text_dpi.hpp"
#include "rvvi_text_writer.hpp"

#include <cstdint>
#include <cstdio>

namespace {
    FILE             *g_fp = nullptr;   /* dut.rvvi, owned here */
    RvviTextWriteSet  g_cur;            /* write-set accumulated for this retire */
    uint64_t          g_lines = 0;      /* emitted lines, for the periodic flush */
}

extern "C" {

int rvviTextOpen(const char *path, uint32_t ilen, uint32_t xlen, uint32_t flen,
                 uint32_t vlen, uint32_t nhart, uint32_t retire)
{
    if (g_fp)
        return 1;
    if (!path)
        return 0;
    g_fp = fopen(path, "w");
    if (!g_fp) {
        fprintf(stderr, "[rvvi_text] cannot open %s\n", path);
        return 0;
    }
    RvviTextParams p{"gvsoc_rvvi", ilen, xlen, flen, vlen, nhart, retire};
    rvvi_text_write_header(g_fp, p);
    g_cur   = RvviTextWriteSet{};
    g_lines = 0;
    return 1;
}

void rvviTextSetGpr(uint32_t idx, uint64_t value)
{
    if (idx < 32) {
        g_cur.gpr[idx]  = (uint32_t)value;
        g_cur.gpr_mask |= (1u << idx);
    }
}

void rvviTextSetFpr(uint32_t idx, uint64_t value)
{
    if (idx < 32) {
        g_cur.fpr[idx]  = (uint32_t)value;
        g_cur.fpr_mask |= (1u << idx);
    }
}

void rvviTextSetCsr(uint32_t addr, uint64_t value)
{
    /* Duplicate pushes of one address are fine (e.g. a trap: the tracer's
     * generic csr_wb scan and its explicit trap-CSR push both report
     * mstatus/mepc/mcause): the formatter emits one C token per address. */
    try {
        g_cur.csr.push_back({addr, (uint32_t)value});
    } catch (...) {
        /* Allocation failed: drop this CSR rather than cross the DPI boundary. */
    }
}

void rvviTextSetMode(uint32_t mode)
{
    g_cur.has_mode = true;
    g_cur.mode     = mode;
}

void rvviTextWrite(uint64_t pc, uint64_t insn, uint8_t isTrap)
{
    if (!g_fp) {
        g_cur = RvviTextWriteSet{};   /* drop accumulated set; file not open */
        return;
    }
    g_cur.pc      = (uint32_t)pc;
    g_cur.insn    = (uint32_t)insn;
    g_cur.is_trap = (isTrap != 0);
    rvvi_text_write_line(g_fp, g_cur);
    g_cur = RvviTextWriteSet{};   /* reset for the next retire */
    /* Periodic flush, same 1000-retire cadence as the DPI bridge: the final
     * block does not run when the simulation is killed, and an unflushed
     * tail is lost exactly where the trace matters most. */
    if ((++g_lines % 1000) == 0)
        fflush(g_fp);
}

void rvviTextClose(void)
{
    if (g_fp) {
        fclose(g_fp);
        g_fp = nullptr;
    }
}

} /* extern "C" */
