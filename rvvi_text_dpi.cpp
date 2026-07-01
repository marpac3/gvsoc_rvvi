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
}

extern "C" {

void rvviTextOpen(const char *path, uint32_t ilen, uint32_t xlen, uint32_t flen,
                  uint32_t vlen, uint32_t nhart, uint32_t retire)
{
    if (g_fp || !path)
        return;
    g_fp = fopen(path, "w");
    if (!g_fp) {
        fprintf(stderr, "[rvvi_text] cannot open %s\n", path);
        return;
    }
    RvviTextParams p{"gvsoc_rvvi", ilen, xlen, flen, vlen, nhart, retire};
    rvvi_text_write_header(g_fp, p);
    g_cur = RvviTextWriteSet{};
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
    /* Same addr can be pushed twice in one retire (e.g. a trap: the tracer's
     * generic csr_wb scan catches mstatus/mepc/mcause, then explicitly
     * re-pushes all four trap CSRs) - dedup on insert, last write wins, so the
     * emitted line carries one C token per address. */
    for (auto &c : g_cur.csr) {
        if (c.addr == addr) {
            c.value = (uint32_t)value;
            return;
        }
    }
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
}

void rvviTextClose(void)
{
    if (g_fp) {
        fclose(g_fp);
        g_fp = nullptr;
    }
}

} /* extern "C" */
