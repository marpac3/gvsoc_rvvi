/* Deterministic unit test of the RVVI-TEXT formatter.
 * Links only rvvi_text_writer (pure C++/stdio) — no GVSOC, no Questa license.
 * Certifies the line grammar: RET/TRAP prefix, X/F/C columns, MODE, ordering.
 *
 * Build + run:  make test   (from vendor_lib/gvsoc_rvvi/) */
#include "rvvi_text_writer.hpp"
#include "rvvi_text_dpi.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>

static int fails = 0;

static std::string emit_line(const RvviTextWriteSet &ws) {
    char *buf = nullptr; size_t len = 0;
    FILE *fp = open_memstream(&buf, &len);
    rvvi_text_write_line(fp, ws);
    fclose(fp);
    std::string s(buf, len); free(buf); return s;
}
static std::string emit_header(const RvviTextParams &p) {
    char *buf = nullptr; size_t len = 0;
    FILE *fp = open_memstream(&buf, &len);
    rvvi_text_write_header(fp, p);
    fclose(fp);
    std::string s(buf, len); free(buf); return s;
}
static void check(const char *name, const std::string &got, const std::string &exp) {
    if (got != exp) {
        printf("FAIL %s\n  got: [%s]\n  exp: [%s]\n", name, got.c_str(), exp.c_str());
        fails++;
    } else {
        printf("PASS %s\n", name);
    }
}
static std::string read_file(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) return "<open-failed>";
    std::string s; char buf[4096]; size_t n;
    while ((n = fread(buf, 1, sizeof buf, fp)) > 0) s.append(buf, n);
    fclose(fp);
    return s;
}

int main() {
    /* Header */
    RvviTextParams p{"gvsoc_rvvi", 32, 32, 32, 0, 1, 1};
    check("header", emit_header(p),
          "VERSION 0 1\nVENDOR \"gvsoc_rvvi\" 0 1\n"
          "PARAMS 6 ILEN 32 XLEN 32 FLEN 32 VLEN 0 NHART 1 RETIRE 1\n");

    /* RET + GPR (X) deltas */
    RvviTextWriteSet ws{};
    ws.pc = 0x84; ws.insn = 0xf8018193;
    ws.gpr_mask = (1u << 3) | (1u << 10);
    ws.gpr[3] = 0x240b0; ws.gpr[10] = 0x23408;
    check("RET+X", emit_line(ws), "RET 0x84 0xf8018193 X 3 0x240b0 X 10 0x23408\n");

    /* RET + FPR (F) */
    RvviTextWriteSet wf{};
    wf.pc = 0x100; wf.insn = 0xdeadbeef;
    wf.fpr_mask = (1u << 5); wf.fpr[5] = 0x3f800000;
    check("RET+F", emit_line(wf), "RET 0x100 0xdeadbeef F 5 0x3f800000\n");

    /* RET + CSR (C) */
    RvviTextWriteSet wc{};
    wc.pc = 0x200; wc.insn = 0x30529073;
    wc.csr = {{0x305, 0x1040}};
    check("RET+C", emit_line(wc), "RET 0x200 0x30529073 C 0x305 0x1040\n");

    /* TRAP line: the four trap-entry CSRs, no X/F */
    RvviTextWriteSet wt{};
    wt.pc = 0x100c; wt.insn = 0x9002; wt.is_trap = true;
    wt.csr = {{0x300, 0x3800}, {0x341, 0x100c}, {0x342, 0x3}, {0x343, 0x9002}};
    check("TRAP+4CSR", emit_line(wt),
          "TRAP 0x100c 0x9002 C 0x300 0x3800 C 0x341 0x100c C 0x342 0x3 C 0x343 0x9002\n");

    /* Trailing MODE column */
    RvviTextWriteSet wm{};
    wm.pc = 0x10; wm.insn = 0x73; wm.has_mode = true; wm.mode = 3;
    check("MODE", emit_line(wm), "RET 0x10 0x73 MODE 0x3\n");

    /* Column ordering: X then C then MODE */
    RvviTextWriteSet wcomb{};
    wcomb.pc = 0x80; wcomb.insn = 0x197;
    wcomb.gpr_mask = (1u << 1); wcomb.gpr[1] = 0xabc;
    wcomb.csr = {{0x300, 0x1800}};
    wcomb.has_mode = true; wcomb.mode = 3;
    check("RET+X+C+MODE order", emit_line(wcomb),
          "RET 0x80 0x197 X 1 0xabc C 0x300 0x1800 MODE 0x3\n");

    /* Full column order: X then F then C then MODE */
    RvviTextWriteSet wall{};
    wall.pc = 0x90; wall.insn = 0x197;
    wall.gpr_mask = (1u << 2); wall.gpr[2] = 0x1;
    wall.fpr_mask = (1u << 4); wall.fpr[4] = 0x2;
    wall.csr = {{0x305, 0x88}};
    wall.has_mode = true; wall.mode = 3;
    check("RET+X+F+C+MODE order", emit_line(wall),
          "RET 0x90 0x197 X 2 0x1 F 4 0x2 C 0x305 0x88 MODE 0x3\n");

    /* Duplicate CSR address on a TRAP (the csr_wb scan and the explicit
     * trap-CSR push both report it): one C token, first position, last value */
    RvviTextWriteSet wd{};
    wd.pc = 0x1010; wd.insn = 0x9002; wd.is_trap = true;
    wd.csr = {{0x300, 0x1800}, {0x341, 0x1010}, {0x300, 0x3800}};
    check("TRAP dedup C", emit_line(wd),
          "TRAP 0x1010 0x9002 C 0x300 0x3800 C 0x341 0x1010\n");

    /* DPI shim (rvvi_text_dpi) end-to-end: open -> set* -> write -> close ->
     * read the file back.  Covers the RTL-only tracer path: header, the
     * per-retire write-set accumulation, RET vs TRAP, and the reset between
     * lines (the RET's X must NOT leak onto the following TRAP line). */
    {
        const char *path = "test/.dpi_smoke.rvvi";
        rvviTextOpen(path, 32, 32, 0, 0, 1, 1);
        rvviTextSetGpr(10, 0x23408);
        rvviTextWrite(0x84, 0xf8018193, 0);          /* RET with X 10 */
        rvviTextSetCsr(0x300, 0x1800);
        rvviTextSetMode(3);
        rvviTextWrite(0x1000, 0x73, 1);              /* TRAP with C + MODE, no X */
        rvviTextClose();
        check("DPI open+write+close", read_file(path),
              "VERSION 0 1\nVENDOR \"gvsoc_rvvi\" 0 1\n"
              "PARAMS 6 ILEN 32 XLEN 32 FLEN 0 VLEN 0 NHART 1 RETIRE 1\n"
              "RET 0x84 0xf8018193 X 10 0x23408\n"
              "TRAP 0x1000 0x73 C 0x300 0x1800 MODE 0x3\n");
        remove(path);
    }

    /* DPI shim: duplicate CSR pushes collapse to one token, last value wins */
    {
        const char *path = "test/.dpi_dedup.rvvi";
        rvviTextOpen(path, 32, 32, 0, 0, 1, 1);
        rvviTextSetCsr(0x300, 0x1800);
        rvviTextSetCsr(0x341, 0x1010);
        rvviTextSetCsr(0x300, 0x3800);
        rvviTextWrite(0x1010, 0x9002, 1);
        rvviTextClose();
        check("DPI dedup C", read_file(path),
              "VERSION 0 1\nVENDOR \"gvsoc_rvvi\" 0 1\n"
              "PARAMS 6 ILEN 32 XLEN 32 FLEN 0 VLEN 0 NHART 1 RETIRE 1\n"
              "TRAP 0x1010 0x9002 C 0x300 0x3800 C 0x341 0x1010\n");
        remove(path);
    }

    /* DPI shim: a failed open reports it, and later calls are safe no-ops */
    {
        if (rvviTextOpen("test/no_such_dir/x.rvvi", 32, 32, 0, 0, 1, 1) != 0) {
            printf("FAIL DPI open-fail status\n  got: 1\n  exp: [0]\n");
            fails++;
        } else {
            printf("PASS DPI open-fail status\n");
        }
        rvviTextSetGpr(1, 0x1);
        rvviTextWrite(0x0, 0x13, 0);   /* nothing open: must not crash */
        rvviTextClose();
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILURES" : "ALL PASS", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
