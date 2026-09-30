// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

// rvvi_trace2api drives the RVVI-API from an RVVI-TRACE interface, with one
// step and compare per retired instruction. For every valid retire slot:
//   1. the GPR, FPR and CSR writes of the row go to the DUT (rvviDut*Set);
//   2. the input nets changed since the previous row go to the reference,
//      unless NETS is 0 and the testbench reports the nets, their samples and
//      the decision points itself (rvviDecisionApiPkg);
//   3. rvviDutRetire, or rvviDutTrap on an exception, closes the row;
//   4. the reference retires its next instruction (rvviRefEventStep), and the
//      PC, instruction, GPR, FPR and CSR states are compared.
//
// The reference is never corrected from the DUT. The simulation stops after
// +rvvi_max_consecutive_mismatch=<n> failing retires in a row (default 50).

module rvvi_trace2api
  import rvviApiPkg::*;
#(
    parameter int NHART  = 1,
    parameter int RETIRE = 1,
    parameter bit NETS   = 1
)
(
    rvviTrace rvvi
);

    // CSRs pushed whatever their write-back flag says. On a trap row these are
    // the trap CSRs, whose flag rises one delta after the retire edge because
    // the testbench wiring detects value changes. On every row these are the
    // retired-instruction counters, which advance without a flag.
    localparam int unsigned CSR_MSTATUS   = 'h300;
    localparam int unsigned CSR_MEPC      = 'h341;
    localparam int unsigned CSR_MCAUSE    = 'h342;
    localparam int unsigned CSR_MTVAL     = 'h343;
    localparam int unsigned CSR_MINSTRET  = 'hB02;
    localparam int unsigned CSR_MINSTRETH = 'hB82;
    localparam int unsigned CSR_INSTRETH  = 'hC82;

    int          client_id;
    longint unsigned retire_count = 0;
    int unsigned consecutive_mismatch = 0;
    int unsigned max_consecutive_mismatch = 50;

    initial begin
        void'($value$plusargs("rvvi_max_consecutive_mismatch=%d", max_consecutive_mismatch));
        // Receive the net changes (interrupts and halt request), without memory events.
        client_id = rvvi.client_register(NETS, 1'b0);
    end

    function automatic void push_row(int h, int r);
        for (int i = 1; i < 32; i++)
            if (rvvi.x_wb[h][r][i])
                rvviDutGprSet(h, i, rvvi.x_wdata[h][r][i]);
        for (int i = 0; i < 32; i++)
            if (rvvi.f_wb[h][r][i])
                rvviDutFprSet(h, i, rvvi.f_wdata[h][r][i]);
        // Stop the scan after the last flagged CSR.
        for (int i = 0, n = $countones(rvvi.csr_wb[h][r]); n > 0; i++)
            if (rvvi.csr_wb[h][r][i]) begin
                rvviDutCsrSet(h, i, rvvi.csr[h][r][i]);
                n--;
            end
        if (rvvi.trap[h][r]) begin
            rvviDutCsrSet(h, CSR_MSTATUS, rvvi.csr[h][r][CSR_MSTATUS]);
            rvviDutCsrSet(h, CSR_MEPC,    rvvi.csr[h][r][CSR_MEPC]);
            rvviDutCsrSet(h, CSR_MCAUSE,  rvvi.csr[h][r][CSR_MCAUSE]);
            rvviDutCsrSet(h, CSR_MTVAL,   rvvi.csr[h][r][CSR_MTVAL]);
        end
        rvviDutCsrSet(h, CSR_MINSTRET,  rvvi.csr[h][r][CSR_MINSTRET]);
        rvviDutCsrSet(h, CSR_MINSTRETH, rvvi.csr[h][r][CSR_MINSTRETH]);
        rvviDutCsrSet(h, CSR_INSTRETH,  rvvi.csr[h][r][CSR_INSTRETH]);
    endfunction

    function automatic void push_nets();
        string           name;
        longint unsigned value;
        longint unsigned when;
        while (rvvi.net_pop(client_id, name, value, when))
            rvviRefNetSet(rvviRefNetIndexGet(name), value, when);
    endfunction

    // Returns 1 when every compare passes.
    function automatic bit step_and_compare(int h, int r);
        bit same = 1;
        if (!rvviRefEventStep(h)) begin
            $error("[rvvi_trace2api] reference failed at retire %0d (DUT pc 0x%08x): %s",
                   retire_count, rvvi.pc_rdata[h][r], rvviErrorGet());
            $finish;
            return 0;
        end
        if (!rvviRefPcCompare(h))     begin same = 0; $error("[rvvi_trace2api] PC mismatch at retire %0d (DUT pc 0x%08x)",          retire_count, rvvi.pc_rdata[h][r]); end
        if (!rvviRefInsBinCompare(h)) begin same = 0; $error("[rvvi_trace2api] instruction mismatch at retire %0d (DUT pc 0x%08x)", retire_count, rvvi.pc_rdata[h][r]); end
        if (!rvviRefGprsCompare(h))   begin same = 0; $error("[rvvi_trace2api] GPR mismatch at retire %0d (DUT pc 0x%08x)",         retire_count, rvvi.pc_rdata[h][r]); end
        if (!rvviRefFprsCompare(h))   begin same = 0; $error("[rvvi_trace2api] FPR mismatch at retire %0d (DUT pc 0x%08x)",         retire_count, rvvi.pc_rdata[h][r]); end
        if (!rvviRefCsrsCompare(h))   begin same = 0; $error("[rvvi_trace2api] CSR mismatch at retire %0d (DUT pc 0x%08x)",         retire_count, rvvi.pc_rdata[h][r]); end
        return same;
    endfunction

    always @(posedge rvvi.clk) begin
        for (int h = 0; h < NHART; h++) begin
            for (int r = 0; r < RETIRE; r++) begin
                if (rvvi.valid[h][r]) begin
                    retire_count++;
                    push_row(h, r);
                    push_nets();
                    if (rvvi.trap[h][r])
                        rvviDutTrap(h, rvvi.pc_rdata[h][r], rvvi.insn[h][r]);
                    else
                        rvviDutRetire(h, rvvi.pc_rdata[h][r], rvvi.insn[h][r], rvvi.debug_mode[h][r]);
                    if (step_and_compare(h, r)) begin
                        consecutive_mismatch = 0;
                    end else if (++consecutive_mismatch >= max_consecutive_mismatch) begin
                        $error("[rvvi_trace2api] %0d consecutive mismatching retires (last %0d, DUT pc 0x%08x): stopping",
                               consecutive_mismatch, retire_count, rvvi.pc_rdata[h][r]);
                        $finish;
                    end
                end
            end
        end
    end

endmodule
