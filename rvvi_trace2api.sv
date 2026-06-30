
/**
 * Module rvvi_trace2api
 *
 * Monitors the rvviTrace interface and drives the RVVI DPI API each cycle.
 * Open-source replacement for the proprietary trace2api module, used ONLY on
 * the GVSOC DPI co-simulation path: this file is compiled only under USE_GVSOC
 * (the Imperas/OVPSim path uses trace2api + uvma_rvvi_ovpsim instead).
 *
 * Per-retire sequence:
 *   1. Push GPR/FPR/CSR write-back data (architectural retire guard)
 *   2. Push pending interrupt nets (always; IRQs are asynchronous)
 *   3. Notify bridge of DUT retirement (rvviDutRetire or rvviDutTrap)
 *   4. On trap: consume the extra ISS exception-dispatch step silently
 *   5. Step reference model and compare PC/GPR/CSR/FPR (non-trap only)
 */
module rvvi_trace2api
  import rvviApiPkg::*;
#(
    parameter int NHART  = 1,
    parameter int RETIRE = 1
)
(
    rvviTrace  rvvi
);

    int client_id;

    initial begin
        // RVVI v1.37 client_register(recv_nets, recv_memory).
        // recv_nets=1: subscribe to the interrupt nets so net_pop() delivers the
        //   DUT irq_i toggles into the reference model's mip (rvviRefNetSet ->
        //   gvsoc_engine_set_irq). Required for tests that software-poll mip
        //   (e.g. interrupt_test): without it the reference mip stays 0 while the
        //   DUT mip reflects the pending bit, and the poll diverges.
        // recv_memory=0: step-n-compare does not consume memory-access events.
        client_id = rvvi.client_register(1'b1, 1'b0);
    end

`ifdef USE_GVSOC
    // GVSOC-specific batch DPI: collapses step + 4 compares into one crossing.
    // Returns a bitmask: 0x01=step 0x02=PC 0x04=GPR 0x08=CSR 0x10=FPR 0x20=runaway.
    import "DPI-C" function int rvviRefRetireAndCompare(
        input int unsigned  hartId,
        input longint unsigned dutPc,
        input int unsigned  dutInsn,
        input byte unsigned debugMode);

    // Informed IRQ injection (OVPSim-style "deferint"): on an external-interrupt
    // trap retire, tell the reference ISS to TAKE the IRQ and COMPUTE the entry
    // itself (no DUT-state copy). Plusarg-gated via +rvvi_informed_irq; when off,
    // the reactive resync in rvvi_api2gvsoc.cpp stays the only IRQ path.
    import "DPI-C" function void rvviRefSetInformedIrq(input int enable);
    import "DPI-C" function void rvviRefInjectIrq(
        input int unsigned hartId,
        input int unsigned mcause);

    // Enable the informed-injection path in the C bridge iff the plusarg is set.
    bit informed_irq_en = 1'b0;
    initial begin
        informed_irq_en = $test$plusargs("rvvi_informed_irq") != 0;
        rvviRefSetInformedIrq(informed_irq_en ? 1 : 0);
    end
`endif

    // Debug counters
    longint unsigned retire_count = 0;
    int unsigned err_count = 0;
    localparam int unsigned MAX_ERR_LOG = 10;

    // Consecutive-mismatch watchdog bound (see the compare block below).
    // Override on the command line with +rvvi_max_consecutive_mismatch=<n>.
    int unsigned consecutive_mismatch = 0;
    int unsigned max_consecutive_mismatch = 50;
    initial void'($value$plusargs("rvvi_max_consecutive_mismatch=%d", max_consecutive_mismatch));

    always @(posedge rvvi.clk) begin
        for (int h=0; h<NHART; h++) begin
            for (int r=0; r<RETIRE; r++) begin
                if (rvvi.valid[h][r]) begin
                    retire_count++;

                    // Diagnostic heartbeat (opt-in): first 5 retires, then every 1000th.
                    // Enable with +rvvi_trace2api_verbose on the simulator command line.
                    if ($test$plusargs("rvvi_trace2api_verbose") &&
                        (retire_count <= 5 || (retire_count % 1000 == 0)))
                        $display("[rvvi_trace2api] retire #%0d: PC=0x%08x insn=0x%08x trap=%0b order=%0d",
                                 retire_count, rvvi.pc_rdata[h][r], rvvi.insn[h][r],
                                 rvvi.trap[h][r], rvvi.order[h][r]);

                    // Skip GPR/FPR/CSR push for pipeline-flush artifacts (PC=0x0).
                    // Trap retires (trap=1, PC!=0) are always allowed through.
                    if (rvvi.trap[h][r] || rvvi.pc_rdata[h][r] != 0) begin

                        // 1. Push GPR changes from DUT to Bridge.
                        // x0 is hardwired to zero - skip it even if x_wb[0] is flagged.
                        for (int i=1; i<32; i++) begin
                            if (rvvi.x_wb[h][r][i]) begin
                                rvviDutGprSet(h, i, rvvi.x_wdata[h][r][i]);
                            end
                        end

                        // 2. Push FPR changes from DUT to Bridge
                        for (int i=0; i<32; i++) begin
                            if (rvvi.f_wb[h][r][i]) begin
                                rvviDutFprSet(h, i, rvvi.f_wdata[h][r][i]);
                            end
                        end

                        // 3. Push CSR changes from DUT to Bridge (sparse scan).
                        begin
                            automatic int wb_total = $countones(rvvi.csr_wb[h][r]);
                            if (wb_total > 0) begin
                                automatic int wb_found = 0;
                                for (int i = 0; i < 4096 && wb_found < wb_total; i++) begin
                                    if (rvvi.csr_wb[h][r][i]) begin
                                        rvviDutCsrSet(h, i, rvvi.csr[h][r][i]);
                                        wb_found++;
                                    end
                                end
                            end
                        end

                    end // architectural retire guard

                    // 4. Push Nets/Interrupts - always, IRQ changes are asynchronous
                    begin
                        string name;
                        longint unsigned value;
                        longint unsigned pslot;
                        while (rvvi.net_pop(client_id, name, value, pslot)) begin
                            rvviRefNetSet(rvviRefNetIndexGet(name), value, pslot);
                        end
                    end

                    // 5. Notify Bridge of DUT retirement
                    if (rvvi.trap[h][r]) begin
                        // csr_wb for exception CSRs is asserted one delta-cycle after
                        // the trap-retire posedge, so step 3 reads csr_wb=0 and misses
                        // mepc/mcause/mtval.  Push them explicitly here from the
                        // combinatorially-stable rvvi.csr values, which use the
                        // RVVI_SET_TRAP_CSR macro (wdata direct when wmask==0) to give
                        // the correct value even on the first exception.
                        rvviDutCsrSet(h, 12'h300, rvvi.csr[h][r][12'h300]);  // mstatus (MPIE/MIE updated on trap)
                        rvviDutCsrSet(h, 12'h341, rvvi.csr[h][r][12'h341]);  // mepc   (trap-CSR macro)
                        rvviDutCsrSet(h, 12'h342, rvvi.csr[h][r][12'h342]);  // mcause (trap-CSR macro)
                        rvviDutCsrSet(h, 12'h343, rvvi.csr[h][r][12'h343]);  // mtval
                        rvviDutTrap(h, rvvi.pc_rdata[h][r], rvvi.insn[h][r]);
                        // GVSOC models exceptions as two ISS steps: (1) faulting
                        // instruction, (2) jump to mtvec.  Consume step 1 silently here;
                        // the handler retire consumes step 2 normally.  No comparison.
                        // NOTE: this is the SYNCHRONOUS-exception path (rvfi_trap=1,
                        // mcause[31]=0).  External interrupts do NOT set rvfi_trap and
                        // are handled in the normal-retire path below (rvfi_intr).
                        void'(rvviRefEventStep(h));
                    end else if (rvvi.pc_rdata[h][r] != 0) begin
                        rvviDutRetire(h, rvvi.pc_rdata[h][r], rvvi.insn[h][r], rvvi.debug_mode[h][r]);
`ifdef USE_GVSOC
                        // Informed IRQ injection (gated +rvvi_informed_irq).  The first
                        // instruction of an EXTERNAL-INTERRUPT handler is a NORMAL retire
                        // (rvfi_trap=0) with mcause[31]=1.  Tell the ISS to TAKE the IRQ
                        // and COMPUTE the handler entry itself (one extra ISS step into
                        // mtvec); the batch step-n-compare below then retires and CHECKS
                        // the first handler instruction against the DUT (entry computed by
                        // the ISS, not copied) -- the OVPSim deferint protocol.  Replaces
                        // the reactive DUT-state-copy resync.
                        //
                        // NOTE: rvfi_intr (rvvi.intr) is UNDRIVEN in the CV32E40P RVFI, so
                        // it cannot gate the entry.  We pass every mcause[31]=1 retire to
                        // the C bridge, which fires exactly once per genuine take: the C
                        // entry-detect confirms the DUT retired AT the vectored mtvec entry
                        // (base + cause*4) and the ISS has not already taken it (ISS MIE==1).
                        // A stale mcause[31] lingering in normal code is rejected because the
                        // DUT PC there is not the trap vector.
                        if (informed_irq_en && rvvi.csr[h][r][12'h342][31])
                            rvviRefInjectIrq(h, rvvi.csr[h][r][12'h342]);
`endif
                    end

                    // 6. Step Reference Model + 7. Comparisons
                    // Skipped on trap retires (handled above) and PC=0 flush artifacts.
                    if (!rvvi.trap[h][r] && rvvi.pc_rdata[h][r] != 0) begin
                        // Batch DPI: step + all compares in one crossing.
                        // Bit 0 (step) is checked first; if it fails the other bits
                        // are meaningless and only the step error is reported.
                        automatic int cmp_result = rvviRefRetireAndCompare(
                            h,
                            rvvi.pc_rdata[h][r],
                            rvvi.insn[h][r],
                            rvvi.debug_mode[h][r]);
                        if (!(cmp_result & 32'h01)) begin
                            if (err_count < MAX_ERR_LOG) begin
                                $error("RVVI Bridge: rvviRefEventStep FAILED for hart %0d at retire #%0d (PC=0x%08x)",
                                       h, retire_count, rvvi.pc_rdata[h][r]);
                                err_count++;
                            end
                        end else begin
                            if (!(cmp_result & 32'h02) && err_count < MAX_ERR_LOG) begin
                                $error("RVVI Mismatch: PC at retire #%0d order=%0d (DUT PC=0x%08x)",
                                       retire_count, rvvi.order[h][r], rvvi.pc_rdata[h][r]);
                                err_count++;
                            end
                            if (!(cmp_result & 32'h04) && err_count < MAX_ERR_LOG) begin
                                $error("RVVI Mismatch: GPR at retire #%0d order=%0d (DUT PC=0x%08x)",
                                       retire_count, rvvi.order[h][r], rvvi.pc_rdata[h][r]);
                                err_count++;
                            end
                            if (!(cmp_result & 32'h08) && err_count < MAX_ERR_LOG) begin
                                $error("RVVI Mismatch: CSR at retire #%0d order=%0d (DUT PC=0x%08x)",
                                       retire_count, rvvi.order[h][r], rvvi.pc_rdata[h][r]);
                                err_count++;
                            end
                            if (!(cmp_result & 32'h10) && err_count < MAX_ERR_LOG) begin
                                $error("RVVI Mismatch: FPR at retire #%0d order=%0d (DUT PC=0x%08x)",
                                       retire_count, rvvi.order[h][r], rvvi.pc_rdata[h][r]);
                                err_count++;
                            end
                        end

                        // Runaway detector (complements the consecutive-mismatch
                        // watchdog below): the ISS has diverged AND got stuck (it
                        // spins at a fixed PC, exhausting its per-step cycle budget
                        // with no clean retire). Without this, the sim crawls until
                        // the external OS timeout reaps it at a non-deterministic
                        // point. The 0x20 bit converts that hang into a clean,
                        // deterministic FAIL. Fires regardless of the per-bit
                        // mismatch logging above.
                        if (cmp_result & 32'h20) begin
                            $error("RVVI Bridge: GVSOC ISS runaway (diverged + stuck) at retire #%0d (DUT PC=0x%08x) - aborting", retire_count, rvvi.pc_rdata[h][r]);
                            $finish;
                        end

                        // Bound a sustained divergence so the regression records
                        // a FAIL instead of running to the test's full length (or
                        // hanging).  A clean retire resets the run; once the ISS
                        // and RTL have disagreed for max_consecutive_mismatch
                        // retires in a row, abort.  A reference model with
                        // reconverge-on-mismatch re-syncs instead, but this
                        // open-source bridge only sees DUT write-backs, so it
                        // bounds-and-aborts.
                        if ((cmp_result & 32'h1F) == 32'h1F) begin
                            consecutive_mismatch = 0;
                        end else begin
                            consecutive_mismatch++;
                            if (consecutive_mismatch >= max_consecutive_mismatch) begin
                                $error("RVVI Bridge: step-and-compare diverged for %0d consecutive retires (last retire #%0d, DUT PC=0x%08x, cmp_result=0x%02x [b1=PC b2=GPR b3=CSR b4=FPR]) - aborting",
                                       consecutive_mismatch, retire_count, rvvi.pc_rdata[h][r], cmp_result);
                                $finish;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule
