// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

// The RVVI-API (riscv-verification/RVVI, rvviApi.h) and its decision-point
// extension (rvviDecisionApi.h), implemented on the CV32E40P model of GVSOC.
//
// After every retire, the DUT shadow written by rvviDut*Set from the RVFI rows
// is compared with the reference shadow written by the commit records of the
// model. The model only takes its inputs from the DUT: the interrupt and debug
// pins with the points where the DUT sampled and decided on them, the values
// read from volatile CSRs and the data loaded from volatile memory.
//
// Stores are compared on the data bus. The writes of rvviDutBusWrite and the
// stores of the model are cut into the same word beats and compared in program
// order.

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <array>
#include <bitset>
#include <deque>
#include <string>
#include <vector>
#include "rvviApi.h"
#include "rvviDecisionApi.h"
#include "cosim_client.hpp"

namespace {

constexpr uint32_t NUM_CSRS = 4096;
constexpr uint32_t CSR_FFLAGS = 0x001, CSR_FRM = 0x002, CSR_FCSR = 0x003;
constexpr uint32_t CSR_INSTRET = 0xC02, CSR_INSTRETH = 0xC82;
constexpr uint32_t CSR_MINSTRET = 0xB02, CSR_MINSTRETH = 0xB82;
constexpr uint32_t CSR_MHPMEVENT = 0x320;
constexpr unsigned MAX_REPORTS = 10;   // detailed messages per compare category

// The model counts these HPM events: retired, load, store, jump, branch, branch
// taken and compressed instructions (hpm_events in cv32e40p_cs_registers.sv).
constexpr uint32_t HPM_INSTRUCTION_EVENTS = 0x7E2;

// rvviRefNetIndexGet returns the irq_i bit for an interrupt net, and this index
// for the debug request.
constexpr uint64_t NET_HALTREQ = 32;

// The net groups follow the RTL. The interrupt lines are sampled by the irq_q
// flop on the gated clock, and haltreq by the sticky debug_req_q flop on the
// free-running clock.
constexpr uint32_t GROUP_IRQ = 1, GROUP_DEBUG = 2;

// The decision kinds of the extension are the opportunity kinds of the model.
static_assert((int)RVVI_DECISION_DISPATCH == (int)CV32E40P_COSIM_OPP_DISPATCH &&
    (int)RVVI_DECISION_FIRST_FETCH == (int)CV32E40P_COSIM_OPP_FIRST_FETCH &&
    (int)RVVI_DECISION_SLEEP == (int)CV32E40P_COSIM_OPP_SLEEP &&
    (int)RVVI_DECISION_BOOT == (int)CV32E40P_COSIM_OPP_BOOT, "decision kinds");

// One beat of a bus write, with a word address, the enabled byte lanes and the
// data on them. A reference beat keeps the retire and pc of its store, and a DUT
// beat keeps the number of retires before it.
struct BusWrite
{
    uint32_t word, byte_enable, data;
    uint64_t retire;
    uint32_t pc;
};

// Cuts the bytes [address, address + size) into word beats, in address order.
static void push_beats(std::deque<BusWrite> &writes, uint64_t address, uint32_t size,
    uint64_t data, uint64_t retire, uint32_t pc)
{
    for (uint32_t i = 0; i < size; i++)
    {
        uint64_t byte = address + i;
        uint32_t word = (uint32_t)(byte >> 2), lane = byte & 3;
        if (writes.empty() || i == 0 || writes.back().word != word)
        {
            writes.push_back({ word, 0, 0, retire, pc });
        }
        writes.back().byte_enable |= 1u << lane;
        writes.back().data |= (uint32_t)((data >> (8 * i)) & 0xFF) << (8 * lane);
    }
}

struct Shadow
{
    std::array<uint32_t, 32> gpr{};
    std::array<uint32_t, 32> fpr{};
    std::array<uint32_t, NUM_CSRS> csr{};
    std::bitset<NUM_CSRS> csr_valid;  // implemented by REF, reported at least once by DUT
    uint32_t gpr_written = 0;         // bitmask of the current retire
    uint32_t fpr_written = 0;

    void set_csr(uint32_t address, uint32_t value)
    {
        this->csr[address] = value;
        this->csr_valid[address] = true;
    }
};

class Bridge
{
public:
    // Lifecycle
    bool init(const char *program);
    void shutdown();

    // DUT side
    void dut_row_start();
    void dut_retire(uint64_t pc, uint64_t insn, bool debug_mode, bool trap);

    // Reference side
    bool event_step();
    void trace(const std::vector<Cv32e40pCosimBoundary> &boundaries);
    void net_set(uint64_t index, uint64_t value);
    bool csr_set(uint32_t address, uint64_t value);

    // Compares
    bool compare_pc();
    bool compare_insn();
    bool compare_gprs(bool written_only, bool ignore_x0);
    bool compare_fprs();
    bool compare_csr(uint32_t address, bool report);
    bool compare_csrs();
    void compare_hpm_read(uint32_t address, uint32_t value);
    void dut_bus_write(uint64_t address, uint64_t value, uint64_t byte_enable);
    void compare_writes();

    bool fail(const char *format, ...);
    bool unsupported(const char *function);
    void mismatch(unsigned &reports, const char *format, ...);

    Cv32e40pCosimClient client;
    bool started = false;
    bool stepped = false;             // set after a step, so that the next DUT call opens a row
    Shadow dut, ref;
    std::bitset<NUM_CSRS> csr_compare, csr_volatile;
    // Volatile memory ranges [base, base + length), whose loads take the data the
    // DUT loaded.
    std::vector<std::pair<uint32_t, uint32_t>> volatile_memory;
    std::array<uint32_t, NUM_CSRS> csr_mask;
    uint64_t dut_pc = 0, dut_insn = 0;
    bool dut_debug = false, dut_trap = false;
    Cv32e40pCosimCommit commit{};
    uint32_t irq_level = 0;
    bool haltreq = false;
    std::string error;
    std::array<uint64_t, RVVI_METRIC_FATALS + 1> metrics{};
    unsigned pc_reports = 0, insn_reports = 0, gpr_reports = 0, fpr_reports = 0, csr_reports = 0;
    std::deque<BusWrite> dut_writes, ref_writes;
    uint64_t store_comparisons = 0;
    unsigned store_reports = 0;
    std::vector<std::string> unsupported_calls;
    // GVSOC_RVVI_TRACE=first:last prints the records of these retires.
    uint64_t trace_first = 1, trace_last = 0;
};

Bridge bridge;

void log(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    printf("[cv32e40p_rvvi] ");
    vprintf(format, ap);
    printf("\n");
    fflush(stdout);
    va_end(ap);
}

bool Bridge::fail(const char *format, ...)
{
    char buffer[512];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);
    this->error = buffer;
    this->metrics[RVVI_METRIC_ERRORS]++;
    log("error: %s", buffer);
    return false;
}

bool Bridge::unsupported(const char *function)
{
    for (const std::string &name : this->unsupported_calls)
    {
        if (name == function)
        {
            return false;
        }
    }
    this->unsupported_calls.push_back(function);
    return this->fail("%s is not supported by the GVSOC reference", function);
}

void Bridge::mismatch(unsigned &reports, const char *format, ...)
{
    this->metrics[RVVI_METRIC_MISMATCHES]++;
    if (reports++ >= MAX_REPORTS)
    {
        return;
    }
    char buffer[512];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);
    log("mismatch at retire %llu (pc 0x%08llx): %s",
        (unsigned long long)this->metrics[RVVI_METRIC_RETIRES],
        (unsigned long long)this->dut_pc, buffer);
}

bool Bridge::init(const char *program)
{
    const char *config = getenv("GVSOC_CONFIG");
    if (config == nullptr)
    {
        return this->fail("GVSOC_CONFIG is not set (gvsoc_config.json from gvrun prepare)");
    }
    log("reference: %s, program: %s", config, program);
    const char *trace = getenv("GVSOC_RVVI_TRACE");
    if (trace != nullptr && sscanf(trace, "%lu:%lu", &this->trace_first, &this->trace_last) != 2)
    {
        return this->fail("GVSOC_RVVI_TRACE=%s: expected first:last", trace);
    }
    if (!this->client.open(config))
    {
        return this->fail("%s", this->client.error().c_str());
    }
    // The model state is only read directly here, at reset. From here on, the
    // reference shadow follows the commit records.
    for (uint32_t i = 1; i < 32; i++)
    {
        if (!this->client.read_gpr(i, this->ref.gpr[i]))
        {
            return this->fail("cannot read x%u at reset", i);
        }
    }
    // read_fpr fails when the core has no F registers (no FPU, or Zfinx).
    for (uint32_t i = 0; i < 32; i++)
    {
        if (!this->client.read_fpr(i, this->ref.fpr[i]))
        {
            break;
        }
    }
    for (uint32_t address = 0; address < NUM_CSRS; address++)
    {
        uint32_t value;
        if (this->client.read_csr(address, value))
        {
            this->ref.set_csr(address, value);
        }
    }
    // The DUT shadow starts from the RTL reset state, with all registers zero,
    // so the first compare checks the reset state of the model.
    this->csr_mask.fill(0xFFFFFFFF);
    this->started = true;
    for (uint32_t address = 0; address < NUM_CSRS; address++)
    {
        if (this->csr_volatile[address] && !this->client.volatile_csr(address))
        {
            return this->fail("%s", this->client.error().c_str());
        }
    }
    for (const auto &range : this->volatile_memory)
    {
        if (!this->client.external_region(range.first, range.second))
        {
            return this->fail("%s", this->client.error().c_str());
        }
    }
    // Send the pin levels set before the model existed.
    if ((this->irq_level != 0 || this->haltreq) &&
        !this->client.input(this->irq_level, this->haltreq))
    {
        return this->fail("%s", this->client.error().c_str());
    }
    return true;
}

void Bridge::shutdown()
{
    if (!this->started)
    {
        return;
    }
    std::string never, absent;
    for (uint32_t address = 0; address < NUM_CSRS; address++)
    {
        if (!this->csr_compare[address] || this->csr_volatile[address])
        {
            continue;
        }
        char name[8];
        snprintf(name, sizeof(name), " 0x%03x", address);
        if (!this->ref.csr_valid[address])
        {
            absent += name;
        }
        else if (!this->dut.csr_valid[address])
        {
            never += name;
        }
    }
    for (const BusWrite &w : this->ref_writes)
    {
        this->mismatch(this->store_reports, "store of retire %llu (pc 0x%08x) to 0x%08x "
            "byte enable 0x%x data 0x%08x: no DUT bus write", (unsigned long long)w.retire, w.pc,
            w.word << 2, w.byte_enable, w.data);
    }
    // The DUT writes the bus before it retires the store. The beats left belong to stores
    // still in the pipeline when the simulation stopped, which the reference never stepped.
    for (const BusWrite &w : this->dut_writes)
    {
        log("DUT bus write to 0x%08x byte enable 0x%x data 0x%08x after retire %llu: store not "
            "retired when the simulation stopped", w.word << 2, w.byte_enable, w.data,
            (unsigned long long)w.retire);
    }
    log("store beats compared %llu", (unsigned long long)this->store_comparisons);
    log("retires %llu, traps %llu, mismatches %llu, errors %llu, reference %s",
        (unsigned long long)this->metrics[RVVI_METRIC_RETIRES],
        (unsigned long long)this->metrics[RVVI_METRIC_TRAPS],
        (unsigned long long)this->metrics[RVVI_METRIC_MISMATCHES],
        (unsigned long long)this->metrics[RVVI_METRIC_ERRORS],
        this->client.ended() ? "exited" : "still running");
    if (!absent.empty())
    {
        log("compare-enabled CSRs not implemented by the reference (not checked):%s",
            absent.c_str());
    }
    if (!never.empty())
    {
        log("compare-enabled CSRs never reported by the DUT (not checked):%s", never.c_str());
    }
    this->client.close();
    this->started = false;
}

void Bridge::dut_row_start()
{
    if (this->stepped)
    {
        this->stepped = false;
        this->dut.gpr_written = this->dut.fpr_written = 0;
    }
}

void Bridge::dut_retire(uint64_t pc, uint64_t insn, bool debug_mode, bool trap)
{
    this->dut_row_start();
    this->dut_pc = pc;
    this->dut_insn = insn;
    this->dut_debug = debug_mode;
    this->dut_trap = trap;
}

// fcsr is {frm, fflags}, so a write to one of the three CSRs changes the others.
static void set_fp_csr(Shadow &shadow, uint32_t address, uint32_t value)
{
    uint32_t fcsr = shadow.csr[CSR_FCSR];
    if (address == CSR_FFLAGS)
    {
        fcsr = (fcsr & ~0x1Fu) | (value & 0x1F);
    }
    else if (address == CSR_FRM)
    {
        fcsr = (fcsr & ~0xE0u) | ((value & 0x7) << 5);
    }
    else
    {
        fcsr = value & 0xFF;
    }
    shadow.set_csr(CSR_FCSR, fcsr);
    shadow.set_csr(CSR_FFLAGS, fcsr & 0x1F);
    shadow.set_csr(CSR_FRM, (fcsr >> 5) & 0x7);
}

static void apply_csr(Shadow &shadow, uint32_t address, uint32_t value)
{
    if (address == CSR_FFLAGS || address == CSR_FRM || address == CSR_FCSR)
    {
        set_fp_csr(shadow, address, value);
    }
    else
    {
        shadow.set_csr(address, value);
    }
}

bool Bridge::event_step()
{
    if (!this->started)
    {
        return this->fail("rvviRefEventStep before rvviRefInit");
    }
    // A read of a volatile CSR takes the value the DUT read, which it wrote to rd.
    uint32_t insn = (uint32_t)this->dut_insn, funct3 = (insn >> 12) & 7;
    uint32_t rd = (insn >> 7) & 31, address = insn >> 20;
    if (!this->dut_trap && (insn & 0x7F) == 0x73 && funct3 != 0 && funct3 != 4 && rd != 0 &&
        this->csr_volatile[address] && ((this->dut.gpr_written >> rd) & 1))
    {
        this->compare_hpm_read(address, this->dut.gpr[rd]);
        if (!this->client.volatile_read(this->metrics[RVVI_METRIC_RETIRES] + 1, address,
            this->dut.gpr[rd]))
        {
            return this->fail("%s", this->client.error().c_str());
        }
    }
    std::vector<Cv32e40pCosimBoundary> boundaries;
    // A load from volatile memory takes the value of its destination register in
    // the DUT, and the model applies the access width and the extension.
    auto external_data = [this](const Cv32e40pCosimExternalLoad &load) -> uint32_t {
        return load.reg == 0 ? 0 : load.reg < 32 ? this->dut.gpr[load.reg]
                                                 : this->dut.fpr[load.reg - 32];
    };
    if (!this->client.step(this->commit, boundaries, external_data))
    {
        this->stepped = true;
        return this->fail("%s", this->client.error().c_str());
    }
    this->stepped = true;
    this->ref.gpr_written = this->ref.fpr_written = 0;
    for (const Cv32e40pCosimBoundary &boundary : boundaries)
    {
        for (uint32_t i = 0; i < boundary.n_csr; i++)
        {
            apply_csr(this->ref, boundary.csr[i].address, boundary.csr[i].value);
        }
    }
    for (uint32_t i = 0; i < this->commit.n_gpr; i++)
    {
        this->ref.gpr[this->commit.gpr[i].index] = this->commit.gpr[i].value;
        this->ref.gpr_written |= 1u << this->commit.gpr[i].index;
    }
    for (uint32_t i = 0; i < this->commit.n_fpr; i++)
    {
        this->ref.fpr[this->commit.fpr[i].index] = this->commit.fpr[i].value;
        this->ref.fpr_written |= 1u << this->commit.fpr[i].index;
    }
    for (uint32_t i = 0; i < this->commit.n_csr; i++)
    {
        apply_csr(this->ref, this->commit.csr[i].address, this->commit.csr[i].value);
    }
    // The instret counters change on every retire and are not in the CSR list.
    uint32_t low = (uint32_t)this->commit.minstret, high = (uint32_t)(this->commit.minstret >> 32);
    for (uint32_t address : { CSR_MINSTRET, CSR_INSTRET })
    {
        if (this->ref.csr_valid[address]) this->ref.set_csr(address, low);
    }
    for (uint32_t address : { CSR_MINSTRETH, CSR_INSTRETH })
    {
        if (this->ref.csr_valid[address]) this->ref.set_csr(address, high);
    }
    this->metrics[RVVI_METRIC_RETIRES]++;
    for (uint32_t i = 0; i < this->commit.n_mem; i++)
    {
        if (this->commit.mem[i].is_store)
        {
            push_beats(this->ref_writes, this->commit.mem[i].address, this->commit.mem[i].size,
                this->commit.mem[i].data, this->metrics[RVVI_METRIC_RETIRES], this->commit.pc);
        }
    }
    this->compare_writes();
    if (this->metrics[RVVI_METRIC_RETIRES] >= this->trace_first &&
        this->metrics[RVVI_METRIC_RETIRES] <= this->trace_last)
    {
        this->trace(boundaries);
    }
    if (this->commit.flags & CV32E40P_COSIM_REC_TRAPPED)
    {
        this->metrics[RVVI_METRIC_TRAPS]++;
    }
    return true;
}

void Bridge::trace(const std::vector<Cv32e40pCosimBoundary> &boundaries)
{
    for (const Cv32e40pCosimBoundary &boundary : boundaries)
    {
        log("trace: boundary kind %u cause %u irq %u pc 0x%08x next 0x%08x after %llu",
            boundary.kind, boundary.cause, boundary.irq_id, boundary.pc, boundary.next_pc,
            (unsigned long long)boundary.after_sequence);
    }
    const Cv32e40pCosimCommit &c = this->commit;
    log("trace: retire %llu seq %llu pc 0x%08x insn 0x%08x flags 0x%x next 0x%08x cycle %lld",
        (unsigned long long)this->metrics[RVVI_METRIC_RETIRES], (unsigned long long)c.sequence,
        c.pc, c.insn, c.flags, c.next_pc, (long long)c.cycle);
    for (uint32_t i = 0; i < c.n_gpr; i++)
        log("trace:   x%u = 0x%08x", c.gpr[i].index, c.gpr[i].value);
    for (uint32_t i = 0; i < c.n_fpr; i++)
        log("trace:   f%u = 0x%08x", c.fpr[i].index, c.fpr[i].value);
    for (uint32_t i = 0; i < c.n_mem; i++)
        log("trace:   %s 0x%08x size %u data 0x%08x", c.mem[i].is_store ? "store" : "load",
            c.mem[i].address, c.mem[i].size, c.mem[i].data);
}

void Bridge::net_set(uint64_t index, uint64_t value)
{
    if (index == NET_HALTREQ)
    {
        this->haltreq = value != 0;
    }
    else if (index < 32)
    {
        this->irq_level = (this->irq_level & ~(1u << index)) | ((value != 0 ? 1u : 0u) << index);
    }
    else
    {
        this->fail("rvviRefNetSet: unknown net index %llu", (unsigned long long)index);
        return;
    }
    if (this->started && !this->client.input(this->irq_level, this->haltreq))
    {
        this->fail("%s", this->client.error().c_str());
    }
}

bool Bridge::csr_set(uint32_t address, uint64_t value)
{
    // The model state is never written from outside. A testbench setting a
    // CSR to the value the model already has at reset is accepted.
    if (this->started && !this->stepped && this->metrics[RVVI_METRIC_RETIRES] == 0 &&
        address < NUM_CSRS && this->ref.csr_valid[address] && this->ref.csr[address] == value)
    {
        return true;
    }
    return this->fail("rvviRefCsrSet(0x%03x, 0x%llx): the reference state cannot be written "
        "(reset value 0x%08x)", address, (unsigned long long)value,
        address < NUM_CSRS ? this->ref.csr[address] : 0);
}

bool Bridge::compare_pc()
{
    this->metrics[RVVI_METRIC_COMPARISONS_PC]++;
    bool trapped = (this->commit.flags & CV32E40P_COSIM_REC_TRAPPED) != 0;
    bool debug = (this->commit.flags & CV32E40P_COSIM_REC_DEBUG_BEFORE) != 0;
    if (this->dut_pc != this->commit.pc)
    {
        this->mismatch(this->pc_reports, "pc DUT 0x%08llx REF 0x%08x",
            (unsigned long long)this->dut_pc, this->commit.pc);
        return false;
    }
    if (this->dut_trap != trapped)
    {
        this->mismatch(this->pc_reports, "trap DUT %d REF %d (REF cause 0x%08x)",
            this->dut_trap, trapped, this->commit.trap_cause);
        return false;
    }
    if (!this->dut_trap && this->dut_debug != debug)
    {
        this->mismatch(this->pc_reports, "debug mode DUT %d REF %d", this->dut_debug, debug);
        return false;
    }
    return true;
}

bool Bridge::compare_insn()
{
    this->metrics[RVVI_METRIC_COMPARISONS_INSBIN]++;
    // The RVFI of the core reports the fetched word, so for a compressed
    // instruction its upper half is the next parcel.
    uint32_t mask = this->commit.insn_len == 2 ? 0xFFFF : 0xFFFFFFFF;
    if (((uint32_t)this->dut_insn & mask) != this->commit.insn)
    {
        this->mismatch(this->insn_reports, "instruction DUT 0x%08llx REF 0x%08x",
            (unsigned long long)this->dut_insn, this->commit.insn);
        return false;
    }
    return true;
}

bool Bridge::compare_gprs(bool written_only, bool ignore_x0)
{
    this->metrics[RVVI_METRIC_COMPARISONS_GPR]++;
    uint32_t scope = written_only ? (this->dut.gpr_written | this->ref.gpr_written) : 0xFFFFFFFF;
    if (ignore_x0)
    {
        scope &= ~1u;
    }
    bool same = true;
    for (uint32_t i = 0; i < 32; i++)
    {
        if (((scope >> i) & 1) && this->dut.gpr[i] != this->ref.gpr[i])
        {
            this->mismatch(this->gpr_reports, "x%u DUT 0x%08x REF 0x%08x (written by %s)", i,
                this->dut.gpr[i], this->ref.gpr[i],
                ((this->dut.gpr_written >> i) & 1)
                    ? (((this->ref.gpr_written >> i) & 1) ? "both" : "DUT")
                    : (((this->ref.gpr_written >> i) & 1) ? "REF" : "none"));
            same = false;
        }
    }
    return same;
}

bool Bridge::compare_fprs()
{
    const Cv32e40pCosimInfo &info = this->client.info();
    if (!info.fpu || info.zfinx)
    {
        return true;
    }
    this->metrics[RVVI_METRIC_COMPARISONS_FPR]++;
    bool same = true;
    for (uint32_t i = 0; i < 32; i++)
    {
        if (this->dut.fpr[i] != this->ref.fpr[i])
        {
            this->mismatch(this->fpr_reports, "f%u DUT 0x%08x REF 0x%08x", i,
                this->dut.fpr[i], this->ref.fpr[i]);
            same = false;
        }
    }
    return same;
}

bool Bridge::compare_csr(uint32_t address, bool report)
{
    if (address >= NUM_CSRS || this->csr_volatile[address] || !this->dut.csr_valid[address])
    {
        return true;
    }
    // A CSR the model does not implement has no reference state. An access to
    // it traps in the model, and the PC and trap compares catch that.
    if (!this->ref.csr_valid[address])
    {
        return true;
    }
    uint32_t mask = this->csr_mask[address];
    if ((this->dut.csr[address] & mask) != (this->ref.csr[address] & mask))
    {
        if (report)
        {
            this->mismatch(this->csr_reports, "csr 0x%03x DUT 0x%08x REF 0x%08x (mask 0x%08x)",
                address, this->dut.csr[address], this->ref.csr[address], mask);
        }
        return false;
    }
    return true;
}

bool Bridge::compare_csrs()
{
    this->metrics[RVVI_METRIC_COMPARISONS_CSR]++;
    bool same = true;
    for (uint32_t address = 0; address < NUM_CSRS; address++)
    {
        if (this->csr_compare[address])
        {
            same &= this->compare_csr(address, true);
        }
    }
    return same;
}

// A volatile HPM counter (mhpmcounter3..31, its upper half or a user alias)
// whose selector in the model has only instruction events, or none, counts in
// the model as in the RTL: the value the DUT read is compared with the counter
// of the model, which has not executed the read yet.
void Bridge::compare_hpm_read(uint32_t address, uint32_t value)
{
    uint32_t index = address & 0x1F, bank = address - index;
    if (index < 3 || (bank != 0xB00 && bank != 0xB80 && bank != 0xC00 && bank != 0xC80))
    {
        return;
    }
    uint32_t selector, counter;
    if (!this->client.read_csr(CSR_MHPMEVENT + index, selector) ||
        (selector & ~HPM_INSTRUCTION_EVENTS) != 0 || !this->client.read_csr(address, counter))
    {
        return;
    }
    if (counter != value)
    {
        this->mismatch(this->csr_reports, "hpm counter read csr 0x%03x DUT 0x%08x REF 0x%08x "
            "(mhpmevent%u 0x%04x)", address, value, counter, index, selector);
    }
}

void Bridge::dut_bus_write(uint64_t address, uint64_t value, uint64_t byte_enable)
{
    // Bit i of the mask enables the byte at address + i, held by byte i of the value.
    bool first = true;
    for (uint32_t i = 0; i < 8; i++)
    {
        if (!((byte_enable >> i) & 1))
        {
            continue;
        }
        uint64_t byte = address + i;
        uint32_t word = (uint32_t)(byte >> 2), lane = byte & 3;
        if (first || this->dut_writes.back().word != word)
        {
            this->dut_writes.push_back({ word, 0, 0, this->metrics[RVVI_METRIC_RETIRES], 0 });
            first = false;
        }
        this->dut_writes.back().byte_enable |= 1u << lane;
        this->dut_writes.back().data |= (uint32_t)((value >> (8 * i)) & 0xFF) << (8 * lane);
    }
    this->compare_writes();
}

// The DUT writes the bus before it retires the store, while the reference writes
// it when the store retires. Compare the beats in order as soon as both sides
// have one.
void Bridge::compare_writes()
{
    while (!this->dut_writes.empty() && !this->ref_writes.empty())
    {
        const BusWrite &dut = this->dut_writes.front(), &ref = this->ref_writes.front();
        uint32_t lanes = 0;
        for (uint32_t lane = 0; lane < 4; lane++)
        {
            if ((ref.byte_enable >> lane) & 1) lanes |= 0xFFu << (8 * lane);
        }
        this->store_comparisons++;
        if (dut.word != ref.word || dut.byte_enable != ref.byte_enable ||
            (dut.data & lanes) != (ref.data & lanes))
        {
            this->mismatch(this->store_reports, "store of retire %llu (pc 0x%08x): "
                "DUT bus write 0x%08x byte enable 0x%x data 0x%08x, "
                "REF 0x%08x byte enable 0x%x data 0x%08x",
                (unsigned long long)ref.retire, ref.pc, dut.word << 2, dut.byte_enable,
                dut.data & lanes, ref.word << 2, ref.byte_enable, ref.data & lanes);
        }
        this->dut_writes.pop_front();
        this->ref_writes.pop_front();
    }
}

const char *const GPR_NAMES[32] = {
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4",
    "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4",
    "t5", "t6",
};

} // namespace

extern "C" {

bool_t rvviVersionCheck(uint32_t version)
{
    if (version != RVVI_API_VERSION)
    {
        return bridge.fail("RVVI API version 0x%08x requested, 0x%08x implemented", version,
            RVVI_API_VERSION);
    }
    return RVVI_TRUE;
}

bool_t rvviRefInit(const char *programPath)
{
    return bridge.init(programPath) ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefShutdown(void)
{
    bridge.shutdown();
    return RVVI_TRUE;
}

bool_t rvviRefCsrSetVolatile(uint32_t hartId, uint32_t csrIndex)
{
    if (csrIndex >= NUM_CSRS) return RVVI_FALSE;
    bridge.csr_volatile[csrIndex] = true;
    // rvviRefInit declares it when the model does not exist yet.
    if (bridge.started && !bridge.client.volatile_csr(csrIndex))
    {
        return bridge.fail("%s", bridge.client.error().c_str());
    }
    return RVVI_TRUE;
}

bool_t rvviRefCsrSetVolatileMask(uint32_t hartId, uint32_t csrIndex, uint64_t csrMask)
{
    if (csrIndex >= NUM_CSRS) return RVVI_FALSE;
    bridge.csr_mask[csrIndex] &= ~(uint32_t)csrMask;
    return RVVI_TRUE;
}

bool_t rvviRefMemorySetVolatile(uint64_t addressLow, uint64_t addressHigh)
{
    if (addressLow > addressHigh || addressHigh > 0xFFFFFFFFull)
    {
        return bridge.fail("rvviRefMemorySetVolatile(0x%llx, 0x%llx): not a 32-bit range",
            (unsigned long long)addressLow, (unsigned long long)addressHigh);
    }
    uint32_t base = (uint32_t)addressLow, length = (uint32_t)(addressHigh - addressLow + 1);
    bridge.volatile_memory.emplace_back(base, length);
    // rvviRefInit declares it when the model does not exist yet.
    if (bridge.started && !bridge.client.external_region(base, length))
    {
        return bridge.fail("%s", bridge.client.error().c_str());
    }
    return RVVI_TRUE;
}

uint64_t rvviRefNetIndexGet(const char *name)
{
    std::string net(name);
    if (net == "MSWInterrupt") return 3;
    if (net == "MTimerInterrupt") return 7;
    if (net == "MExternalInterrupt") return 11;
    if (net == "haltreq") return NET_HALTREQ;
    const std::string local = "LocalInterrupt";
    if (net.compare(0, local.size(), local) == 0)
    {
        char *end;
        long line = strtol(net.c_str() + local.size(), &end, 10);
        if (*end == '\0' && end != net.c_str() + local.size() && line >= 0 && line < 16)
        {
            return 16 + line;
        }
    }
    bridge.fail("rvviRefNetIndexGet: unknown net %s", name);
    return (uint64_t)RVVI_INVALID_INDEX;
}

void rvviRefNetGroupSet(uint64_t netIndex, uint32_t group)
{
    // The groups are fixed by the RTL, so only the same placement is accepted.
    uint32_t expected = netIndex == NET_HALTREQ ? GROUP_DEBUG : GROUP_IRQ;
    if (group != expected)
    {
        bridge.fail("rvviRefNetGroupSet: net %llu is in group %u", (unsigned long long)netIndex,
            expected);
    }
}

bool_t rvviRefNetGroupSample(uint32_t group)
{
    if (group != GROUP_IRQ && group != GROUP_DEBUG)
    {
        return bridge.fail("rvviRefNetGroupSample: unknown group %u", group);
    }
    if (!bridge.started)
    {
        return bridge.fail("rvviRefNetGroupSample before rvviRefInit");
    }
    uint32_t domain = group == GROUP_IRQ ? CV32E40P_COSIM_DOMAIN_IRQ : CV32E40P_COSIM_DOMAIN_DEBUG;
    if (!bridge.client.sample(domain))
    {
        return bridge.fail("%s", bridge.client.error().c_str());
    }
    return RVVI_TRUE;
}

bool_t rvviRefDecisionPoint(uint32_t hartId, uint32_t kind, uint64_t order)
{
    if (kind > RVVI_DECISION_BOOT)
    {
        return bridge.fail("rvviRefDecisionPoint: unknown kind %u", kind);
    }
    if (!bridge.started)
    {
        return bridge.fail("rvviRefDecisionPoint before rvviRefInit");
    }
    if (!bridge.client.opportunity(kind, order))
    {
        return bridge.fail("%s", bridge.client.error().c_str());
    }
    return RVVI_TRUE;
}

void rvviRefNetSet(uint64_t netIndex, uint64_t value, uint64_t when)
{
    bridge.net_set(netIndex, value);
}

uint64_t rvviRefNetGet(uint64_t netIndex)
{
    if (netIndex == NET_HALTREQ) return bridge.haltreq;
    if (netIndex < 32) return (bridge.irq_level >> netIndex) & 1;
    return 0;
}

void rvviDutGprSet(uint32_t hartId, uint32_t gprIndex, uint64_t value)
{
    bridge.dut_row_start();
    if (gprIndex == 0 || gprIndex >= 32) return;
    bridge.dut.gpr[gprIndex] = (uint32_t)value;
    bridge.dut.gpr_written |= 1u << gprIndex;
}

void rvviDutFprSet(uint32_t hartId, uint32_t fprIndex, uint64_t value)
{
    bridge.dut_row_start();
    if (fprIndex >= 32) return;
    bridge.dut.fpr[fprIndex] = (uint32_t)value;
    bridge.dut.fpr_written |= 1u << fprIndex;
}

void rvviDutCsrSet(uint32_t hartId, uint32_t csrIndex, uint64_t value)
{
    bridge.dut_row_start();
    if (csrIndex >= NUM_CSRS) return;
    uint64_t retire = bridge.metrics[RVVI_METRIC_RETIRES] + 1;
    if (retire >= bridge.trace_first && retire <= bridge.trace_last &&
        (!bridge.dut.csr_valid[csrIndex] || bridge.dut.csr[csrIndex] != (uint32_t)value))
    {
        log("trace: DUT retire %llu csr 0x%03x = 0x%08x", (unsigned long long)retire, csrIndex,
            (uint32_t)value);
    }
    bridge.dut.set_csr(csrIndex, (uint32_t)value);
}

void rvviDutRetire(uint32_t hartId, uint64_t dutPc, uint64_t dutInsBin, bool_t debugMode)
{
    bridge.dut_retire(dutPc, dutInsBin, debugMode != 0, false);
}

void rvviDutTrap(uint32_t hartId, uint64_t dutPc, uint64_t dutInsBin)
{
    // rvviDutTrap carries no debug mode, and the previous row cannot give it,
    // since a dret may come before the trap. The debug mode of a trapped row is
    // not compared, and the next row shows where the trap went.
    bridge.dut_retire(dutPc, dutInsBin, false, true);
}

bool_t rvviRefEventStep(uint32_t hartId)
{
    return bridge.event_step() ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefPcCompare(uint32_t hartId)
{
    return bridge.compare_pc() ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefInsBinCompare(uint32_t hartId)
{
    return bridge.compare_insn() ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefGprsCompare(uint32_t hartId)
{
    return bridge.compare_gprs(false, false) ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefGprsCompareWritten(uint32_t hartId, bool_t ignoreX0)
{
    return bridge.compare_gprs(true, ignoreX0 != 0) ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefFprsCompare(uint32_t hartId)
{
    return bridge.compare_fprs() ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefCsrCompare(uint32_t hartId, uint32_t csrIndex)
{
    bridge.metrics[RVVI_METRIC_COMPARISONS_CSR]++;
    return bridge.compare_csr(csrIndex, true) ? RVVI_TRUE : RVVI_FALSE;
}

void rvviRefCsrCompareEnable(uint32_t hartId, uint32_t csrIndex, bool_t enableState)
{
    if (csrIndex < NUM_CSRS) bridge.csr_compare[csrIndex] = enableState != 0;
}

void rvviRefCsrCompareMask(uint32_t hartId, uint32_t csrIndex, uint64_t mask)
{
    if (csrIndex < NUM_CSRS) bridge.csr_mask[csrIndex] = (uint32_t)mask;
}

bool_t rvviRefCsrsCompare(uint32_t hartId)
{
    return bridge.compare_csrs() ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefVrsCompare(uint32_t hartId)
{
    return RVVI_TRUE;  // no vector registers
}

void rvviRefReservationInvalidate(uint32_t hartId)
{
    // CV32E40P has no A extension, so there is no reservation to drop.
}

uint64_t rvviRefGprGet(uint32_t hartId, uint32_t gprIndex)
{
    return gprIndex < 32 ? bridge.ref.gpr[gprIndex] : 0;
}

uint32_t rvviRefGprsWrittenGet(uint32_t hartId)
{
    return bridge.ref.gpr_written;
}

uint64_t rvviRefPcGet(uint32_t hartId)
{
    return bridge.commit.pc;
}

uint64_t rvviRefCsrGet(uint32_t hartId, uint32_t csrIndex)
{
    return csrIndex < NUM_CSRS ? bridge.ref.csr[csrIndex] : 0;
}

uint64_t rvviRefInsBinGet(uint32_t hartId)
{
    return bridge.commit.insn;
}

uint64_t rvviRefFprGet(uint32_t hartId, uint32_t fprIndex)
{
    return fprIndex < 32 ? bridge.ref.fpr[fprIndex] : 0;
}

bool_t rvviRefCsrPresent(uint32_t hartId, uint32_t csrIndex)
{
    return csrIndex < NUM_CSRS && bridge.ref.csr_valid[csrIndex] ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefFprsPresent(uint32_t hartId)
{
    if (!bridge.started) return RVVI_FALSE;
    const Cv32e40pCosimInfo &info = bridge.client.info();
    return info.fpu && !info.zfinx ? RVVI_TRUE : RVVI_FALSE;
}

bool_t rvviRefVrsPresent(uint32_t hartId)
{
    return RVVI_FALSE;
}

const char *rvviRefGprName(uint32_t hartId, uint32_t gprIndex)
{
    return gprIndex < 32 ? GPR_NAMES[gprIndex] : "";
}

const char *rvviRefCsrName(uint32_t hartId, uint32_t csrIndex)
{
    static char name[8];
    snprintf(name, sizeof(name), "0x%03x", csrIndex & 0xFFF);
    return name;
}

const char *rvviRefFprName(uint32_t hartId, uint32_t fprIndex)
{
    static char name[8];
    snprintf(name, sizeof(name), "f%u", fprIndex & 31);
    return name;
}

const char *rvviRefVrName(uint32_t hartId, uint32_t vrIndex)
{
    return "";
}

const char *rvviErrorGet(void)
{
    return bridge.error.c_str();
}

uint64_t rvviRefMetricGet(rvviMetricE metric)
{
    return (unsigned)metric < bridge.metrics.size() ? bridge.metrics[metric] : 0;
}

void rvviRefCsrSet(uint32_t hartId, uint32_t csrIndex, uint64_t value)
{
    bridge.csr_set(csrIndex, value);
}

void rvviDutCycleCountSet(uint64_t cycleCount)
{
    // Cycle-accurate comparison is not part of the functional compare.
}

// The functions below are not provided. The reference state is only written by
// its own execution, and the program comes from the platform configuration.

bool_t rvviRefPcSet(uint32_t hartId, uint64_t address)
{
    return bridge.unsupported("rvviRefPcSet");
}

void rvviRefGprSet(uint32_t hartId, uint32_t gprIndex, uint64_t gprValue)
{
    bridge.unsupported("rvviRefGprSet");
}

void rvviRefFprSet(uint32_t hartId, uint32_t fprIndex, uint64_t fprValue)
{
    bridge.unsupported("rvviRefFprSet");
}

void rvviRefVrSet(uint32_t hartId, uint32_t vrIndex, uint32_t byteIndex, uint8_t data)
{
    bridge.unsupported("rvviRefVrSet");
}

uint8_t rvviRefVrGet(uint32_t hartId, uint32_t vrIndex, uint32_t byteIndex)
{
    bridge.unsupported("rvviRefVrGet");
    return 0;
}

void rvviDutVrSet(uint32_t hartId, uint32_t vrIndex, uint32_t byteIndex, uint8_t data)
{
    bridge.unsupported("rvviDutVrSet");
}

void rvviDutBusWrite(uint32_t hartId, uint64_t address, uint64_t value, uint64_t byteEnableMask)
{
    bridge.dut_bus_write(address, value, byteEnableMask);
}

void rvviRefMemoryWrite(uint32_t hartId, uint64_t address, uint64_t data, uint32_t size)
{
    bridge.unsupported("rvviRefMemoryWrite");
}

uint64_t rvviRefMemoryRead(uint32_t hartId, uint64_t address, uint32_t size)
{
    bridge.unsupported("rvviRefMemoryRead");
    return 0;
}

const char *rvviDasmInsBin(uint32_t hartId, uint64_t address, uint64_t insBin)
{
    return "";
}

void rvviRefStateDump(uint32_t hartId)
{
    bridge.unsupported("rvviRefStateDump");
}

bool_t rvviRefProgramLoad(const char *programPath)
{
    return bridge.unsupported("rvviRefProgramLoad");
}

bool_t rvviRefCsrSetOneWayCompare(uint32_t hartId, uint32_t csrIndex, bool_t enable)
{
    return bridge.unsupported("rvviRefCsrSetOneWayCompare");
}

bool_t rvviRefCsrSetOneWayCompareMask(uint32_t hartId, uint32_t csrIndex, uint64_t csrMask)
{
    return bridge.unsupported("rvviRefCsrSetOneWayCompareMask");
}

bool_t rvviRefConfigSetInt(uint64_t configParam, uint64_t value)
{
    return bridge.unsupported("rvviRefConfigSetInt");
}

bool_t rvviRefConfigSetString(uint64_t configParam, const char *value)
{
    return bridge.unsupported("rvviRefConfigSetString");
}

uint32_t rvviRefCsrIndex(uint32_t hartId, const char *csrName)
{
    bridge.unsupported("rvviRefCsrIndex");
    return (uint32_t)RVVI_INVALID_INDEX;
}

bool_t rvviRefMemorySetPrivilege(uint64_t addrLo, uint64_t addrHi, uint32_t access)
{
    return bridge.unsupported("rvviRefMemorySetPrivilege");
}

// Declared by rvviApiPkg.sv, but not by rvviApi.h.
void setContextExtMemory(const char *func)
{
    bridge.unsupported("setContextExtMemory");
}

uint64_t rvviRefConnIndexGet(const char *name)
{
    bridge.unsupported("rvviRefConnIndexGet");
    return (uint64_t)RVVI_INVALID_INDEX;
}

bool_t rvviRefConnSetEmpty(uint64_t connIndex)
{
    return bridge.unsupported("rvviRefConnSetEmpty");
}

bool_t rvviRefConnSetFull(uint64_t connIndex)
{
    return bridge.unsupported("rvviRefConnSetFull");
}

bool_t rvviRefConnData(uint64_t connIndex, uint32_t offset, uint64_t value, bool_t commit)
{
    return bridge.unsupported("rvviRefConnData");
}

} // extern "C"
