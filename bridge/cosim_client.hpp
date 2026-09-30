// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <gv/gvsoc.hpp>
#include <cpu/iss_v2/include/cores/cv32e40p/cosim.hpp>

// Embedded GVSOC engine running a cv32e40p_cosim* platform, driven in lock
// step through the CV32E40P co-simulation interface. It only uses the public
// engine API (gv::Gvsoc) and cosim.hpp, and depends on nothing of the core
// layout.
class Cv32e40pCosimClient : public gv::Gvsoc_user
{
public:
    // config_path is the gvsoc_config.json written by `gvrun ... prepare`. The
    // runtime config next to it is also used when present.
    bool open(const std::string &config_path);
    void close();

    // Runs the model until it publishes its next commit record. Boundary
    // events (interrupt take, debug entry, wake-up) published before the
    // commit are appended to boundaries, in model order. The decision points
    // of that instruction must have been reported before.
    bool step(Cv32e40pCosimCommit &commit, std::vector<Cv32e40pCosimBoundary> &boundaries);

    // Sets new levels on the irq_i[31:0] and debug_req_i pins.
    bool input(uint32_t irq_level, bool debug_req);
    // Reports that the RTL sampled the pins of one domain (Cv32e40pCosimDomain).
    bool sample(uint32_t domain);
    // Reports that the RTL decided on interrupts and debug before the instruction
    // with sequence number ordinal. kind is a Cv32e40pCosimOpportunityKind.
    bool opportunity(uint32_t kind, uint64_t ordinal);

    // Architectural state reads, for checks and diagnostics. They return the
    // reset state before the first step, then the current state.
    bool read_gpr(uint32_t index, uint32_t &value) const;
    bool read_csr(uint32_t address, uint32_t &value) const;

    const Cv32e40pCosimInfo &info() const { return *this->cosim->info(); }
    bool ended() const { return this->has_ended_; }
    int exit_status() const { return this->exit_status_; }
    const std::string &error() const { return this->error_; }

    // gv::Gvsoc_user
    void has_ended(int status) override;

private:
    bool fail(const std::string &message);
    // Fails on REJECTED and ERROR, with the error text of the model.
    bool check(Cv32e40pCosimStatus status, const char *what);
    std::string model_error() const;

    // Upper bound on the core cycles spent waiting for one record, so that a
    // divergent model that spins or sleeps fails the step instead of hanging
    // the testbench.
    static constexpr int64_t STEP_BUDGET_CYCLES = 1000000;

    gv::GvsocConf conf;              // gvsoc_new keeps a pointer to it
    gv::Gvsoc *gvsoc = nullptr;
    Cv32e40pCosim *cosim = nullptr;
    uint64_t sequence = 0;
    uint64_t input_sequence = 0;
    uint64_t sample_sequence = 0;
    uint64_t opportunity_id = 0;
    bool has_ended_ = false;
    int exit_status_ = 0;
    std::string error_;
};
