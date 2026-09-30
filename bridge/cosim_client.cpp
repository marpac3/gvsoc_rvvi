// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

#include <dlfcn.h>
#include <string.h>
#include <sys/stat.h>
#include <exception>
#include "cosim_client.hpp"

bool Cv32e40pCosimClient::fail(const std::string &message)
{
    this->error_ = message;
    return false;
}

void Cv32e40pCosimClient::has_ended(int status)
{
    this->has_ended_ = true;
    this->exit_status_ = status;
}

bool Cv32e40pCosimClient::open(const std::string &config_path)
{
    this->conf.config_path = config_path;
    std::string dir = config_path.substr(0, config_path.find_last_of('/') + 1);
    std::string runtime_path = dir + "gvsoc_runtime_config.txt";
    struct stat st;
    if (stat(runtime_path.c_str(), &st) == 0)
    {
        this->conf.runtime_config_path = runtime_path;
    }
    // In synchronous mode the engine runs inside step(), in the caller thread.
    this->conf.api_mode = gv::Api_mode_sync;

    void *core = nullptr;
    try
    {
        this->gvsoc = gv::gvsoc_new(&this->conf);
        this->gvsoc->bind(this);
        this->gvsoc->open();
        this->gvsoc->start();
        core = this->gvsoc->get_component("soc/core");
    }
    catch (const std::exception &e)
    {
        return this->fail(std::string("GVSOC failed to start: ") + e.what());
    }
    if (core == nullptr)
    {
        return this->fail("no component soc/core in " + config_path);
    }

    // The engine loads models with RTLD_GLOBAL, so the symbol of the CV32E40P
    // model is visible from here.
    Cv32e40pCosimAcquire acquire =
        (Cv32e40pCosimAcquire)dlsym(RTLD_DEFAULT, CV32E40P_COSIM_ACQUIRE);
    if (acquire == nullptr)
    {
        return this->fail(std::string("the core model does not export ") + CV32E40P_COSIM_ACQUIRE);
    }
    uint32_t minor = 0;
    this->cosim = acquire(core, CV32E40P_COSIM_MAJOR, &minor);
    if (this->cosim == nullptr)
    {
        return this->fail("soc/core is not a CV32E40P model with co-simulation support");
    }

    // No region is configured here. external_region() adds them before the
    // first step.
    Cv32e40pCosimConfig config;
    memset(&config, 0, sizeof(config));
    config.struct_size = sizeof(config);
    if (this->cosim->configure(&config) != CV32E40P_COSIM_OK)
    {
        return this->fail("the core model rejected the co-simulation configuration");
    }
    return true;
}

void Cv32e40pCosimClient::close()
{
    if (this->gvsoc != nullptr)
    {
        this->gvsoc->close();
        this->gvsoc = nullptr;
        this->cosim = nullptr;
    }
}

bool Cv32e40pCosimClient::step(Cv32e40pCosimCommit &commit,
    std::vector<Cv32e40pCosimBoundary> &boundaries)
{
    if (this->cosim == nullptr)
    {
        return this->fail("step before a successful open");
    }
    // Admit exactly one more instruction.
    if (this->cosim->arm(this->sequence + 1) == CV32E40P_COSIM_REJECTED)
    {
        return this->fail("the core model rejected arm()");
    }
    const int64_t period = this->info().clock_period_ps;
    for (int64_t cycles = 0; ; cycles++)
    {
        Cv32e40pCosimEvent event;
        Cv32e40pCosimStatus status;
        for (;;)
        {
            event.struct_size = sizeof(event);
            status = this->cosim->poll(&event);
            if (status != CV32E40P_COSIM_EVENT)
            {
                break;
            }
            if (event.type == CV32E40P_COSIM_EVENT_BOUNDARY)
            {
                boundaries.push_back(event.u.boundary);
                continue;
            }
            commit = event.u.commit;
            this->sequence = commit.sequence;
            return true;
        }
        if (status == CV32E40P_COSIM_ERROR)
        {
            return this->fail(std::string("core model error: ") + this->model_error());
        }

        if (this->has_ended_)
        {
            return this->fail("the reference software exited before its next instruction");
        }
        if (cycles == STEP_BUDGET_CYCLES)
        {
            return this->fail(std::string("no instruction retired by the reference in ")
                + std::to_string(STEP_BUDGET_CYCLES) + " cycles"
                + (status == CV32E40P_COSIM_SLEEPING ? " (sleeping in WFI)" : ""));
        }
        this->gvsoc->step(period);
    }
}

bool Cv32e40pCosimClient::input(uint32_t irq_level, bool debug_req)
{
    if (this->cosim == nullptr)
    {
        return this->fail("input before a successful open");
    }
    Cv32e40pCosimInput input;
    memset(&input, 0, sizeof(input));
    input.struct_size = sizeof(input);
    input.input_sequence = ++this->input_sequence;
    input.irq_level = irq_level;
    input.debug_req = debug_req;
    input.fetch_enable = 1;
    return this->check(this->cosim->input(&input), "an input");
}

bool Cv32e40pCosimClient::sample(uint32_t domain)
{
    if (this->cosim == nullptr)
    {
        return this->fail("sample before a successful open");
    }
    Cv32e40pCosimSample sample;
    memset(&sample, 0, sizeof(sample));
    sample.struct_size = sizeof(sample);
    sample.domain = domain;
    sample.sample_sequence = ++this->sample_sequence;
    return this->check(this->cosim->sample(&sample), "an input sample");
}

bool Cv32e40pCosimClient::opportunity(uint32_t kind, uint64_t ordinal)
{
    if (this->cosim == nullptr)
    {
        return this->fail("decision point before a successful open");
    }
    Cv32e40pCosimOpportunity opportunity;
    memset(&opportunity, 0, sizeof(opportunity));
    opportunity.struct_size = sizeof(opportunity);
    opportunity.kind = kind;
    opportunity.opportunity_id = ++this->opportunity_id;
    opportunity.ordinal = ordinal;
    return this->check(this->cosim->opportunity(&opportunity), "a decision point");
}

bool Cv32e40pCosimClient::check(Cv32e40pCosimStatus status, const char *what)
{
    if (status == CV32E40P_COSIM_ERROR)
    {
        return this->fail(std::string("core model error: ") + this->model_error());
    }
    if (status == CV32E40P_COSIM_REJECTED)
    {
        std::string error = this->model_error();
        return this->fail(std::string("the core model rejected ") + what +
            (error.empty() ? "" : ": " + error));
    }
    return true;
}

std::string Cv32e40pCosimClient::model_error() const
{
    Cv32e40pCosimSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.struct_size = sizeof(snapshot);
    this->cosim->snapshot(&snapshot);
    return snapshot.error;
}

bool Cv32e40pCosimClient::read_gpr(uint32_t index, uint32_t &value) const
{
    return this->cosim != nullptr && this->cosim->read_gpr(index, &value) == CV32E40P_COSIM_OK;
}

bool Cv32e40pCosimClient::read_csr(uint32_t address, uint32_t &value) const
{
    return this->cosim != nullptr && this->cosim->read_csr(address, &value) == CV32E40P_COSIM_OK;
}
