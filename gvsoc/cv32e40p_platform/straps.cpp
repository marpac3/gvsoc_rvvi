// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

/*
 * CV32E40P static configuration inputs driven by the testbench (RTL
 * mtvec_addr_i). The values are runtime configuration fields, so each test
 * sets its own without rebuilding the platform.
 */

#include <vp/vp.hpp>
#include <vp/itf/wire.hpp>
#include <cv32e40p_platform/devices/cv32e40p_straps_config.hpp>

class Cv32e40pStraps : public vp::Component
{
public:
    Cv32e40pStraps(vp::ComponentConf &config);
    void reset(bool active) override;

private:
    Cv32e40pStrapsConfig cfg;
    vp::WireMaster<uint32_t> mtvec_addr_itf;
};

Cv32e40pStraps::Cv32e40pStraps(vp::ComponentConf &config)
    : vp::Component(config, this->cfg)
{
    this->new_master_port("mtvec_addr", &this->mtvec_addr_itf);
}

void Cv32e40pStraps::reset(bool active)
{
    if (!active && this->mtvec_addr_itf.is_bound())
    {
        this->mtvec_addr_itf.sync(this->cfg.mtvec_addr);
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Cv32e40pStraps(config);
}
