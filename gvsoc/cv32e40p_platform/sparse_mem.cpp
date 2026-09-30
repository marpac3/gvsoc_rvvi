// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

/*
 * Background sparse memory: mapped as the interconnect's catch-all route,
 * it makes never-written bytes read 0 and written bytes persist, matching
 * the UVM testbench sparse memory model.
 */

#include <cstdint>
#include <unordered_map>

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>

class Cv32e40pSparseMem : public vp::Component
{
public:
    Cv32e40pSparseMem(vp::ComponentConf &config);

private:
    static vp::IoReqStatus req(vp::Block *__this, vp::IoReq *req);

    vp::Trace   trace;
    vp::IoSlave in{&Cv32e40pSparseMem::req};

    /* Byte-granular backing store: only written bytes are kept. */
    std::unordered_map<uint64_t, uint8_t> store;
};

Cv32e40pSparseMem::Cv32e40pSparseMem(vp::ComponentConf &config)
    : vp::Component(config)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_slave_port("input", &this->in);
}

vp::IoReqStatus Cv32e40pSparseMem::req(vp::Block *__this, vp::IoReq *req)
{
    Cv32e40pSparseMem *_this = (Cv32e40pSparseMem *)__this;

    uint64_t addr = req->get_addr();
    uint64_t size = req->get_size();
    uint8_t *data = req->get_data();

    _this->trace.msg(vp::Trace::LEVEL_DEBUG,
        "background access (addr: 0x%llx, size: 0x%llx, is_write: %d)\n",
        addr, size, req->get_is_write());

    if (req->get_opcode() == vp::IoReqOpcode::WRITE)
    {
        for (uint64_t i = 0; i < size; i++)
            _this->store[addr + i] = data[i];
    }
    else if (req->get_opcode() == vp::IoReqOpcode::READ)
    {
        for (uint64_t i = 0; i < size; i++)
        {
            auto it = _this->store.find(addr + i);
            data[i] = (it != _this->store.end()) ? it->second : 0;
        }
    }
    else
    {
        /* Atomics are not supported (no A extension on this platform). */
        req->set_resp_status(vp::IO_RESP_INVALID);
    }

    return vp::IO_REQ_DONE;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Cv32e40pSparseMem(config);
}
