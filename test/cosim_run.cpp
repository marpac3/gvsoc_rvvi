// SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Marco Paci (marco.paci@chips.it)

// Runs a cv32e40p_cosim* platform through the co-simulation interface
// alone, without a DUT, and prints one line per commit record in the
// instruction trace format of GVSOC (pc, opcode). Used to check that the
// record stream matches the standalone instruction trace. Without an RTL,
// every instruction gets one DISPATCH decision point, and the interrupt
// lines are the wires of the platform.
//
//   cosim_run [-v] <gvsoc_config.json> [max_records]   (-v adds the register and CSR writes)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cosim_client.hpp"

int main(int argc, char **argv)
{
    bool verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    if (verbose)
    {
        argv++;
        argc--;
    }
    if (argc < 2)
    {
        fprintf(stderr, "usage: %s [-v] <gvsoc_config.json> [max_records]\n", argv[0]);
        return 2;
    }
    uint64_t max_records = argc > 2 ? strtoull(argv[2], NULL, 0) : UINT64_MAX;
    Cv32e40pCosimClient client;
    if (!client.open(argv[1]))
    {
        fprintf(stderr, "error: %s\n", client.error().c_str());
        return 1;
    }
    uint64_t records = 0, boundaries_seen = 0;
    while (records < max_records)
    {
        Cv32e40pCosimCommit commit;
        std::vector<Cv32e40pCosimBoundary> boundaries;
        // No external region is declared, so the model owns all the memory.
        if (!client.opportunity(CV32E40P_COSIM_OPP_DISPATCH, records + 1) ||
            !client.step(commit, boundaries, [](const Cv32e40pCosimExternalLoad &) { return 0u; }))
        {
            if (client.ended())
            {
                break;
            }
            fprintf(stderr, "error after %llu records: %s\n", (unsigned long long)records,
                client.error().c_str());
            return 1;
        }
        boundaries_seen += boundaries.size();
        printf("%08x %08x%s", commit.pc, commit.insn,
            (commit.flags & CV32E40P_COSIM_REC_TRAPPED) ? " trap" : "");
        if (verbose)
        {
            for (const Cv32e40pCosimBoundary &boundary : boundaries)
            {
                printf(" [boundary %u", boundary.kind);
                for (uint32_t i = 0; i < boundary.n_csr; i++)
                    printf(" csr%03x=%08x", boundary.csr[i].address, boundary.csr[i].value);
                printf("]");
            }
            for (uint32_t i = 0; i < commit.n_gpr; i++)
                printf(" x%u=%08x", commit.gpr[i].index, commit.gpr[i].value);
            for (uint32_t i = 0; i < commit.n_fpr; i++)
                printf(" f%u=%08x", commit.fpr[i].index, commit.fpr[i].value);
            for (uint32_t i = 0; i < commit.n_csr; i++)
                printf(" csr%03x=%08x", commit.csr[i].address, commit.csr[i].value);
        }
        printf("\n");
        records++;
    }
    fprintf(stderr, "records %llu, boundaries %llu, exit status %d\n",
        (unsigned long long)records, (unsigned long long)boundaries_seen, client.exit_status());
    client.close();
    return 0;
}
