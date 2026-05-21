// =========================================================================
// cmd_port_test_step9_phaseD.c (v5 — 256 slots, random access)
//
// argv: <N> <instance_id> [n_iters] [num_slots]
//   N:            total instances (unused by worker)
//   instance_id:  which ORAM instance this Process drives
//   n_iters:      write+read pairs (default 10)
//   num_slots:    addressable slots (default 256)
//
// Build:
//   musl-gcc -O0 -static -o cmd_port_test_step9_phaseD cmd_port_test_step9_phaseD.c
// =========================================================================

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Addresses — must match phase_d_layout.py.
#define ORAM_CMD_BASE     0x0E0000000ULL
#define CMD_RING_BASE     0x400000000ULL
#define RESULT_BUF_BASE   0x410000000ULL

// MMIO offsets
#define OFF_NUM_K              0x80
#define OFF_TOKEN_BASE         0x84
#define OFF_READY              0xC4
#define OFF_CPU_OP_COUNT       0xE0
#define OFF_CMD_RING_DOORBELL  0x150

#define DEBUG_OFF_TOTAL_OPS    0x10
#define DEBUG_OFF_READY        0x14
#define DEBUG_OFF_K            0x18
#define DEBUG_OFF_OP_COUNT     0x1C
#define DEBUG_OFF_PASS         0x20
#define DEBUG_OFF_FAIL         0x24

// Ring layout
#define RING_PROD_IDX_OFF      0x000
#define RING_CONS_IDX_OFF      0x040
#define RING_ENTRIES_OFF       0x100
#define RING_ENTRY_BYTES       64
#define CMD_RING_DEPTH         16

// Result buffer entry layout
#define RES_OPIDX_OFF          0x00
#define RES_STATUS_OFF         0x08
#define RES_RDATA_OFF          0x20

#define RES_DONE_BIT           (1ull << 0)
#define RES_OP_WR_BIT          (1ull << 1)
#define RES_OP_RD_BIT          (1ull << 2)
#define RES_RDATA_VALID        (1ull << 3)

#define DEFAULT_N_ITERS        10
#define DEFAULT_NUM_SLOTS      256

// ---- Memory primitives ----
static inline uint32_t rd32(uint64_t addr) { return *(volatile uint32_t *)(uintptr_t)addr; }
static inline void     wr32(uint64_t addr, uint32_t v) { *(volatile uint32_t *)(uintptr_t)addr = v; }
static inline uint64_t rd64(uint64_t addr) { return *(volatile uint64_t *)(uintptr_t)addr; }
static inline void     wr64(uint64_t addr, uint64_t v) { *(volatile uint64_t *)(uintptr_t)addr = v; }

static uint32_t poll_mmio32(uint64_t addr, uint32_t mask, int maxIter) {
    for (int i = 0; i < maxIter; i++) {
        uint32_t v = rd32(addr);
        if (v & mask) return v;
    }
    return 0;
}

// ---- xorshift32 PRNG — period 2^32-1, no zero state ----
static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

// ---- Build a command ring entry ----
static void write_ring_entry(uint64_t ring_base, uint64_t opIdx,
                              uint32_t slot, uint32_t token,
                              uint8_t lease, uint8_t op, uint8_t hwid,
                              const uint32_t *wdata)
{
    uint64_t base = ring_base + RING_ENTRIES_OFF
                  + (opIdx % CMD_RING_DEPTH) * RING_ENTRY_BYTES;

    wr32(base + 0x00, slot);
    wr32(base + 0x04, token);
    uint32_t pack08 = (uint32_t)lease | ((uint32_t)op << 8) | ((uint32_t)hwid << 16);
    wr32(base + 0x08, pack08);
    wr32(base + 0x0C, (uint32_t)(opIdx & 0xFFFFFFFFu));
    if (op == 1 && wdata) {
        for (int j = 0; j < 8; j++)
            wr32(base + 0x10 + j * 4, wdata[j]);
    } else {
        for (int j = 0; j < 8; j++)
            wr32(base + 0x10 + j * 4, 0);
    }
    wr32(base + 0x30, (uint32_t)((opIdx >> 32) & 0xFFFFFFFFu));
    __asm__ __volatile__("" ::: "memory");
    wr32(base + 0x34, 1u);
    wr32(base + 0x38, 0);
    wr32(base + 0x3C, 0);
}

// ---- main ----
int main(int argc, char **argv)
{
    int N           = (argc > 1) ? atoi(argv[1]) : 1;
    int instance_id = (argc > 2) ? atoi(argv[2]) : 0;
    int n_iters     = (argc > 3) ? atoi(argv[3]) : DEFAULT_N_ITERS;
    int num_slots   = (argc > 4) ? atoi(argv[4]) : DEFAULT_NUM_SLOTS;
    (void)N;

    if (instance_id < 0 || instance_id >= 16) {
        fprintf(stderr, "instance_id out of range: %d\n", instance_id);
        return 1;
    }

    uint64_t cmd_base    = ORAM_CMD_BASE   + (uint64_t)instance_id * 0x1000ULL;
    uint64_t ring_base   = CMD_RING_BASE   + (uint64_t)instance_id * 0x1000ULL;
    uint64_t result_base = RESULT_BUF_BASE + (uint64_t)instance_id * 0x100000ULL;
    const uint32_t TOTAL_OPS = 2u * (uint32_t)n_iters;

    /* Wait for ORAM init complete. */
    uint32_t ready  = poll_mmio32(cmd_base + OFF_READY, 1, 10000000);
    uint32_t K      = rd32(cmd_base + OFF_NUM_K);
    uint32_t token0 = rd32(cmd_base + OFF_TOKEN_BASE + 0);
    uint32_t token1 = rd32(cmd_base + OFF_TOKEN_BASE + 4);
    uint32_t slots_per_client = (uint32_t)num_slots / K;

    /* Initialize ring header. */
    wr64(ring_base + RING_PROD_IDX_OFF, 0);
    wr64(ring_base + RING_CONS_IDX_OFF, 0);

    /* PRNG seeded per-instance: different access pattern per ORAM. */
    uint32_t rng_state = (uint32_t)(instance_id + 1) * 2654435761u; /* Knuth multiplicative */

    /* Inner loop — random slot access, write then read. */
    for (uint32_t i = 0; i < (uint32_t)n_iters; i++) {
        uint32_t slotIdx = xorshift32(&rng_state) % (uint32_t)num_slots;
        uint32_t slot = 0x1000u + slotIdx * 0x1000u;
        uint8_t  hwid = (slotIdx < slots_per_client) ? 0 : 1;
        uint32_t token = (hwid == 0) ? token0 : token1;
        uint8_t  lease_id = hwid + 1;

        uint32_t wdata[8];
        for (int j = 0; j < 8; j++)
            wdata[j] = 0xABCD0000u | (i << 8) | (uint32_t)j;

        /* Write */
        uint64_t wrIdx = 2ull * i;
        while ((wrIdx - rd64(ring_base + RING_CONS_IDX_OFF)) >= CMD_RING_DEPTH) { }
        write_ring_entry(ring_base, wrIdx, slot, token, lease_id, 1, hwid, wdata);
        wr64(ring_base + RING_PROD_IDX_OFF, wrIdx + 1);
        wr32(cmd_base + OFF_CMD_RING_DOORBELL, 1);

        /* Read same slot */
        uint64_t rdIdx = wrIdx + 1;
        while ((rdIdx - rd64(ring_base + RING_CONS_IDX_OFF)) >= CMD_RING_DEPTH) { }
        write_ring_entry(ring_base, rdIdx, slot, token, lease_id, 0, hwid, NULL);
        wr64(ring_base + RING_PROD_IDX_OFF, rdIdx + 1);
        wr32(cmd_base + OFF_CMD_RING_DOORBELL, 1);
    }

    /* Drain. */
    while (rd32(cmd_base + OFF_CPU_OP_COUNT) < TOTAL_OPS) { }

    /* Verify — each read returns data from the paired write in same iteration. */
    uint32_t pass = 0, fail = 0;
    for (uint32_t i = 0; i < (uint32_t)n_iters; i++) {
        uint64_t opIdx   = 2ull * i + 1;
        uint64_t resAddr = result_base + opIdx * 64;
        uint64_t status  = rd64(resAddr + RES_STATUS_OFF);
        if (!(status & RES_DONE_BIT))    { fail++; continue; }
        if (!(status & RES_RDATA_VALID)) { fail++; continue; }

        uint32_t r0 = rd32(resAddr + RES_RDATA_OFF + 0);
        uint32_t r1 = rd32(resAddr + RES_RDATA_OFF + 4);
        uint32_t expect0 = 0xABCD0000u | (i << 8) | 0u;
        uint32_t expect1 = 0xABCD0000u | (i << 8) | 1u;
        if (r0 == expect0 && r1 == expect1) pass++;
        else fail++;
    }

    /* MMIO debug echo. */
    wr32(cmd_base + DEBUG_OFF_TOTAL_OPS, TOTAL_OPS);
    wr32(cmd_base + DEBUG_OFF_READY,     ready);
    wr32(cmd_base + DEBUG_OFF_K,         K);
    wr32(cmd_base + DEBUG_OFF_OP_COUNT,  rd32(cmd_base + OFF_CPU_OP_COUNT));
    wr32(cmd_base + DEBUG_OFF_PASS,      pass);
    wr32(cmd_base + DEBUG_OFF_FAIL,      fail);

    return (fail == 0) ? 0 : 1;
}