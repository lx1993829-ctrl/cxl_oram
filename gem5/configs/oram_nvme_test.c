/*
 * oram_nvme_test.c — Multi-instance NVMe + ORAM staging test.
 *
 * One shared NVMe SSD, LBA-partitioned across N ORAM instances.
 * Each instance has its own I/O queue pair (qid = instance_id + 1).
 * Instance 0 initializes the NVMe controller and creates all queue pairs;
 * other instances poll a ready flag before proceeding.
 *
 * Per iteration (zero-copy — NVMe DMAs directly to/from DDR5 slab):
 *   1. Stage IN:  NVMe Read SSD → DDR5 ORAM slab (direct DMA)
 *   2. ORAM Write + Read via cmd ring
 *   3. Stage OUT: NVMe Write DDR5 ORAM slab → SSD (direct DMA)
 *
 * argv (matches config script):
 *   [1] N             — total ORAM instances
 *   [2] instance_id   — this instance (0-based)
 *   [3] n_iters       — ORAM iterations (default 10)
 *   [4] num_slots     — ORAM slots per instance (default 32)
 *   [5] bar0_addr     — NVMe BAR0 (hex, default 0xF0000000)
 *   [6] max_lba       — LBAs per instance (default 1048576 = 512MB/512B)
 *
 * Build: musl-gcc -O2 -Wall -static -o oram_nvme_test oram_nvme_test.c
 */

#include "se_io.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ================================================================
 * NVMe register offsets and bits
 * ================================================================ */
#define NVME_REG_CAP    0x0000
#define NVME_REG_VS     0x0008
#define NVME_REG_CC     0x0014
#define NVME_REG_CSTS   0x001C
#define NVME_REG_AQA    0x0024
#define NVME_REG_ASQ    0x0028
#define NVME_REG_ACQ    0x0030

#define NVME_DBELL_SQ(y)  (0x1000u + (2u * (y)) * 4u)
#define NVME_DBELL_CQ(y)  (0x1000u + ((2u * (y)) + 1u) * 4u)

#define CC_EN           (1u << 0)
#define CC_IOSQES_64B   (6u << 16)
#define CC_IOCQES_16B   (4u << 20)
#define CSTS_RDY        (1u << 0)

#define ADMIN_OPC_CREATE_SQ   0x01
#define ADMIN_OPC_CREATE_CQ   0x05
#define IO_OPC_WRITE          0x01
#define IO_OPC_READ           0x02

#define ADMIN_QUEUE_ENTRIES   16
#define IO_QUEUE_ENTRIES      64
#define POLL_MAX_ITERS        2000000
#define MAX_INSTANCES         16

/* ================================================================
 * Shared NVMe QREGION layout inside DDR5 aggregate.
 *
 * QREGION_BASE = DDR_AGG_BASE + 0x020000000 = 0x620000000
 * (gap between result_buf end and DDR slab start)
 *
 *   +0x00000  ASQ              (shared, admin init only)
 *   +0x01000  ACQ              (shared, admin init only)
 *   +0x02000  IOSQ[0]  (qid=1, instance 0)
 *   +0x03000  IOCQ[0]  (qid=1, instance 0)
 *   +0x04000  IOSQ[1]  (qid=2, instance 1)
 *   +0x05000  IOCQ[1]  (qid=2, instance 1)
 *   ...       (stride 0x2000 per instance)
 *   +0x0F000  Ready flag (uint32_t, set by instance 0 after init)
 *   +0x10000  PRP list[0]      (instance 0, 4 KB page)
 *   +0x11000  PRP list[1]
 *   ...
 *   +0x100000 Data buf[0]      (instance 0, 32 KB)
 *   +0x108000 Data buf[1]      (instance 1, 32 KB)
 *   ...
 * ================================================================ */
#define QREGION_BASE       0x620000000ULL
#define QOFF_ASQ           0x00000
#define QOFF_ACQ           0x01000
#define QOFF_IOSQ(i)       (0x02000 + (i) * 0x2000)
#define QOFF_IOCQ(i)       (0x03000 + (i) * 0x2000)
#define QOFF_READY_FLAG    0x0F000
#define QOFF_PRPLIST(i)    (0x10000 + (i) * 0x1000)
#define QOFF_DATA(i)       (0x100000 + (i) * 0x8000)

/* Total footprint: 0x100000 + MAX_INSTANCES * 0x8000 = ~1.5 MB */
#define NVME_SHARED_SIZE_USED  (0x100000 + MAX_INSTANCES * 0x8000)

#define IO_DATA_SIZE       0x8000   /* 32 KB */
#define LBA_SIZE           512
#define IO_NUM_LBAS        (IO_DATA_SIZE / LBA_SIZE) /* 64 */
#define PAGE_SIZE          4096
#define IO_NUM_PRPS        (IO_DATA_SIZE / PAGE_SIZE) /* 8 */

/* ================================================================
 * ORAM addresses (from phase_d_layout.py, computed per instance)
 * ================================================================ */
#define ORAM_CMD_BASE      0x0E0000000ULL
#define CMD_RING_BASE      0x600000000ULL
#define RESULT_BUF_BASE    0x610000000ULL
#define DDR_SLAB_BASE      0x700000000ULL
#define DDR_SLAB_STRIDE    0x020000000ULL   /* 512 MB per instance */

#define OFF_NUM_K              0x80
#define OFF_TOKEN_BASE         0x84
#define OFF_READY              0xC4
#define OFF_CPU_OP_COUNT       0xE0
#define OFF_CMD_RING_DOORBELL  0x150
#define OFF_GATE_RELEASE       0x180

#define RING_PROD_IDX_OFF   0x000
#define RING_CONS_IDX_OFF   0x040
#define RING_ENTRIES_OFF    0x100
#define RING_ENTRY_BYTES    64
#define CMD_RING_DEPTH      16

#define RES_STATUS_OFF      0x08
#define RES_RDATA_OFF       0x20
#define RES_DONE_BIT        (1ull << 0)
#define RES_RDATA_VALID     (1ull << 3)

#define ORAM_LEASE_BASE     0x1000
#define ORAM_SLOT_SIZE      0x1000
#define BUCKET_SIZE_BYTES   IO_DATA_SIZE     /* 32 KB */
#define LBAS_PER_SLOT       (BUCKET_SIZE_BYTES / LBA_SIZE)

/* ================================================================ */
typedef struct {
    uint8_t  opc;
    uint8_t  fuse_psdt;
    uint16_t cid;
    uint32_t nsid;
    uint64_t rsvd_2_3;
    uint64_t mptr;
    uint64_t prp1;
    uint64_t prp2;
    uint32_t cdw10;
    uint32_t cdw11;
    uint32_t cdw12;
    uint32_t cdw13;
    uint32_t cdw14;
    uint32_t cdw15;
} __attribute__((packed)) sqe_t;

typedef struct {
    uint32_t dw0;
    uint32_t dw1;
    uint16_t sq_head;
    uint16_t sq_id;
    uint16_t cid;
    uint16_t status;
} __attribute__((packed)) cqe_t;

#define CQE_PHASE(cqe)  ((cqe)->status & 1u)
#define CQE_SC(cqe)     (((cqe)->status >> 1) & 0xff)
#define CQE_SCT(cqe)    (((cqe)->status >> 9) & 0x7)

static volatile uint8_t *bar0;

static inline uint32_t mmio_r32(uint32_t off) {
    return *(volatile uint32_t *)(bar0 + off);
}
static inline void mmio_w32(uint32_t off, uint32_t v) {
    *(volatile uint32_t *)(bar0 + off) = v;
}
static inline uint64_t mmio_r64(uint32_t off) {
    uint32_t lo = mmio_r32(off);
    uint32_t hi = mmio_r32(off + 4);
    return ((uint64_t)hi << 32) | lo;
}
static inline void mmio_w64(uint32_t off, uint64_t v) {
    mmio_w32(off, (uint32_t)v);
    mmio_w32(off + 4, (uint32_t)(v >> 32));
}

static int wait_csts(uint32_t mask, uint32_t want, const char *label) {
    for (int i = 0; i < 100000; i++) {
        if ((mmio_r32(NVME_REG_CSTS) & mask) == want) return 0;
    }
    se_eputs("FAIL wait_csts "); se_eputs(label); se_eputs("\n");
    return -1;
}

/* Admin CQ poll — used only during init by instance 0. */
static int admin_cq_wait(volatile cqe_t *acq, uint16_t expected_cid,
                         uint16_t *head, uint8_t *phase,
                         const char *label) {
    for (int i = 0; i < POLL_MAX_ITERS; i++) {
        volatile cqe_t *cqe = &acq[*head];
        if (CQE_PHASE(cqe) == *phase) {
            uint16_t cid = cqe->cid;
            uint8_t sc = CQE_SC(cqe);
            *head = (*head + 1) % ADMIN_QUEUE_ENTRIES;
            if (*head == 0) *phase ^= 1;
            mmio_w32(NVME_DBELL_CQ(0), *head);
            if (cid != expected_cid || sc != 0) {
                se_eputs("|||FAIL admin "); se_eputs(label); se_eputs("\n");
                return -1;
            }
            return 0;
        }
    }
    se_eputs("|||FAIL admin timeout "); se_eputs(label); se_eputs("\n");
    return -1;
}

/* I/O CQ poll — per-instance, uses instance's own queue pair. */
static int io_cq_wait(volatile cqe_t *iocq, uint16_t expected_cid,
                      uint16_t *head, uint8_t *phase,
                      uint32_t cq_dbell_off, const char *label) {
    for (int i = 0; i < POLL_MAX_ITERS; i++) {
        (void)mmio_r32(NVME_REG_CSTS);  /* keep tick advancing */
        volatile cqe_t *cqe = &iocq[*head];

        if (i > 0 && (i % 200000) == 0) {
            se_eputs("|||"); se_eputs(label); se_eputs("_poll_prog\n");
        }

        if (CQE_PHASE(cqe) == *phase) {
            uint16_t cid = cqe->cid;
            uint8_t sc = CQE_SC(cqe);
            *head = (*head + 1) % IO_QUEUE_ENTRIES;
            if (*head == 0) *phase ^= 1;
            mmio_w32(cq_dbell_off, *head);
            if (cid != expected_cid || sc != 0) {
                se_eputs("|||FAIL io "); se_eputs(label); se_eputs("\n");
                return -1;
            }
            return i;
        }
    }
    se_eputs("|||FAIL io timeout "); se_eputs(label); se_eputs("\n");
    return -1;
}


/* Drain CQ until both wanted CIDs consumed (0 = skip that slot).
 * Handles out-of-order: Rd and Wr CQEs can arrive in either order. */
static int drain_cqes(volatile cqe_t *iocq, uint16_t *head, uint8_t *phase,
                      uint32_t cq_dbell, uint16_t want_a, uint16_t want_b) {
    int got_a = (want_a == 0);
    int got_b = (want_b == 0);
    for (int i = 0; i < POLL_MAX_ITERS && (!got_a || !got_b); i++) {
        (void)mmio_r32(NVME_REG_CSTS);
        volatile cqe_t *e = &iocq[*head];
        if (CQE_PHASE(e) == *phase) {
            uint16_t cid = e->cid;
            uint8_t sc = CQE_SC(e);
            *head = (*head + 1) % IO_QUEUE_ENTRIES;
            if (*head == 0) *phase ^= 1;
            mmio_w32(cq_dbell, *head);
            if (sc != 0) {
                se_eputs("|||FAIL drain sc!=0\n");
                return -1;
            }
            if (cid == want_a) got_a = 1;
            else if (cid == want_b) got_b = 1;
        }
    }
    if (!got_a || !got_b) {
        se_eputs("|||FAIL drain timeout\n");
        return -1;
    }
    return 0;
}

/* Submit one NVMe I/O command (read or write) */
static void submit_nvme_io(sqe_t *iosq, uint16_t *sq_tail, uint32_t sq_dbell,
                           uint8_t opc, uint16_t cid,
                           uint64_t prp1, uint64_t prp2,
                           uint64_t lba, uint32_t nlba_m1) {
    sqe_t e = {0};
    e.opc = opc;
    e.cid = cid;
    e.nsid = 1;
    e.prp1 = prp1;
    e.prp2 = prp2;
    e.cdw10 = (uint32_t)(lba & 0xFFFFFFFFu);
    e.cdw11 = (uint32_t)((lba >> 32) & 0xFFFFFFFFu);
    e.cdw12 = nlba_m1;
    iosq[*sq_tail] = e;
    *sq_tail = (*sq_tail + 1) % IO_QUEUE_ENTRIES;
    mmio_w32(sq_dbell, *sq_tail);
}

int main(int argc, char **argv) {
    se_puts("oram_nvme_test: starting\n");

    /* ---- Parse argv (matches config script format) ---- */
    int N           = (argc > 1) ? atoi(argv[1]) : 1;
    int instance_id = (argc > 2) ? atoi(argv[2]) : 0;
    int n_iters     = (argc > 3) ? atoi(argv[3]) : 10;
    int num_slots   = (argc > 4) ? atoi(argv[4]) : 32;
    uint64_t bar0_addr = (argc > 5) ? strtoull(argv[5], NULL, 0) : 0xF0000000ULL;
    uint64_t max_lba   = (argc > 6) ? strtoull(argv[6], NULL, 0) : 1048576ULL;

    if (N < 1 || N > MAX_INSTANCES || instance_id < 0 || instance_id >= N) {
        se_eputs("FAIL: bad N or instance_id\n"); return 1;
    }

    se_printf_uint("|||N", (uint64_t)N);
    se_printf_uint("|||instance_id", (uint64_t)instance_id);
    se_printf_uint("|||n_iters", (uint64_t)n_iters);
    se_printf_uint("|||num_slots", (uint64_t)num_slots);
    se_printf_hex("|||bar0_addr", bar0_addr);
    se_printf_uint("|||max_lba_per_inst", max_lba);

    bar0 = (volatile uint8_t *)bar0_addr;

    /* ---- Per-instance addresses (from phase_d_layout.py) ---- */
    uint64_t oram_cmd_base   = ORAM_CMD_BASE   + (uint64_t)instance_id * 0x1000ULL;
    uint64_t cmd_ring_base   = CMD_RING_BASE   + (uint64_t)instance_id * 0x1000ULL;
    uint64_t result_buf_base = RESULT_BUF_BASE + (uint64_t)instance_id * 0x1000000ULL;
    uint64_t oram_host_base  = DDR_SLAB_BASE   + (uint64_t)instance_id * DDR_SLAB_STRIDE;
    uint64_t inst_lba_base   = (uint64_t)instance_id * max_lba;

    se_printf_hex("|||oram_cmd_base", oram_cmd_base);
    se_printf_hex("|||oram_host_base", oram_host_base);
    se_printf_uint("|||inst_lba_base", inst_lba_base);

    /* ---- Shared NVMe pointers ---- */
    sqe_t    *asq     = (sqe_t    *)(uintptr_t)(QREGION_BASE + QOFF_ASQ);
    volatile cqe_t    *acq     = (volatile cqe_t    *)(uintptr_t)(QREGION_BASE + QOFF_ACQ);

    /* ---- Per-instance NVMe pointers ---- */
    sqe_t    *iosq    = (sqe_t    *)(uintptr_t)(QREGION_BASE + QOFF_IOSQ(instance_id));
    volatile cqe_t    *iocq    = (volatile cqe_t    *)(uintptr_t)(QREGION_BASE + QOFF_IOCQ(instance_id));
    uint64_t *prplist = (uint64_t *)(uintptr_t)(QREGION_BASE + QOFF_PRPLIST(instance_id));
    uint8_t  *data    = (uint8_t  *)(uintptr_t)(QREGION_BASE + QOFF_DATA(instance_id));

    uint32_t my_qid     = (uint32_t)instance_id + 1;
    uint32_t sq_dbell   = NVME_DBELL_SQ(my_qid);
    uint32_t cq_dbell   = NVME_DBELL_CQ(my_qid);

    volatile uint32_t *ready_flag =
        (volatile uint32_t *)(uintptr_t)(QREGION_BASE + QOFF_READY_FLAG);

    /* ================================================================
     * Instance 0: NVMe init — enable controller, create N queue pairs.
     * Other instances: spin on ready flag.
     * ================================================================ */
    if (instance_id == 0) {
        /* Zero ALL shared pages */
        for (uint64_t off = 0; off < NVME_SHARED_SIZE_USED; off += 4096) {
            volatile uint8_t *p = (volatile uint8_t *)(uintptr_t)(QREGION_BASE + off);
            *p = 0; (void)*p;
        }
        *ready_flag = 0;
        se_puts("instance 0: pages touched, starting NVMe init\n");

        /* Enable controller */
        uint64_t cap = mmio_r64(NVME_REG_CAP);
        if (cap == 0) { se_eputs("FAIL: CAP=0\n"); return 1; }

        uint32_t cc = mmio_r32(NVME_REG_CC);
        if (cc & CC_EN) {
            mmio_w32(NVME_REG_CC, cc & ~CC_EN);
            if (wait_csts(CSTS_RDY, 0, "RDY=0")) return 1;
        }

        mmio_w32(NVME_REG_AQA,
                 ((ADMIN_QUEUE_ENTRIES - 1) << 16) | (ADMIN_QUEUE_ENTRIES - 1));
        mmio_w64(NVME_REG_ASQ, (uint64_t)(uintptr_t)asq);
        mmio_w64(NVME_REG_ACQ, (uint64_t)(uintptr_t)acq);
        mmio_w32(NVME_REG_CC, CC_IOSQES_64B | CC_IOCQES_16B | CC_EN);
        if (wait_csts(CSTS_RDY, CSTS_RDY, "RDY=1")) return 1;
        se_puts("controller enabled\n");

        uint16_t admin_sq_tail = 0;
        uint16_t admin_cq_head = 0;
        uint8_t  admin_cq_phase = 1;

        /* Create N I/O queue pairs (CQ first, then SQ per NVMe spec) */
        for (int inst = 0; inst < N; inst++) {
            uint32_t qid = (uint32_t)inst + 1;
            volatile cqe_t *inst_iocq = (volatile cqe_t *)(uintptr_t)(QREGION_BASE + QOFF_IOCQ(inst));
            sqe_t *inst_iosq = (sqe_t *)(uintptr_t)(QREGION_BASE + QOFF_IOSQ(inst));

            /* Zero the queues */
            memset((void*)inst_iocq, 0, IO_QUEUE_ENTRIES * sizeof(cqe_t));
            memset(inst_iosq, 0, IO_QUEUE_ENTRIES * sizeof(sqe_t));

            /* Create I/O CQ */
            {
                sqe_t e = {0};
                e.opc  = ADMIN_OPC_CREATE_CQ;
                e.cid  = (uint16_t)(0x0C00 + qid);
                e.prp1 = (uint64_t)(uintptr_t)inst_iocq;
                e.cdw10 = ((uint32_t)(IO_QUEUE_ENTRIES - 1) << 16) | qid;
                e.cdw11 = 1; /* PC=1 */
                asq[admin_sq_tail] = e;
                admin_sq_tail = (admin_sq_tail + 1) % ADMIN_QUEUE_ENTRIES;
                mmio_w32(NVME_DBELL_SQ(0), admin_sq_tail);
                if (admin_cq_wait(acq, e.cid, &admin_cq_head,
                                  &admin_cq_phase, "create_cq") < 0)
                    return 1;
            }

            /* Create I/O SQ, linked to CQ of same qid */
            {
                sqe_t e = {0};
                e.opc  = ADMIN_OPC_CREATE_SQ;
                e.cid  = (uint16_t)(0x0500 + qid);
                e.prp1 = (uint64_t)(uintptr_t)inst_iosq;
                e.cdw10 = ((uint32_t)(IO_QUEUE_ENTRIES - 1) << 16) | qid;
                e.cdw11 = (qid << 16) | 1; /* CQID=qid, PC=1 */
                asq[admin_sq_tail] = e;
                admin_sq_tail = (admin_sq_tail + 1) % ADMIN_QUEUE_ENTRIES;
                mmio_w32(NVME_DBELL_SQ(0), admin_sq_tail);
                if (admin_cq_wait(acq, e.cid, &admin_cq_head,
                                  &admin_cq_phase, "create_sq") < 0)
                    return 1;
            }
            se_printf_uint("|||queue_pair_created", (uint64_t)qid);
        }

        /* Signal other instances */
        __asm__ __volatile__("" ::: "memory");
        *ready_flag = 0xDEADBEEFu;
        se_puts("instance 0: NVMe init complete, ready flag set\n");

    } else {
        /* Non-zero instances: poll ready flag */
        se_puts("waiting for NVMe ready flag\n");
        for (int i = 0; i < 100000000; i++) {
            if (*ready_flag == 0xDEADBEEFu) break;
        }
        if (*ready_flag != 0xDEADBEEFu) {
            se_eputs("|||FAIL NVMe ready flag timeout\n");
            return 1;
        }
        se_puts("NVMe ready flag seen\n");
    }

    /* ================================================================
     * Build per-instance PRP list.
     * 32 KB = 8 pages.  PRP1 = data[0..4KB), PRP2 → 7-entry list.
     * ================================================================ */
    for (int i = 0; i < IO_NUM_PRPS - 1; i++) {
        prplist[i] = (uint64_t)(uintptr_t)data + (i + 1) * PAGE_SIZE;
    }

    uint16_t io_sq_tail = 0;
    uint16_t io_cq_head = 0;
    uint8_t  io_cq_phase = 1;

    /* ================================================================
     * NVMe smoke test: Write + Read 32 KB at inst_lba_base.
     * Non-fatal on verify mismatch — proves the queue pair works.
     * ================================================================ */
    /* Write pattern */
    for (uint32_t i = 0; i < IO_DATA_SIZE; i++) data[i] = (uint8_t)(i & 0xff);

    {
        sqe_t e = {0};
        e.opc = IO_OPC_WRITE; e.cid = 0xBEEF; e.nsid = 1;
        e.prp1 = (uint64_t)(uintptr_t)data;
        e.prp2 = (uint64_t)(uintptr_t)prplist;
        e.cdw10 = (uint32_t)(inst_lba_base & 0xFFFFFFFFu);
        e.cdw11 = (uint32_t)((inst_lba_base >> 32) & 0xFFFFFFFFu);
        e.cdw12 = IO_NUM_LBAS - 1;
        iosq[io_sq_tail] = e;
        io_sq_tail = (io_sq_tail + 1) % IO_QUEUE_ENTRIES;
        mmio_w32(sq_dbell, io_sq_tail);
        if (io_cq_wait(iocq, 0xBEEF, &io_cq_head, &io_cq_phase,
                       cq_dbell, "smoke_write") < 0)
            return 1;
    }
    memset(data, 0, IO_DATA_SIZE);
    {
        sqe_t e = {0};
        e.opc = IO_OPC_READ; e.cid = 0xCAFE; e.nsid = 1;
        e.prp1 = (uint64_t)(uintptr_t)data;
        e.prp2 = (uint64_t)(uintptr_t)prplist;
        e.cdw10 = (uint32_t)(inst_lba_base & 0xFFFFFFFFu);
        e.cdw11 = (uint32_t)((inst_lba_base >> 32) & 0xFFFFFFFFu);
        e.cdw12 = IO_NUM_LBAS - 1;
        iosq[io_sq_tail] = e;
        io_sq_tail = (io_sq_tail + 1) % IO_QUEUE_ENTRIES;
        mmio_w32(sq_dbell, io_sq_tail);
        if (io_cq_wait(iocq, 0xCAFE, &io_cq_head, &io_cq_phase,
                       cq_dbell, "smoke_read") < 0)
            return 1;
    }

    uint32_t mismatches = 0;
    for (uint32_t i = 0; i < IO_DATA_SIZE; i++) {
        if (data[i] != (uint8_t)(i & 0xff)) mismatches++;
    }
    se_printf_uint("|||nvme_smoke_mismatches", (uint64_t)mismatches);
    if (mismatches) se_eputs("|||WARN NVMe smoke mismatch (continuing)\n");
    else            se_puts("|||NVMe smoke OK\n");

    /* ================================================================
     * Release ORAM gate, wait for readiness.
     * ================================================================ */
    se_puts("|||releasing ORAM gate\n");
    *(volatile uint32_t *)(uintptr_t)(oram_cmd_base + OFF_GATE_RELEASE) = 1;

    {
        const int MAX_POLLS = 50 * 1000 * 1000;
        int p = 0;
        while (p < MAX_POLLS) {
            if (*(volatile uint32_t *)(uintptr_t)(oram_cmd_base + OFF_READY) & 0x1)
                break;
            p++;
        }
        if (p >= MAX_POLLS) { se_eputs("|||FAIL ORAM not ready\n"); return 1; }
        se_printf_uint("|||oram_ready_polls", (uint64_t)p);
    }

    uint32_t K = *(volatile uint32_t *)(uintptr_t)(oram_cmd_base + OFF_NUM_K);
    if (K == 0 || K > 16) { se_eputs("|||FAIL invalid K\n"); return 1; }

    uint32_t lease_tokens[16] = {0};
    for (uint32_t k = 0; k < K; k++) {
        lease_tokens[k] = *(volatile uint32_t *)(uintptr_t)
            (oram_cmd_base + OFF_TOKEN_BASE + k * 4);
    }
    uint32_t slots_per_client = (uint32_t)num_slots / K;
    if (slots_per_client == 0) { se_eputs("|||FAIL slots<K\n"); return 1; }

    se_printf_uint("|||K", (uint64_t)K);
    se_printf_uint("|||slots_per_client", (uint64_t)slots_per_client);

    /* Init cmd_ring header */
    *(volatile uint64_t *)(uintptr_t)(cmd_ring_base + RING_PROD_IDX_OFF) = 0;
    *(volatile uint64_t *)(uintptr_t)(cmd_ring_base + RING_CONS_IDX_OFF) = 0;

    /* ================================================================
     * TWO-PASS PIPELINED ORAM + NVMe staging — matches test_random.c.
     *
     * Pass 1: All Writes (N iterations)
     *   NVMe Read bucket → ORAM Write → NVMe Write bucket
     * Reset PRNG
     * Pass 2: All Reads + Verify (N iterations)
     *   NVMe Read bucket → ORAM Read → verify → NVMe Write bucket
     *
     * Both passes use pipeline: Rd(N+1) overlaps ORAM(N) + Wr(N).
     * Two-pass forces data eviction from stash, triggering AES-GCM
     * decrypt on reads — matching test_random.c behavior exactly.
     * ================================================================ */
    int iter_pass = 0, iter_fail = 0;
    uint32_t rng_init = (uint32_t)(instance_id + 1) * 2654435761u;
    uint32_t rng = rng_init;

    /* Two PRP list regions within the same 4KB page */
    uint64_t *prplist_rd = prplist;
    uint64_t *prplist_wr = prplist + 16;  /* 128 bytes offset */

    #define BUILD_PRP(pl, slab) do { \
        for (int _i = 0; _i < IO_NUM_PRPS - 1; _i++) \
            (pl)[_i] = (slab) + (uint64_t)(_i + 1) * PAGE_SIZE; \
    } while(0)

    /* Helper: compute slot info from rng state */
    #define COMPUTE_SLOT(rng_v, s_slot, s_client, s_lease, s_token, \
                         s_oram, s_bucket, s_lba, s_slab) do { \
        (rng_v) ^= (rng_v) << 13; (rng_v) ^= (rng_v) >> 17; \
        (rng_v) ^= (rng_v) << 5; \
        (s_slot) = (rng_v) % (uint32_t)num_slots; \
        (s_client) = (s_slot) / slots_per_client; \
        if ((s_client) >= K) (s_client) = K - 1; \
        (s_lease) = (s_client) + 1; \
        (s_token) = lease_tokens[(s_client)]; \
        (s_oram) = ORAM_LEASE_BASE + (s_slot) * ORAM_SLOT_SIZE; \
        (s_bucket) = (s_slot) / 4; \
        (s_lba) = inst_lba_base + (uint64_t)(s_bucket) * LBAS_PER_SLOT; \
        (s_slab) = oram_host_base + (uint64_t)(s_bucket) * BUCKET_SIZE_BYTES; \
    } while(0)

    /* Slot info variables */
    uint32_t cur_slot, cur_client_id, cur_lease_id, cur_token;
    uint32_t cur_oram_addr, cur_bucket;
    uint64_t cur_lba, cur_slab;
    uint32_t nxt_slot, nxt_client_id, nxt_lease_id, nxt_token;
    uint32_t nxt_oram_addr, nxt_bucket;
    uint64_t nxt_lba, nxt_slab;

    /* ================================================================
     * PASS 1: All Writes
     * ================================================================ */
    /* ----------------------------------------------------------------
     * SSD INIT: bulk-copy DDR_INIT bucket data from DDR5 to SSD.
     *
     * DDR_INIT writes encrypted buckets to DDR5.  The SSD has never
     * been written, so NVMe Read would overwrite valid DDR5 data with
     * zeros — corrupting eviction buckets and causing stash overflow.
     *
     * Sequential: one NVMe Write per bucket, drain CQE, next.
     * num_slots=32768 → 8192 buckets × 32 KB = 256 MB total.
     * At ~12 µs/write, ~100 ms simulated. Wall-clock: 15-30 min
     * (gem5+Verilator overhead).
     * ---------------------------------------------------------------- */
    se_puts("|||ssd_init_start\n");
    {
        uint32_t num_buckets = (uint32_t)num_slots / 4;
        for (uint32_t b = 0; b < num_buckets; b++) {
            uint64_t slab = oram_host_base + (uint64_t)b * BUCKET_SIZE_BYTES;
            uint64_t lba  = inst_lba_base  + (uint64_t)b * LBAS_PER_SLOT;

            BUILD_PRP(prplist_wr, slab);
            uint16_t cid = (uint16_t)(0xA000u | (b & 0x0FFFu));
            submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                           IO_OPC_WRITE, cid, slab,
                           (uint64_t)(uintptr_t)prplist_wr,
                           lba, LBAS_PER_SLOT - 1);
            if (drain_cqes(iocq, &io_cq_head, &io_cq_phase, cq_dbell,
                           cid, 0) < 0) {
                se_eputs("|||FAIL ssd_init\n"); return 1;
            }

            if ((b % 100) == 0)
                se_printf_uint("|||ssd_init_bucket", (uint64_t)b);
        }
        se_printf_uint("|||ssd_init_total_buckets", (uint64_t)num_buckets);
    }
    se_puts("|||ssd_init_done\n");

    se_puts("|||pass1_writes_start\n");

    COMPUTE_SLOT(rng, cur_slot, cur_client_id, cur_lease_id, cur_token,
                 cur_oram_addr, cur_bucket, cur_lba, cur_slab);

    /* Prolog: submit and wait for first NVMe Read */
    BUILD_PRP(prplist_rd, cur_slab);
    submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                   IO_OPC_READ, (uint16_t)0xC000u, cur_slab,
                   (uint64_t)(uintptr_t)prplist_rd,
                   cur_lba, LBAS_PER_SLOT - 1);
    if (drain_cqes(iocq, &io_cq_head, &io_cq_phase, cq_dbell,
                   0xC000u, 0) < 0) {
        se_eputs("|||FAIL pass1 prolog\n"); return 1;
    }

    for (uint32_t iter = 0; iter < (uint32_t)n_iters; iter++) {
        uint64_t op = (uint64_t)iter;

        uint32_t wdata_i[8];
        for (int j = 0; j < 8; j++)
            wdata_i[j] = 0xABCD0000u | ((uint32_t)iter << 8) | (uint32_t)j;

        /* ---- ORAM Write only ---- */
        {
            uint64_t base = cmd_ring_base + RING_ENTRIES_OFF
                          + (op % CMD_RING_DEPTH) * RING_ENTRY_BYTES;
            *(volatile uint32_t *)(uintptr_t)(base + 0x00) = cur_oram_addr;
            *(volatile uint32_t *)(uintptr_t)(base + 0x04) = cur_token;
            *(volatile uint32_t *)(uintptr_t)(base + 0x08) =
                (cur_lease_id & 0xffu) | (1u << 8) | (cur_client_id << 16);
            *(volatile uint32_t *)(uintptr_t)(base + 0x0C) =
                (uint32_t)(op & 0xFFFFFFFFu);
            for (int j = 0; j < 8; j++)
                *(volatile uint32_t *)(uintptr_t)(base + 0x10 + j*4) = wdata_i[j];
            *(volatile uint32_t *)(uintptr_t)(base + 0x30) =
                (uint32_t)((op >> 32) & 0xFFFFFFFFu);
            __asm__ __volatile__("" ::: "memory");
            *(volatile uint32_t *)(uintptr_t)(base + 0x34) = 1u;
            *(volatile uint32_t *)(uintptr_t)(base + 0x38) = 0;
            *(volatile uint32_t *)(uintptr_t)(base + 0x3C) = 0;

            while ((op + 1 - *(volatile uint64_t *)(uintptr_t)
                   (cmd_ring_base + RING_CONS_IDX_OFF)) > CMD_RING_DEPTH) { }
            *(volatile uint64_t *)(uintptr_t)
                (cmd_ring_base + RING_PROD_IDX_OFF) = op + 1;
            *(volatile uint32_t *)(uintptr_t)
                (oram_cmd_base + OFF_CMD_RING_DOORBELL) = 1;
        }

        /* ---- Prefetch next NVMe Read ---- */
        uint16_t next_rd_cid = 0;
        if (iter + 1 < (uint32_t)n_iters) {
            COMPUTE_SLOT(rng, nxt_slot, nxt_client_id, nxt_lease_id,
                         nxt_token, nxt_oram_addr, nxt_bucket, nxt_lba, nxt_slab);
            BUILD_PRP(prplist_rd, nxt_slab);
            next_rd_cid = (uint16_t)(0xC000u | ((iter + 1) & 0x0FFFu));
            submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                           IO_OPC_READ, next_rd_cid, nxt_slab,
                           (uint64_t)(uintptr_t)prplist_rd,
                           nxt_lba, LBAS_PER_SLOT - 1);
        }

        /* ---- Wait ORAM Write completion ---- */
        {
            const int MAX_POLLS = 50 * 1000 * 1000;
            int p = 0;
            while (p < MAX_POLLS) {
                if (*(volatile uint32_t *)(uintptr_t)
                    (oram_cmd_base + OFF_CPU_OP_COUNT) >= op + 1) break;
                p++;
            }
            if (p >= MAX_POLLS) {
                se_eputs("|||FAIL pass1 ORAM timeout\n");
                return 1;
            }
        }

        /* ---- Submit write-back ---- */
        BUILD_PRP(prplist_wr, cur_slab);
        {
            uint16_t cur_wr_cid = (uint16_t)(0xD000u | (iter & 0x0FFFu));
            submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                           IO_OPC_WRITE, cur_wr_cid, cur_slab,
                           (uint64_t)(uintptr_t)prplist_wr,
                           cur_lba, LBAS_PER_SLOT - 1);

            if (drain_cqes(iocq, &io_cq_head, &io_cq_phase, cq_dbell,
                           next_rd_cid, cur_wr_cid) < 0) {
                se_eputs("|||FAIL pass1 drain\n"); return 1;
            }
        }

        /* Advance */
        if (iter + 1 < (uint32_t)n_iters) {
            cur_slot = nxt_slot; cur_bucket = nxt_bucket;
            cur_lba = nxt_lba; cur_slab = nxt_slab;
            cur_oram_addr = nxt_oram_addr;
            cur_client_id = nxt_client_id;
            cur_lease_id = nxt_lease_id; cur_token = nxt_token;
        }
    }
    se_puts("|||pass1_writes_done\n");

    /* ================================================================
     * PASS 2: All Reads + Verify (reset PRNG for same slot sequence)
     * ================================================================ */
    se_puts("|||pass2_reads_start\n");
    rng = rng_init;  /* reset PRNG — same slot sequence as pass 1 */

    /* Pre-compute last_writer map in O(n): for each slot, record the
     * last pass-1 iteration that wrote to it.  Handles PRNG collisions
     * where the same slot is written multiple times. */
    int32_t last_writer_map[32768];
    memset(last_writer_map, -1, sizeof(last_writer_map));
    {
        uint32_t rng_pre = rng_init;
        for (uint32_t j = 0; j < (uint32_t)n_iters; j++) {
            rng_pre = (rng_pre ^ ((rng_pre << 13) & 0xFFFFFFFFu));
            rng_pre = (rng_pre ^ (rng_pre >> 17));
            rng_pre = (rng_pre ^ ((rng_pre << 5) & 0xFFFFFFFFu));
            last_writer_map[rng_pre % (uint32_t)num_slots] = (int32_t)j;
        }
    }

    COMPUTE_SLOT(rng, cur_slot, cur_client_id, cur_lease_id, cur_token,
                 cur_oram_addr, cur_bucket, cur_lba, cur_slab);

    /* Prolog: submit and wait for first NVMe Read */
    BUILD_PRP(prplist_rd, cur_slab);
    submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                   IO_OPC_READ, (uint16_t)0xE000u, cur_slab,
                   (uint64_t)(uintptr_t)prplist_rd,
                   cur_lba, LBAS_PER_SLOT - 1);
    if (drain_cqes(iocq, &io_cq_head, &io_cq_phase, cq_dbell,
                   0xE000u, 0) < 0) {
        se_eputs("|||FAIL pass2 prolog\n"); return 1;
    }

    for (uint32_t iter = 0; iter < (uint32_t)n_iters; iter++) {
        uint64_t op = (uint64_t)n_iters + (uint64_t)iter;

        /* Lookup last writer from precomputed map */
        uint32_t last_writer = (last_writer_map[cur_slot] >= 0)
                             ? (uint32_t)last_writer_map[cur_slot] : iter;
        uint32_t wdata_i[8];
        for (int j = 0; j < 8; j++)
            wdata_i[j] = 0xABCD0000u | (last_writer << 8) | (uint32_t)j;

        /* ---- ORAM Read only ---- */
        {
            uint64_t base = cmd_ring_base + RING_ENTRIES_OFF
                          + (op % CMD_RING_DEPTH) * RING_ENTRY_BYTES;
            *(volatile uint32_t *)(uintptr_t)(base + 0x00) = cur_oram_addr;
            *(volatile uint32_t *)(uintptr_t)(base + 0x04) = cur_token;
            *(volatile uint32_t *)(uintptr_t)(base + 0x08) =
                (cur_lease_id & 0xffu) | (0u << 8) | (cur_client_id << 16);
            *(volatile uint32_t *)(uintptr_t)(base + 0x0C) =
                (uint32_t)(op & 0xFFFFFFFFu);
            for (int j = 0; j < 8; j++)
                *(volatile uint32_t *)(uintptr_t)(base + 0x10 + j*4) = 0;
            *(volatile uint32_t *)(uintptr_t)(base + 0x30) =
                (uint32_t)((op >> 32) & 0xFFFFFFFFu);
            __asm__ __volatile__("" ::: "memory");
            *(volatile uint32_t *)(uintptr_t)(base + 0x34) = 1u;
            *(volatile uint32_t *)(uintptr_t)(base + 0x38) = 0;
            *(volatile uint32_t *)(uintptr_t)(base + 0x3C) = 0;

            while ((op + 1 - *(volatile uint64_t *)(uintptr_t)
                   (cmd_ring_base + RING_CONS_IDX_OFF)) > CMD_RING_DEPTH) { }
            *(volatile uint64_t *)(uintptr_t)
                (cmd_ring_base + RING_PROD_IDX_OFF) = op + 1;
            *(volatile uint32_t *)(uintptr_t)
                (oram_cmd_base + OFF_CMD_RING_DOORBELL) = 1;
        }

        /* ---- Prefetch next NVMe Read ---- */
        uint16_t next_rd_cid = 0;
        if (iter + 1 < (uint32_t)n_iters) {
            COMPUTE_SLOT(rng, nxt_slot, nxt_client_id, nxt_lease_id,
                         nxt_token, nxt_oram_addr, nxt_bucket, nxt_lba, nxt_slab);
            BUILD_PRP(prplist_rd, nxt_slab);
            next_rd_cid = (uint16_t)(0xE000u | ((iter + 1) & 0x0FFFu));
            submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                           IO_OPC_READ, next_rd_cid, nxt_slab,
                           (uint64_t)(uintptr_t)prplist_rd,
                           nxt_lba, LBAS_PER_SLOT - 1);
        }

        /* ---- Wait ORAM Read completion ---- */
        {
            const int MAX_POLLS = 50 * 1000 * 1000;
            int p = 0;
            while (p < MAX_POLLS) {
                if (*(volatile uint32_t *)(uintptr_t)
                    (oram_cmd_base + OFF_CPU_OP_COUNT) >= op + 1) break;
                p++;
            }
            if (p >= MAX_POLLS) {
                se_eputs("|||FAIL pass2 ORAM timeout\n");
                return 1;
            }
        }

        /* ---- Verify rdata (poll DONE bit) ---- */
        uint64_t res_addr = result_buf_base + op * 64;
        uint64_t status;
        while (!((status = *(volatile uint64_t *)
                  (uintptr_t)(res_addr + RES_STATUS_OFF)) & RES_DONE_BIT)) {
            for (volatile int d = 0; d < 100; d++) {}
        }
        if (!(status & RES_RDATA_VALID)) {
            iter_fail++; goto next_iter_p2;
        }
        uint32_t r0 = *(volatile uint32_t *)(uintptr_t)(res_addr + RES_RDATA_OFF + 0);
        uint32_t r1 = *(volatile uint32_t *)(uintptr_t)(res_addr + RES_RDATA_OFF + 4);
        if (r0 == wdata_i[0] && r1 == wdata_i[1]) iter_pass++;
        else { iter_fail++;
            se_printf_uint("|||iter_mismatch", (uint64_t)iter);
            se_printf_hex("|||  got_r0", r0);
            se_printf_hex("|||  exp_r0", wdata_i[0]);
        }

        /* ---- Submit write-back (ORAM modifies bucket on every access) ---- */
        BUILD_PRP(prplist_wr, cur_slab);
        {
            uint16_t cur_wr_cid = (uint16_t)(0xF000u | (iter & 0x0FFFu));
            submit_nvme_io(iosq, &io_sq_tail, sq_dbell,
                           IO_OPC_WRITE, cur_wr_cid, cur_slab,
                           (uint64_t)(uintptr_t)prplist_wr,
                           cur_lba, LBAS_PER_SLOT - 1);

            if (drain_cqes(iocq, &io_cq_head, &io_cq_phase, cq_dbell,
                           next_rd_cid, cur_wr_cid) < 0) {
                se_eputs("|||FAIL pass2 drain\n"); return 1;
            }
        }

next_iter_p2:
        /* Advance */
        if (iter + 1 < (uint32_t)n_iters) {
            cur_slot = nxt_slot; cur_bucket = nxt_bucket;
            cur_lba = nxt_lba; cur_slab = nxt_slab;
            cur_oram_addr = nxt_oram_addr;
            cur_client_id = nxt_client_id;
            cur_lease_id = nxt_lease_id; cur_token = nxt_token;
        }
    }
    se_puts("|||pass2_reads_done\n");

    #undef BUILD_PRP
    #undef COMPUTE_SLOT

    se_printf_uint("|||iter_pass", (uint64_t)iter_pass);
    se_printf_uint("|||iter_fail", (uint64_t)iter_fail);
    if (iter_fail) { se_eputs("|||FAIL ORAM mismatch\n"); return 1; }
    se_puts("|||PASS oram_nvme_test\n");
    return 0;
}
