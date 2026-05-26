/*
 * nvme_io_test.c — NVMe I/O Read/Write smoke test.
 *
 * After Identify (proven working in nvme_identify), this test:
 *   1. Initializes the controller (same as Identify)
 *   2. Creates an I/O Completion Queue   (admin opcode 0x05)
 *   3. Creates an I/O Submission Queue   (admin opcode 0x01)
 *   4. NVMe Write of 32 KB at LBA 0      (I/O opcode 0x01)
 *   5. NVMe Read  of 32 KB from LBA 0    (I/O opcode 0x02)
 *   6. Verifies read-back matches written pattern.
 *
 * LBA size = 512 B (from sample.cfg). 32 KB = 64 LBAs.
 * 32 KB at 4 KB pages = 8 PRP entries. PRP1 holds the first;
 * PRP2 points to a 7-entry PRP-list page.
 *
 * Build: gcc -O2 -Wall -static -o nvme_io_test nvme_io_test.c
 */

#include "se_io.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ============================================================== */
/* NVMe register offsets and bits                                  */
/* ============================================================== */
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

/* Memory layout (Option Z, must match NVME_SHARED_BASE in config) */
#define QREGION_BASE       0x220000000ULL
#define QREGION_ASQ        (QREGION_BASE + 0x0000)
#define QREGION_ACQ        (QREGION_BASE + 0x1000)
#define QREGION_IOSQ       (QREGION_BASE + 0x2000)
#define QREGION_IOCQ       (QREGION_BASE + 0x3000)
#define QREGION_PRPLIST    (QREGION_BASE + 0x10000)
#define QREGION_DATA       (QREGION_BASE + 0x100000)

/* Highest offset we actually use within the 16 MB NVMe shared region.
 * QREGION_DATA + 32 KB = 0x100000 + 0x8000 = 0x108000. Round up. */
#define NVME_SHARED_SIZE_USED  0x110000   /* 1.06 MB — covers all used pages */

#define IO_DATA_SIZE       0x8000   /* 32 KB */
#define LBA_SIZE           512
#define IO_NUM_LBAS        (IO_DATA_SIZE / LBA_SIZE) /* = 64 */
#define PAGE_SIZE          4096
#define IO_NUM_PRPS        (IO_DATA_SIZE / PAGE_SIZE) /* = 8 */

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

/* Poll budget. Each iteration includes one MMIO read (~382 ns
 * simulated PIO round-trip). Identify finished at iter=1000 (~382 µs
 * simulated), but Create CQ/SQ admin commands take longer in
 * SimpleSSD because they exercise queue-allocation code paths. The
 * I/O Read/Write commands additionally exercise the full ICL→FTL→PAL
 * NAND chain, which can take many ms simulated.
 *
 * 2M polls × 382 ns = 764 ms simulated polling window. Generous. */
#define POLL_MAX_ITERS  2000000

static int wait_csts(uint32_t mask, uint32_t want, const char *label) {
    for (int i = 0; i < 100000; i++) {
        uint32_t csts = mmio_r32(NVME_REG_CSTS);
        if ((csts & mask) == want) return 0;
    }
    se_eputs("FAIL wait_csts ");
    se_eputs(label);
    se_eputs("\n");
    return -1;
}

/* Wait for a CQE matching expected_cid on a specific completion queue. */
static int cq_wait(cqe_t *cq, uint16_t expected_cid,
                   uint16_t *cq_head, uint8_t *cq_phase,
                   uint32_t cq_doorbell_off, uint16_t queue_entries,
                   const char *label) {
    /* Pre-build label strings so all output is one atomic write each. */
    char lbl_cid[64], lbl_sc[64], lbl_sct[64];
    {
        int p;
        p = 0; lbl_cid[p++] = '|'; lbl_cid[p++] = '|'; lbl_cid[p++] = '|';
        for (int j = 0; label[j] && p < 56; j++) lbl_cid[p++] = label[j];
        lbl_cid[p++] = '_'; lbl_cid[p++] = 'c'; lbl_cid[p++] = 'i';
        lbl_cid[p++] = 'd'; lbl_cid[p] = '\0';

        p = 0; lbl_sc[p++] = '|'; lbl_sc[p++] = '|'; lbl_sc[p++] = '|';
        for (int j = 0; label[j] && p < 58; j++) lbl_sc[p++] = label[j];
        lbl_sc[p++] = '_'; lbl_sc[p++] = 's'; lbl_sc[p++] = 'c';
        lbl_sc[p] = '\0';

        p = 0; lbl_sct[p++] = '|'; lbl_sct[p++] = '|'; lbl_sct[p++] = '|';
        for (int j = 0; label[j] && p < 57; j++) lbl_sct[p++] = label[j];
        lbl_sct[p++] = '_'; lbl_sct[p++] = 's'; lbl_sct[p++] = 'c';
        lbl_sct[p++] = 't'; lbl_sct[p] = '\0';
    }

    for (int i = 0; i < POLL_MAX_ITERS; i++) {
        volatile uint32_t csts = mmio_r32(NVME_REG_CSTS);
        (void)csts;

        cqe_t *cqe = &cq[*cq_head];

        /* Periodic snapshot so we can debug if polling fails to see CQE.
         * Every 100K iters = ~38 ms simulated apart. Built into one
         * atomic write so it doesn't interleave. */
        if (i > 0 && (i % 100000) == 0) {
            char snap[160];
            int p = 0;
            snap[p++] = '|'; snap[p++] = '|'; snap[p++] = '|';
            for (int j = 0; label[j] && p < 60; j++) snap[p++] = label[j];
            snap[p++] = '_'; snap[p++] = 'p'; snap[p++] = 'r'; snap[p++] = 'o';
            snap[p++] = 'g'; snap[p++] = ' '; snap[p++] = 'i'; snap[p++] = '=';
            char dbuf[12]; int dp = 0; int v = i;
            while (v) { dbuf[dp++] = '0' + (v % 10); v /= 10; }
            while (dp && p < 70) snap[p++] = dbuf[--dp];
            snap[p++] = ' '; snap[p++] = 'b'; snap[p++] = 'y'; snap[p++] = 't';
            snap[p++] = 'e'; snap[p++] = 's'; snap[p++] = ':';
            uint8_t *b = (uint8_t *)cqe;
            static const char hex[] = "0123456789abcdef";
            for (int j = 0; j < 16 && p < 150; j++) {
                snap[p++] = ' ';
                snap[p++] = hex[(b[j] >> 4) & 0xf];
                snap[p++] = hex[b[j] & 0xf];
            }
            snap[p++] = '\n';
            ssize_t r __attribute__((unused)) = write(2, snap, p);
        }

        if (CQE_PHASE(cqe) == *cq_phase) {
            uint16_t got_cid = cqe->cid;
            uint8_t  sc      = CQE_SC(cqe);
            uint8_t  sct     = CQE_SCT(cqe);

            se_printf_hex(lbl_cid, got_cid);
            se_printf_hex(lbl_sc,  sc);
            se_printf_hex(lbl_sct, sct);

            *cq_head = (*cq_head + 1) % queue_entries;
            if (*cq_head == 0) *cq_phase ^= 1;
            mmio_w32(cq_doorbell_off, *cq_head);

            if (got_cid != expected_cid) {
                se_eputs("|||FAIL CID mismatch in ");
                se_eputs(label); se_eputs("\n");
                return -1;
            }
            if (sc != 0 || sct != 0) {
                se_eputs("|||FAIL non-success in ");
                se_eputs(label); se_eputs("\n");
                return -1;
            }
            return i;
        }
    }
    se_eputs("|||FAIL ");
    se_eputs(label);
    se_eputs(" timeout\n");
    return -1;
}

int main(int argc, char **argv) {
    se_puts("nvme_io_test: starting\n");

    uint64_t bar0_addr;
    if (argc >= 5) bar0_addr = strtoull(argv[4], NULL, 0);
    else if (argc >= 2) bar0_addr = strtoull(argv[1], NULL, 0);
    else bar0_addr = 0x500000000ULL;

    /* max_lba: passed by the gem5 config from DDR_PER_INSTANCE / 512.
     * Enforces fair comparison with CXL SSD (same effective storage
     * size). Smoke test only uses LBA 0..63, well within any sane
     * max_lba, so this is informational here. Future ORAM staging
     * test must respect it for the paper comparison to be valid. */
    uint64_t max_lba;
    if (argc >= 6) max_lba = strtoull(argv[5], NULL, 0);
    else max_lba = 131072; /* default: 64 MB / 512 B */

    se_printf_uint("|||max_lba", max_lba);

    bar0 = (volatile uint8_t *)bar0_addr;

    sqe_t   *asq  = (sqe_t   *)(uintptr_t)QREGION_ASQ;
    cqe_t   *acq  = (cqe_t   *)(uintptr_t)QREGION_ACQ;
    sqe_t   *iosq = (sqe_t   *)(uintptr_t)QREGION_IOSQ;
    cqe_t   *iocq = (cqe_t   *)(uintptr_t)QREGION_IOCQ;
    uint64_t *prplist = (uint64_t *)(uintptr_t)QREGION_PRPLIST;
    uint8_t *data = (uint8_t *)(uintptr_t)QREGION_DATA;

    /* ===================== STEP 0: Init ============================ */
    memset(asq,  0, ADMIN_QUEUE_ENTRIES * sizeof(sqe_t));
    memset(acq,  0, ADMIN_QUEUE_ENTRIES * sizeof(cqe_t));
    memset(iosq, 0, IO_QUEUE_ENTRIES * sizeof(sqe_t));
    memset(iocq, 0, IO_QUEUE_ENTRIES * sizeof(cqe_t));
    memset(prplist, 0, 4096);
    memset(data, 0, IO_DATA_SIZE);

    /* Force-touch every 4 KB page in the NVMe shared region with a
     * volatile write. memset alone may not guarantee SE-mode page
     * allocation for every VA, especially mid-region pages. This
     * eager-allocates pages so subsequent device DMAs land in the
     * same physical cells the binary reads. */
    for (uint64_t page = 0; page < NVME_SHARED_SIZE_USED; page += 4096) {
        volatile uint8_t *p = (volatile uint8_t *)(QREGION_BASE + page);
        *p = 0;  /* write triggers allocation */
        (void)*p;  /* read confirms we own it */
    }
    se_puts("queues + buffers zeroed and page-touched\n");

    /* ===================== STEP 1: Enable controller =============== */
    uint64_t cap = mmio_r64(NVME_REG_CAP);
    if (cap == 0) { se_eputs("FAIL: CAP=0\n"); return 1; }

    uint32_t cc = mmio_r32(NVME_REG_CC);
    if (cc & CC_EN) {
        mmio_w32(NVME_REG_CC, cc & ~CC_EN);
        if (wait_csts(CSTS_RDY, 0, "RDY=0 (initial)")) return 1;
    }

    uint32_t aqa = ((ADMIN_QUEUE_ENTRIES - 1) << 16) | (ADMIN_QUEUE_ENTRIES - 1);
    mmio_w32(NVME_REG_AQA, aqa);
    mmio_w64(NVME_REG_ASQ, (uint64_t)(uintptr_t)asq);
    mmio_w64(NVME_REG_ACQ, (uint64_t)(uintptr_t)acq);

    cc = CC_IOSQES_64B | CC_IOCQES_16B | CC_EN;
    mmio_w32(NVME_REG_CC, cc);
    if (wait_csts(CSTS_RDY, CSTS_RDY, "RDY=1")) return 1;
    se_puts("controller enabled\n");

    uint16_t admin_sq_tail = 0;
    uint16_t admin_cq_head = 0;
    uint8_t  admin_cq_phase = 1;

    /* ===================== STEP 2: Create I/O CQ =================== */
    se_puts("creating I/O CQ (qid=1)\n");
    {
        sqe_t e = {0};
        e.opc   = ADMIN_OPC_CREATE_CQ;
        e.cid   = 0x0CA1;
        e.prp1  = (uint64_t)(uintptr_t)iocq;
        e.cdw10 = ((uint32_t)(IO_QUEUE_ENTRIES - 1) << 16) | 1;
        e.cdw11 = 1; /* PC=1, IEN=0, IV=0 */
        asq[admin_sq_tail] = e;
        admin_sq_tail = (admin_sq_tail + 1) % ADMIN_QUEUE_ENTRIES;
        mmio_w32(NVME_DBELL_SQ(0), admin_sq_tail);

        if (cq_wait(acq, 0x0CA1, &admin_cq_head, &admin_cq_phase,
                    NVME_DBELL_CQ(0), ADMIN_QUEUE_ENTRIES, "create_cq") < 0)
            return 1;
    }

    /* ===================== STEP 3: Create I/O SQ =================== */
    se_puts("creating I/O SQ (qid=1, cqid=1)\n");
    {
        sqe_t e = {0};
        e.opc   = ADMIN_OPC_CREATE_SQ;
        e.cid   = 0x05A1;
        e.prp1  = (uint64_t)(uintptr_t)iosq;
        e.cdw10 = ((uint32_t)(IO_QUEUE_ENTRIES - 1) << 16) | 1;
        e.cdw11 = ((uint32_t)1 << 16) | 1; /* CQID=1, PC=1 */
        asq[admin_sq_tail] = e;
        admin_sq_tail = (admin_sq_tail + 1) % ADMIN_QUEUE_ENTRIES;
        mmio_w32(NVME_DBELL_SQ(0), admin_sq_tail);

        if (cq_wait(acq, 0x05A1, &admin_cq_head, &admin_cq_phase,
                    NVME_DBELL_CQ(0), ADMIN_QUEUE_ENTRIES, "create_sq") < 0)
            return 1;
    }
    se_puts("I/O SQ + CQ created\n");

    /* ===================== STEP 4: PRP list ======================= */
    /* 32 KB at 4 KB pages = 8 PRPs. PRP1 = data[0..4KB).
     * PRP2 = pointer to a list of the remaining 7 PRPs. */
    for (int i = 0; i < IO_NUM_PRPS - 1; i++) {
        prplist[i] = (uint64_t)(uintptr_t)data + (i + 1) * PAGE_SIZE;
    }
    se_puts("PRP list built\n");

    uint16_t io_sq_tail = 0;
    uint16_t io_cq_head = 0;
    uint8_t  io_cq_phase = 1;

    /* ===================== STEP 5: Write =========================== */
    for (uint32_t i = 0; i < IO_DATA_SIZE; i++) data[i] = (uint8_t)(i & 0xff);
    se_puts("data buffer pattern filled\n");

    /* Verify LBA range fits within the per-instance bound. */
    if ((uint64_t)IO_NUM_LBAS > max_lba) {
        se_eputs("FAIL: test LBA range exceeds max_lba\n");
        return 1;
    }

    se_puts("submitting NVMe Write (LBA=0, NLB=64)\n");
    {
        sqe_t e = {0};
        e.opc   = IO_OPC_WRITE;
        e.cid   = 0xBEEF;
        e.nsid  = 1;
        e.prp1  = (uint64_t)(uintptr_t)data;
        e.prp2  = (uint64_t)(uintptr_t)prplist;
        e.cdw10 = 0; /* SLBA low */
        e.cdw11 = 0; /* SLBA high */
        e.cdw12 = IO_NUM_LBAS - 1; /* NLB-1 (zero-based) */
        iosq[io_sq_tail] = e;
        io_sq_tail = (io_sq_tail + 1) % IO_QUEUE_ENTRIES;
        mmio_w32(NVME_DBELL_SQ(1), io_sq_tail);

        int polls = cq_wait(iocq, 0xBEEF, &io_cq_head, &io_cq_phase,
                            NVME_DBELL_CQ(1), IO_QUEUE_ENTRIES, "write");
        if (polls < 0) return 1;
        se_printf_uint("|||write_polls", (uint64_t)polls);
    }
    se_puts("Write completed\n");

    /* Clear the buffer so we can verify Read fills it. */
    memset(data, 0, IO_DATA_SIZE);

    /* ===================== STEP 6: Read ============================ */
    se_puts("submitting NVMe Read (LBA=0, NLB=64)\n");
    {
        sqe_t e = {0};
        e.opc   = IO_OPC_READ;
        e.cid   = 0xCAFE;
        e.nsid  = 1;
        e.prp1  = (uint64_t)(uintptr_t)data;
        e.prp2  = (uint64_t)(uintptr_t)prplist;
        e.cdw10 = 0;
        e.cdw11 = 0;
        e.cdw12 = IO_NUM_LBAS - 1;
        iosq[io_sq_tail] = e;
        io_sq_tail = (io_sq_tail + 1) % IO_QUEUE_ENTRIES;
        mmio_w32(NVME_DBELL_SQ(1), io_sq_tail);

        int polls = cq_wait(iocq, 0xCAFE, &io_cq_head, &io_cq_phase,
                            NVME_DBELL_CQ(1), IO_QUEUE_ENTRIES, "read");
        if (polls < 0) return 1;
        se_printf_uint("|||read_polls", (uint64_t)polls);
    }
    se_puts("Read completed\n");

    /* ===================== STEP 7: Verify ========================= */
    uint32_t mismatches = 0;
    uint32_t first_mismatch_offset = 0xffffffff;
    for (uint32_t i = 0; i < IO_DATA_SIZE; i++) {
        uint8_t expected = (uint8_t)(i & 0xff);
        if (data[i] != expected) {
            if (mismatches == 0) first_mismatch_offset = i;
            mismatches++;
        }
    }

    se_printf_uint("|||mismatches", (uint64_t)mismatches);
    if (mismatches != 0) {
        se_printf_uint("|||first_mismatch_at", (uint64_t)first_mismatch_offset);
        se_dump_hex("|||data[0..15]", data, 16);
        se_dump_hex("|||data[mm..+15]",
                    data + first_mismatch_offset, 16);
        se_eputs("|||FAIL data mismatch\n");
        return 1;
    }

    se_puts("|||PASS nvme_io_test\n");
    return 0;
}
