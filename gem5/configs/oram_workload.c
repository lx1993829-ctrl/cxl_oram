// =========================================================================
// test_random.c — random writes then random reads, with slot reuse.
//
// Pass 1: N random writes to randomly-chosen slots (may hit same slot
//         multiple times). Data keyed by write iteration i.
// Pass 2: N random reads to randomly-chosen slots from the written set.
//         Each read must return the LAST write's data for that slot.
//
// This exercises stash/bucket placement under random access patterns,
// unlike test_permutation which writes each slot exactly once.
//
// argv: <N> <instance_id> [n_iters] [num_slots]
//
// Build:
//   musl-gcc -O0 -static -o test_random test_random.c
// =========================================================================

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ORAM_CMD_BASE     0x0E0000000ULL
#define CMD_RING_BASE     0x600000000ULL
#define RESULT_BUF_BASE   0x610000000ULL

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

#define RING_PROD_IDX_OFF      0x000
#define RING_CONS_IDX_OFF      0x040
#define RING_ENTRIES_OFF       0x100
#define RING_ENTRY_BYTES       64
#define CMD_RING_DEPTH         16

#define RES_STATUS_OFF         0x08
#define RES_RDATA_OFF          0x20
#define RES_DONE_BIT           (1ull << 0)
#define RES_RDATA_VALID        (1ull << 3)

#define DEFAULT_N_ITERS        100
#define DEFAULT_NUM_SLOTS      128
#define MAX_SLOTS              32768
#define MAX_ITERS              131072

static inline uint32_t rd32(uint64_t a){return *(volatile uint32_t*)(uintptr_t)a;}
static inline void     wr32(uint64_t a,uint32_t v){*(volatile uint32_t*)(uintptr_t)a=v;}
static inline uint64_t rd64(uint64_t a){return *(volatile uint64_t*)(uintptr_t)a;}
static inline void     wr64(uint64_t a,uint64_t v){*(volatile uint64_t*)(uintptr_t)a=v;}

static uint32_t poll_mmio32(uint64_t addr,uint32_t mask,int maxIter){
    for(int i=0;i<maxIter;i++){uint32_t v=rd32(addr);if(v&mask)return v;}
    return 0;
}
static uint32_t xorshift32(uint32_t *s){
    uint32_t x=*s; x^=x<<13; x^=x>>17; x^=x<<5; *s=x; return x;
}

static inline uint32_t payload_word(uint32_t i,int j){
    return 0xABCD0000u | (i<<8) | (uint32_t)j;
}

static void write_ring_entry(uint64_t ring_base,uint64_t opIdx,uint32_t slot,
                             uint32_t token,uint8_t lease,uint8_t op,
                             uint8_t hwid,const uint32_t *wdata){
    uint64_t base=ring_base+RING_ENTRIES_OFF+(opIdx%CMD_RING_DEPTH)*RING_ENTRY_BYTES;
    wr32(base+0x00,slot);
    wr32(base+0x04,token);
    wr32(base+0x08,(uint32_t)lease|((uint32_t)op<<8)|((uint32_t)hwid<<16));
    wr32(base+0x0C,(uint32_t)(opIdx&0xFFFFFFFFu));
    if(op==1&&wdata){ for(int j=0;j<8;j++) wr32(base+0x10+j*4,wdata[j]); }
    else            { for(int j=0;j<8;j++) wr32(base+0x10+j*4,0); }
    wr32(base+0x30,(uint32_t)((opIdx>>32)&0xFFFFFFFFu));
    __asm__ __volatile__("":::"memory");
    wr32(base+0x34,1u); wr32(base+0x38,0); wr32(base+0x3C,0);
}

int main(int argc,char**argv){
    int N=(argc>1)?atoi(argv[1]):1;
    int instance_id=(argc>2)?atoi(argv[2]):0;
    int n_iters=(argc>3)?atoi(argv[3]):DEFAULT_N_ITERS;
    int num_slots=(argc>4)?atoi(argv[4]):DEFAULT_NUM_SLOTS;
    (void)N;

    if(instance_id<0||instance_id>=32){fprintf(stderr,"bad instance_id\n");return 1;}
    if(n_iters>MAX_ITERS){fprintf(stderr,"n_iters %d exceeds MAX_ITERS %d\n",n_iters,MAX_ITERS);return 1;}
    if(num_slots<1||num_slots>MAX_SLOTS){fprintf(stderr,"bad num_slots\n");return 1;}

    uint64_t cmd_base=ORAM_CMD_BASE+(uint64_t)instance_id*0x1000ULL;
    uint64_t ring_base=CMD_RING_BASE+(uint64_t)instance_id*0x1000ULL;
    uint64_t result_base=RESULT_BUF_BASE+(uint64_t)instance_id*0x1000000ULL;
    const uint32_t TOTAL_OPS=2u*(uint32_t)n_iters;
    uint32_t rng=(uint32_t)(instance_id+1)*2654435761u;

    uint32_t ready=poll_mmio32(cmd_base+OFF_READY,1,10000000);
    uint32_t K=rd32(cmd_base+OFF_NUM_K);
    uint32_t token0=rd32(cmd_base+OFF_TOKEN_BASE+0);
    uint32_t token1=rd32(cmd_base+OFF_TOKEN_BASE+4);
    uint32_t slots_per_client=(uint32_t)num_slots/K;

    wr64(ring_base+RING_PROD_IDX_OFF,0);
    wr64(ring_base+RING_CONS_IDX_OFF,0);

    /* Track which iteration last wrote each slot (for verification). */
    static uint32_t last_write_iter[MAX_SLOTS]; /* 0xFFFFFFFF = never written */
    memset(last_write_iter,0xFF,sizeof(last_write_iter));

    /* Track which slots were written and the slot index for each write/read op. */
    static uint32_t write_slot[MAX_ITERS];  /* write_slot[i] = slotIdx for write i */
    static uint32_t read_slot[MAX_ITERS];   /* read_slot[i] = slotIdx for read i */

    /* Collect written slots for read-phase random selection. */
    static uint32_t written_set[MAX_SLOTS];
    int written_count = 0;

    /* Pass 1 — WRITE random slots. */
    for(uint32_t i=0;i<(uint32_t)n_iters;i++){
        uint32_t slotIdx=xorshift32(&rng)%(uint32_t)num_slots;
        write_slot[i]=slotIdx;
        uint32_t slot=0x1000u+slotIdx*0x1000u;
        uint8_t hwid=(slotIdx<slots_per_client)?0:1;
        uint32_t token=(hwid==0)?token0:token1;
        uint8_t lease_id=hwid+1;

        uint32_t wdata[8];
        for(int j=0;j<8;j++) wdata[j]=payload_word(i,j);

        /* Track last write iteration for this slot. */
        if(last_write_iter[slotIdx]==0xFFFFFFFFu){
            written_set[written_count++]=slotIdx;
        }
        last_write_iter[slotIdx]=i;

        uint64_t wrIdx=(uint64_t)i;
        while((wrIdx-rd64(ring_base+RING_CONS_IDX_OFF))>=CMD_RING_DEPTH)
            for(volatile int d=0;d<100;d++){}
        write_ring_entry(ring_base,wrIdx,slot,token,lease_id,1,hwid,wdata);
        wr64(ring_base+RING_PROD_IDX_OFF,wrIdx+1);
        wr32(cmd_base+OFF_CMD_RING_DOORBELL,1);
    }

    /* Pass 2 — READ random slots from the written set. */
    for(uint32_t i=0;i<(uint32_t)n_iters;i++){
        uint32_t pick=xorshift32(&rng)%(uint32_t)written_count;
        uint32_t slotIdx=written_set[pick];
        read_slot[i]=slotIdx;
        uint32_t slot=0x1000u+slotIdx*0x1000u;
        uint8_t hwid=(slotIdx<slots_per_client)?0:1;
        uint32_t token=(hwid==0)?token0:token1;
        uint8_t lease_id=hwid+1;

        uint64_t rdIdx=(uint64_t)n_iters+i;
        while((rdIdx-rd64(ring_base+RING_CONS_IDX_OFF))>=CMD_RING_DEPTH)
            for(volatile int d=0;d<100;d++){}
        write_ring_entry(ring_base,rdIdx,slot,token,lease_id,0,hwid,NULL);
        wr64(ring_base+RING_PROD_IDX_OFF,rdIdx+1);
        wr32(cmd_base+OFF_CMD_RING_DOORBELL,1);
    }

    while(rd32(cmd_base+OFF_CPU_OP_COUNT)<TOTAL_OPS)
        for(volatile int d=0;d<100;d++){}

    /* Verify — each read must return the last-written data for that slot.
       Poll each entry's DONE bit before reading — PCIe posted writes may
       not have committed to DDR5 yet, and the DDR5 controller can reorder
       writes across banks. */
    uint32_t pass=0,fail=0;
    for(uint32_t i=0;i<(uint32_t)n_iters;i++){
        uint64_t opIdx=(uint64_t)n_iters+i;
        uint64_t resAddr=result_base+opIdx*64;
        uint64_t status;
        while(!((status=rd64(resAddr+RES_STATUS_OFF))&RES_DONE_BIT))
            for(volatile int d=0;d<100;d++){}
        if(!(status&RES_RDATA_VALID)){
            fprintf(stderr,"RDATA_INVALID inst=%d opIdx=%lu status=0x%lx "
                    "resAddr=0x%lx\n",
                    instance_id,(unsigned long)opIdx,
                    (unsigned long)status,(unsigned long)resAddr);
            fail++;continue;
            fflush(stderr);
        }
        uint32_t r0=rd32(resAddr+RES_RDATA_OFF+0);
        uint32_t r1=rd32(resAddr+RES_RDATA_OFF+4);
        uint32_t slotIdx=read_slot[i];
        uint32_t last_i=last_write_iter[slotIdx];
        uint32_t e0=payload_word(last_i,0);
        uint32_t e1=payload_word(last_i,1);
        if(r0==e0&&r1==e1) pass++; else {
            uint32_t r2=rd32(resAddr+RES_RDATA_OFF+8);
            uint32_t r3=rd32(resAddr+RES_RDATA_OFF+12);
            fprintf(stderr,"MISMATCH inst=%d opIdx=%lu slot=%u last_wr_i=%u "
                    "got=[%08x %08x %08x %08x] exp=[%08x %08x] status=0x%lx\n",
                    instance_id,(unsigned long)opIdx,slotIdx,last_i,
                    r0,r1,r2,r3,e0,e1,(unsigned long)status);
            fail++;
            fflush(stderr);
        }
    }

    wr32(cmd_base+DEBUG_OFF_TOTAL_OPS,TOTAL_OPS);
    wr32(cmd_base+DEBUG_OFF_READY,ready);
    wr32(cmd_base+DEBUG_OFF_K,K);
    wr32(cmd_base+DEBUG_OFF_OP_COUNT,rd32(cmd_base+OFF_CPU_OP_COUNT));
    wr32(cmd_base+DEBUG_OFF_PASS,pass);
    wr32(cmd_base+DEBUG_OFF_FAIL,fail);
    return (fail==0)?0:1;
}