#!/usr/bin/env python3
# =============================================================================
# oram_ssd.py — Phase D CXL config (GitHub name)
# Local name: oram_cmd_port_test_phaseD_ssd.py
#
# Storage backend modes (mutually exclusive):
#   default        : ORAM bucket storage in DDR5
#   --use-ssd      : ORAM bucket storage in SsdMemory (sync HIL behind xbar)
#   --use-nvme     : NvmeSsdDevice on the host fabric (PCIe NVMe model);
#                    the CPU drives it via BAR0 MMIO + DMA from the device
#                    back into host DDR5. ORAM stays in DDR5.
#
# NVMe wiring:
#   CPU -> membus -> iobridge -> iobus -> NvmeSsdDevice.pio  (BAR0)
#   NvmeSsdDevice.dma -> system.pcie -> cxl_host_xbar -> DDR5
#
# Dedicated PCIeModel instance for NVMe (separate from the ORAM fabric).
# In --use-nvme mode, both ORAM and NVMe use PCIeModel (no CXL).
# On a real machine, two PCIe root ports meet at the host fabric (the mesh
# / memory-controller side). Modeling them as separate fabric instances
# joined at cxl_host_xbar is the faithful structure: independent tag
# pools, link-layer credits, and timing parameters; shared contention
# only where it physically exists (DDR5 bandwidth at the host xbar).
#
# Run (DDR5 default, same as before):
#   ./build/ALL/gem5.opt configs/oram_ssd.py \
#       --binary=configs/oram_workload \
#       --num-instances=N
#
# Run (SSD via SsdMemory):
#   ./build/ALL/gem5.opt configs/oram_ssd.py \
#       --binary=configs/oram_workload \
#       --num-instances=N --use-ssd
#
# Run (PCIe NVMe SSD):
#   ./build/ALL/gem5.opt configs/oram_ssd.py \
#       --binary=configs/oram_nvme_test \
#       --num-instances=1 --use-nvme
# =============================================================================

import argparse
import math as _math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oram_addr_layout import (
    MAIN_DRAM_BASE, MAIN_DRAM_SIZE,
    HBM_PER_INSTANCE, DDR_PER_INSTANCE,
    HBM_BASE, DDR_AGG_BASE, DDR_SLAB_BASE,
    CMD_RING_BASE, RESULT_BUF_BASE, ORAM_CMD_BASE,
    STASH_OFFSET,
    per_instance_addrs, ddr_aggregate_size,
)

import m5
from m5.objects import (
    System, SrcClockDomain, VoltageDomain, AddrRange,
    Process, Root,
    Cache, SystemXBar, L2XBar, NoncoherentXBar, Bridge,
    MemCtrl, HBM_2000_4H_1x64,
    OramDevice, CxlModel,
    SimpleMemory,
)
from m5.objects.X86CPU import X86TimingSimpleCPU
from m5.objects.SsdMemory import SsdMemory
from m5.objects.NvmeSsdDevice import NvmeSsdDevice
from m5.objects.PCIeModel import PCIeModel
from gem5.components.memory.dram_interfaces.ddr5 import DDR5_6400_4x8


# =============================================================================
# Cache classes — identical to Phase B.
# =============================================================================
class L1Cache(Cache):
    assoc = 8
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 16
    tgts_per_mshr = 20
    writeback_clean = False

class MyL1DCache(L1Cache): size = '32kB'
class MyL1ICache(L1Cache): size = '32kB'

class MyL2Cache(Cache):
    size = '256kB'
    assoc = 8
    tag_latency = 10
    data_latency = 10
    response_latency = 10
    mshrs = 20
    tgts_per_mshr = 12
    writeback_clean = False


# =============================================================================
# Args
# =============================================================================
parser = argparse.ArgumentParser()
parser.add_argument('--binary',         type=str, required=True)
parser.add_argument('--num-instances',  type=int, default=1)
parser.add_argument('--num-slots',      type=int, default=16)
parser.add_argument('--num-ops',        type=int, default=20,
                    help='Total ops per instance (writes + reads). '
                         'Must be even. Binary gets n_iters = num_ops/2. '
                         'Default 20 (= 10 write + 10 read).')
parser.add_argument('--local-pct',      type=int, default=0)

# Backend selection (mutually exclusive)
backend = parser.add_mutually_exclusive_group()
backend.add_argument('--use-ssd',  action='store_true',
                     help='Use SsdMemory (sync HIL) instead of DDR5 for '
                          'ORAM bucket storage.')
backend.add_argument('--use-nvme', action='store_true',
                     help='Attach a PCIe NvmeSsdDevice on the host fabric. '
                          'The test binary drives it via BAR0 MMIO. ORAM '
                          'stays in DDR5.')

parser.add_argument('--ssd-config',     type=str,
                    default='src/mem/ssd/simplessd/config/sample.cfg',
                    help='Path to SimpleSSD config file (used by both '
                         '--use-ssd and --use-nvme).')
parser.add_argument('--dram-cache',     type=str, default='0B',
                    help='On-device DRAM cache size for SsdMemory '
                         '(e.g. 4MB). 0B = disabled. Ignored for --use-nvme.')
parser.add_argument('--cache-warm-after', type=int, default=0,
                    help='Number of SSD accesses before DRAM cache activates. '
                         'Set to init access count to start cache cold after '
                         'write-init. 0 = cache active from start.')

# NVMe-specific
parser.add_argument('--nvme-bar0', type=lambda x: int(x, 0),
                    default=0x500000000,
                    help='Base address of NVMe BAR0 (controller registers + '
                         'doorbells). Default 0x500000000 (20 GiB) to clearly '
                         'sit above all DDR/HBM/SSD/ORAM ranges. The test '
                         'binary reads this from argv to know where to MMIO.')
parser.add_argument('--nvme-bar0-size', type=lambda x: int(x, 0),
                    default=0x10000,
                    help='Size of NVMe BAR0 (must match NvmeSsdDevice). '
                         'Default 64kB.')
args = parser.parse_args()

N = args.num_instances
assert 1 <= N <= 16

# =============================================================================
# Infer whether ORAM is needed from the binary name.
#
# NVMe-only correctness/smoke tests (nvme_*, aliasing_test, minimal_test,
# ddr5_latency) don't touch ORAM. Instantiating ORAM + CXL + HBM + Verilator
# RTL for these tests adds ~5-10x wallclock cost for nothing — Verilator
# steps the entire ORAM datapath every cycle even when idle.
#
# Convention: any binary whose basename starts with one of NO_ORAM_PREFIXES
# runs without ORAM. Everything else gets the full ORAM stack.
NO_ORAM_PREFIXES = ('nvme_', 'aliasing_test', 'minimal_test', 'ddr5_latency')
binary_basename = os.path.basename(args.binary)
need_oram = not any(binary_basename.startswith(p) for p in NO_ORAM_PREFIXES)


# =============================================================================
# Address ranges
# =============================================================================
main_range  = AddrRange(MAIN_DRAM_BASE, size=MAIN_DRAM_SIZE)

hbm_ranges = [
    AddrRange(HBM_BASE + i * HBM_PER_INSTANCE, size=HBM_PER_INSTANCE)
    for i in range(N)
]

# v4: single DDR5 aggregate covers cmd_ring + result_buf + ORAM DDR slabs.
# This range is interleaved across all 8 DDR5 channels.
ddr_agg_size  = ddr_aggregate_size(N)
ddr_agg_range = AddrRange(DDR_AGG_BASE, size=ddr_agg_size)

cmd_ranges = [
    AddrRange(ORAM_CMD_BASE + i * 0x1000, size=0x1000)
    for i in range(N)
]
cmd_combined_range = AddrRange(ORAM_CMD_BASE, size=N * 0x1000)

# SSD range: placed at 0x400000000 (16 GiB), after DDR5 aggregate.
SSD_BASE = 0x400000000
SSD_TOTAL = N * DDR_PER_INSTANCE
ssd_range = AddrRange(SSD_BASE, size=SSD_TOTAL) if args.use_ssd else None

# NVMe BAR0 range
nvme_bar0_range = (AddrRange(args.nvme_bar0, size=args.nvme_bar0_size)
                   if args.use_nvme else None)

# NVMe shared region (Option Z): a dedicated SimpleMemory range used for
# NVMe queues, PRP-list pool, and stage buffers. Carved out OUTSIDE the
# DDR5 aggregate to avoid the CPU<->DMA address-aliasing problem we hit
# when these structures lived inside the interleaved DDR5 range.
#
# Layout (16 MB total at 0x600000000):
#   +0x000000  ASQ                (1 KB)
#   +0x001000  ACQ                (256 B, padded)
#   +0x002000  IOSQ               (4 KB)
#   +0x003000  IOCQ               (1 KB)
#   +0x004000  Identify buffer    (4 KB)
#   +0x010000  PRP-list pool      (per-instance, 4 KB each)
#   +0x100000  Stage buffer 0     (1 MB)
#   +0x200000  Stage buffer 1     (1 MB)
#   ...        Stage buffer N-1   (one per ORAM instance)
#
# Bug fix A1+A2: was 0x220000000 which overlaps HBM instance 9 at N>=10.
# Moved INSIDE DDR_AGG so the existing 8-channel DDR5 controllers serve
# it — both CPU loads/stores and NVMe DMAs route through cxl_host_xbar
# to the same DDR5 backing, no separate SimpleMemory needed.
# Offset 0x020000000 sits safely after result_buf (ends +0x011000000
# at N=16) and before DDR slabs (start +0x100000000).
NVME_SHARED_BASE = DDR_AGG_BASE + 0x020000000
NVME_SHARED_SIZE = 0x01000000  # 16 MB
nvme_shared_range = (AddrRange(NVME_SHARED_BASE, size=NVME_SHARED_SIZE)
                     if args.use_nvme else None)

# Sanity: NVME_SHARED must sit inside DDR_AGG and not overlap result_buf or DDR slabs.
_result_buf_end = RESULT_BUF_BASE + N * 0x1000000
assert NVME_SHARED_BASE >= _result_buf_end, \
    f"NVME_SHARED 0x{NVME_SHARED_BASE:x} overlaps result_buf ending at 0x{_result_buf_end:x}"
assert NVME_SHARED_BASE + NVME_SHARED_SIZE <= DDR_SLAB_BASE, \
    f"NVME_SHARED end 0x{NVME_SHARED_BASE+NVME_SHARED_SIZE:x} overlaps DDR slabs at 0x{DDR_SLAB_BASE:x}"


# =============================================================================
# Build the system
# =============================================================================
system = System()
system.clk_domain = SrcClockDomain(clock='3GHz', voltage_domain=VoltageDomain())
system.mem_mode = 'timing'
system.mem_ranges = [main_range, ddr_agg_range,
                     cmd_combined_range] + hbm_ranges
# Bug fix A3: ssd_range NOT added to mem_ranges. SsdMemory.port
# advertises its range to cxl_host_xbar for routing. Adding it to
# mem_ranges causes gem5 SE mode to calloc a SECOND backing store for
# the same range — wasting 4 GB host RAM at N=16. The CPU never
# accesses SSD addresses directly; only ORAM does via CXL.
if nvme_bar0_range:
    system.mem_ranges.append(nvme_bar0_range)
# nvme_shared_range is inside DDR_AGG (see NVME_SHARED_BASE definition),
# so ddr_agg_range already covers it. Do NOT add separately — SE-mode
# memPools would let glibc heap overwrite our queues.

# --- CPU + caches ---
system.cpu = [X86TimingSimpleCPU(cpu_id=i) for i in range(N)]

system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

for i in range(N):
    cpu = system.cpu[i]

    cpu.icache = MyL1ICache()
    cpu.dcache = MyL1DCache()

    cpu.icache_port = cpu.icache.cpu_side
    cpu.icache.mem_side = system.membus.cpu_side_ports
    cpu.dcache_port = cpu.dcache.cpu_side
    cpu.dcache.mem_side = system.membus.cpu_side_ports

    cpu.createInterruptController()
    cpu.interrupts[0].pio           = system.membus.mem_side_ports
    cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    cpu.interrupts[0].int_responder = system.membus.mem_side_ports

# --- Main CPU DRAM ---
system.main_mem = MemCtrl()
system.main_mem.dram = DDR5_6400_4x8()
system.main_mem.dram.range = main_range
system.main_mem.port = system.membus.mem_side_ports

# --- N OramDevices, each with its own HBM ---
# Skipped entirely for NVMe-only correctness tests. Verilator stepping
# the ORAM RTL every cycle is by far the largest wallclock cost in
# the system; skipping it for tests that don't exercise ORAM speeds
# the iteration loop dramatically.
if need_oram:
    oram_list, hbm_xbar_list, hbm_ctrl_list = [], [], []

    for i in range(N):
        a = per_instance_addrs(i)
        if args.use_ssd:
            oram_host_base = SSD_BASE + i * DDR_PER_INSTANCE
        else:
            oram_host_base = a['ddr']

        oram_i = OramDevice(
            oram_freq='300MHz',
            local_pct=args.local_pct,
            num_slots=args.num_slots,
            gated_start = args.use_nvme,
            num_ops=args.num_ops,
            hbm_base       = a['hbm'],
            host_base      = oram_host_base,
            stash_offset   = STASH_OFFSET,
            cpu_driven     = True,
            num_logical_clients = 2,
            cmd_base       = a['cmd_port'],
            result_buf_base= a['result_buf'],
            result_buf_size= 0x1000000,
            cmd_ring_base  = a['cmd_ring'],
            cmd_ring_depth = 16,
            cmd_queue_depth= 16,
        )

        hbm_xbar_i = NoncoherentXBar(
            width=128, clk_domain=system.clk_domain,
            frontend_latency=1, forward_latency=1, response_latency=1,
        )
        oram_i.hbm_port = hbm_xbar_i.cpu_side_ports

        hbm_ctrl_i = MemCtrl()
        hbm_ctrl_i.dram = HBM_2000_4H_1x64()
        hbm_ctrl_i.dram.range = hbm_ranges[i]
        hbm_ctrl_i.port = hbm_xbar_i.mem_side_ports

        oram_list.append(oram_i)
        hbm_xbar_list.append(hbm_xbar_i)
        hbm_ctrl_list.append(hbm_ctrl_i)

    system.oram     = oram_list
    system.hbm_xbar = hbm_xbar_list
    system.hbm_ctrl = hbm_ctrl_list

# --- Shared fabric: PCIe for --use-nvme, CXL for everything else ---
if need_oram:
    if args.use_nvme:
        # NVMe mode: ORAM uses PCIeModel (Gen5 x16) to reach DDR5.
        # Both ORAM and NVMe SSD are on PCIe — no CXL in the system.
        system.oram_fabric = PCIeModel(
            gen=5,
            lanes=16,
            mps=512,
            mrrs=512,
            max_tags=256,
            max_outstanding=64,
            max_outstanding_writes=128,
            rrb_depth=256,
            completion_reorder_depth=64,
            pcie_core_clock='1ns',
            pcie_core_width=64,
            pcie_tlp_gap='2ns',
            host_inject_interval='1ns',
            requester_id=0x0100,  # bus=1, dev=0, func=0
        )
        system.oram_fabric.clk_domain = SrcClockDomain(
            clock='1GHz', voltage_domain=VoltageDomain())

        for i in range(N):
            system.oram[i].pcie_port = system.oram_fabric.device_side_port
    else:
        # DDR5 or CXL SSD mode: ORAM uses CxlModel (Gen5 x16).
        system.oram_fabric = CxlModel(
            gen=5,
            lanes=16,
            cxl_core_clock='1ns',
            max_tags=256,
            max_outstanding=64,
            max_outstanding_writes=128,
            flit_credits=128,
            completion_buffer_depth=2048,
            host_inject_interval='1ns',
        )
        system.oram_fabric.clk_domain = SrcClockDomain(
            clock='1GHz', voltage_domain=VoltageDomain())

        for i in range(N):
            system.oram[i].pcie_port = system.oram_fabric.device_side_port

# --- Host xbar + memory backings ---
system.cxl_host_xbar = NoncoherentXBar(
    width=64, frontend_latency=2, forward_latency=1, response_latency=2,
)
system.cxl_host_xbar.clk_domain = SrcClockDomain(
    clock='2GHz', voltage_domain=VoltageDomain())

NUM_HOST_PORTS = 4
if need_oram:
    for _ in range(NUM_HOST_PORTS):
        system.oram_fabric.host_side_port = system.cxl_host_xbar.cpu_side_ports

NUM_DDR5_CHANNELS = 16
ddr5_intlv_bits = int(_math.log(NUM_DDR5_CHANNELS, 2))
ddr5_masks = [1 << (6 + b) for b in range(ddr5_intlv_bits)]

system.cxl_ddr5_ctrls = [MemCtrl(dram=DDR5_6400_4x8())
                          for _ in range(NUM_DDR5_CHANNELS)]
for i, ctrl in enumerate(system.cxl_ddr5_ctrls):
    ctrl.dram.range = AddrRange(
        start=DDR_AGG_BASE, size=ddr_agg_size,
        masks=ddr5_masks, intlvMatch=i,
    )
    ctrl.port = system.cxl_host_xbar.mem_side_ports

# --- SSD Memory (--use-ssd) ---
if args.use_ssd:
    system.ssd_mem = SsdMemory(
        ssd_config=args.ssd_config,
        range=AddrRange(SSD_BASE, size=SSD_TOTAL),
        coalesce_window='500ns',
        dram_cache_size=args.dram_cache,
        dram_cache_warm_after=args.cache_warm_after,
    )
    system.ssd_mem.port = system.cxl_host_xbar.mem_side_ports

# --- NVMe shared region (--use-nvme, Option Z) ---
# NVMe queues (ASQ/ACQ/IOSQ/IOCQ), PRP-list pool, and stage buffers
# live at NVME_SHARED_BASE, which is inside DDR_AGG. The 8-channel DDR5
# controllers already claim and serve this address range. Both CPU
# loads/stores (CPU -> membus -> cpu_fabric_bridge -> cxl_host_xbar) and
# NVMe DMAs (nvme_ssd.dma -> system.pcie -> cxl_host_xbar) hit the same
# DDR5 backing — no separate SimpleMemory needed.

# --- CPU traffic through fabric ---
# CPU accesses to cmd_ring/result_buf/nvme_shared go directly into
# cxl_host_xbar, bypassing the CxlModel (avoids contention with ORAM
# traffic). All these ranges are inside DDR_AGG, so the single
# ddr_agg_range bridge entry covers everything.
fabric_bridge_ranges = [ddr_agg_range]

system.cpu_fabric_bridge = Bridge(
    delay='2ns',
    ranges=fabric_bridge_ranges,
)
system.cpu_fabric_bridge.cpu_side_port = system.membus.mem_side_ports
system.cpu_fabric_bridge.mem_side_port = system.cxl_host_xbar.cpu_side_ports

# --- CPU MMIO path: iobridge -> iobus -> {ORAM cmd_ports, NVMe BAR0} ---
# Both the ORAM cmd_port MMIO and the optional NVMe BAR0 are reached via
# the same iobridge/iobus path. iobridge ranges grow conditionally to
# cover whichever MMIO targets are present.
system.iobus = NoncoherentXBar(
    width=8, clk_domain=system.clk_domain,
    frontend_latency=1, forward_latency=1, response_latency=1,
)

iobridge_ranges = []
if need_oram:
    iobridge_ranges.append(cmd_combined_range)
if nvme_bar0_range:
    iobridge_ranges.append(nvme_bar0_range)

system.iobridge = Bridge(delay='50ns', ranges=iobridge_ranges)
system.iobridge.cpu_side_port = system.membus.mem_side_ports
system.iobridge.mem_side_port = system.iobus.cpu_side_ports

if need_oram:
    for i in range(N):
        system.oram[i].cmd_port = system.iobus.mem_side_ports

# --- NVMe SSD Device (--use-nvme) ---
# pio: CPU -> membus -> iobridge -> iobus -> nvme_ssd.pio   (BAR0)
# dma: nvme_ssd.dma -> system.pcie -> cxl_host_xbar -> DDR5
#
# Dedicated PCIeModel for NVMe (Gen5 x4), separate from system.oram_fabric
# (Gen5 x16) which carries ORAM traffic. Both are PCIe in --use-nvme mode.
# Two distinct fabric models joined at the host xbar mirrors a real CPU's
# separate PCIe root ports that meet only downstream at the
# memory-controller fabric.
#
# Gen5 x4 is typical for a high-end NVMe SSD (Samsung 990 Pro et al.);
# tag and credit pools are sized smaller than ORAM's Gen5 x16 since a
# single NVMe device generates much less concurrency than a CXL Type-3
# memory expander. MPS=512 / MRRS=512 are consumer-NVMe defaults.
if args.use_nvme:
    system.nvme_ssd = NvmeSsdDevice(
        ssd_config=args.ssd_config,
        pio_addr=args.nvme_bar0,
        pio_size=args.nvme_bar0_size,
        pio_latency='1ns',
    )
    system.nvme_ssd.pio = system.iobus.mem_side_ports

    # Dedicated PCIe fabric for NVMe (independent from system.oram_fabric).
    system.pcie = PCIeModel(
        gen=5,
        lanes=4,
        mps=512,
        mrrs=512,
        max_tags=128,
        max_outstanding=64,
        max_outstanding_writes=64,
        rrb_depth=128,
        completion_reorder_depth=32,
        pcie_core_clock='4ns',
        pcie_core_width=64,
        pcie_tlp_gap='4ns',
        host_inject_interval='1ns',
        requester_id=0x0200,  # bus=2, dev=0, func=0
    )
    system.pcie.clk_domain = SrcClockDomain(
        clock='1GHz', voltage_domain=VoltageDomain())

    system.nvme_ssd.dma = system.pcie.device_side_port
    system.pcie.host_side_port = system.cxl_host_xbar.cpu_side_ports


# =============================================================================
# Workload
# =============================================================================
processes = [Process(pid=100 + i) for i in range(N)]
for i, p in enumerate(processes):
    # Pass NVMe BAR0 + max_lba as extra argv when --use-nvme.
    # max_lba enforces Option 1 fair-comparison: PCIe NVMe SSD only uses
    # the same LBA span that CXL SSD covers, sized at DDR_PER_INSTANCE
    # bytes (the ORAM bucket-storage size). LBA size is 512 B (matches
    # SimpleSSD sample.cfg).
    if args.use_nvme:
        max_lba = DDR_PER_INSTANCE // 512  # = 131072 for 64 MB / 512
        p.cmd = [args.binary, str(N), str(i), str(args.num_ops // 2), str(args.num_slots),
                 hex(args.nvme_bar0), str(max_lba)]
    else:
        p.cmd = [args.binary, str(N), str(i), str(args.num_ops // 2), str(args.num_slots), str(args.num_slots)]

system.workload = m5.objects.SEWorkload.init_compatible(args.binary)
for i in range(N):
    system.cpu[i].workload = processes[i]
    system.cpu[i].createThreads()


# =============================================================================
# Run
# =============================================================================
root = Root(full_system=False, system=system)
m5.instantiate()

# Map MMIO and fabric memory uncached for each instance, in EACH Process.
for proc in processes:
    for i in range(N):
        a = per_instance_addrs(i)
        proc.map(a['cmd_port'],   a['cmd_port'],   0x1000,   False)
        proc.map(a['result_buf'], a['result_buf'], 0x1000000, False)
        proc.map(a['cmd_ring'],   a['cmd_ring'],   0x1000,   False)
    # Map NVMe BAR0 uncached so the binary can MMIO doorbells/regs.
    if args.use_nvme:
        proc.map(args.nvme_bar0, args.nvme_bar0,
                 args.nvme_bar0_size, False)
        proc.map(NVME_SHARED_BASE, NVME_SHARED_BASE,
                 NVME_SHARED_SIZE, False)
        for i in range(N):
            slab_addr = DDR_SLAB_BASE + i * DDR_PER_INSTANCE
            proc.map(slab_addr, slab_addr, DDR_PER_INSTANCE, False)


print('=' * 60)
print(f'oram_cmd_port_test_phaseD (CXL): N={N}')
print(f'  fabric:  CxlModel(gen=5, lanes=16, core_clock=1ns) shared')
print(f'  HBM:     {N} controllers, [0x{HBM_BASE:09x}, 0x{HBM_BASE + N*HBM_PER_INSTANCE:09x})')
print(f'  DDR5:    {NUM_DDR5_CHANNELS} channels, [0x{DDR_AGG_BASE:09x}, 0x{DDR_AGG_BASE + ddr_agg_size:09x}) (incl. cmd_ring + result_buf)')
if args.use_ssd:
    print(f'  SSD:     [0x{SSD_BASE:09x}, 0x{SSD_BASE + SSD_TOTAL:09x}) (ORAM bucket storage)')
    print(f'           config: {args.ssd_config}')
elif args.use_nvme:
    print(f'  NVMe:    BAR0 at [0x{args.nvme_bar0:09x}, '
          f'0x{args.nvme_bar0 + args.nvme_bar0_size:09x})')
    print(f'           DMA -> system.pcie (Gen5 x4) -> cxl_host_xbar -> DDR5')
    print(f'           shared (queues + stage bufs): '
          f'[0x{NVME_SHARED_BASE:09x}, 0x{NVME_SHARED_BASE + NVME_SHARED_SIZE:09x}) '
          f'(inside DDR5 aggregate)')
    print(f'           config: {args.ssd_config}')
else:
    print(f'  SSD:     disabled (ORAM buckets in DDR5)')
print(f'  ORAM cmd MMIO: [0x{ORAM_CMD_BASE:09x}, 0x{ORAM_CMD_BASE + N*0x1000:09x})')
print(f'  binary: {args.binary}')
print('=' * 60)

exit_event = m5.simulate()
print(f'Exited @ tick {m5.curTick()} because {exit_event.getCause()}')