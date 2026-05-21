#!/usr/bin/env python3
# =============================================================================
# oram_cmd_port_test_phaseD.py — Phase D multi-instance CXL config (v3)
#
# v3 changes:
#  - CPU-touched regions in 32-bit: cmd_ring at 0x80000000, result_buf at
#    0x90000000, ORAM_CMD MMIO at 0xE0000000. Same as Phase B.
#  - ORAM-only memory (HBM, DDR aggregate) at 64-bit, ORAM-side fabric
#    routes to it via the CXL model's host xbar.
#  - numThreads only set when N>1 (avoids SE-mode threading issues at N=1).
#  - small_mem at 0x80000000-0xA0000000 (512 MB) backs cmd_ring + result_buf
#    via the fabric, same role as Phase B's cxl_ddr5_ctrls range had.
#
# Topology at N instances:
#   CPU → membus → cpu_fabric_bridge(small_range) → cxl.device_side_port[N]
#   ORAM[0..N-1].pcie_port → cxl.device_side_port[0..N-1]
#   cxl.host_side_port (4-way) → cxl_host_xbar → small_mem + 8×DDR5
#
# Run:
#   ./build/ALL/gem5.opt configs/oram_cmd_port_test_phaseD.py \
#       --binary=/path/to/cmd_port_test_step9_phaseD \
#       --num-instances=N
# =============================================================================

import argparse
import math as _math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from phase_d_layout import (
    MAIN_DRAM_BASE, MAIN_DRAM_SIZE,
    HBM_PER_INSTANCE, DDR_PER_INSTANCE,
    HBM_BASE, DDR_AGG_BASE, DDR_SLAB_BASE,
    CMD_RING_BASE, RESULT_BUF_BASE, ORAM_CMD_BASE,
    per_instance_addrs, ddr_aggregate_size,
)

import m5
from m5.objects import (
    System, SrcClockDomain, VoltageDomain, AddrRange,
    Process, Root,
    Cache, SystemXBar, L2XBar, NoncoherentXBar, Bridge,
    MemCtrl, HBM_2000_4H_1x64,
    OramDevice, CxlModel,
)
from m5.objects.X86CPU import X86TimingSimpleCPU
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
parser.add_argument('--num-ops',        type=int, default=1)
parser.add_argument('--local-pct',      type=int, default=0)
args = parser.parse_args()

N = args.num_instances
assert 1 <= N <= 16


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


# =============================================================================
# Build the system
# =============================================================================
system = System()
system.clk_domain = SrcClockDomain(clock='3GHz', voltage_domain=VoltageDomain())
system.mem_mode = 'timing'
system.mem_ranges = [main_range, ddr_agg_range,
                     cmd_combined_range] + hbm_ranges

# --- CPU + caches ---
# At N=1, default numThreads=1 (workload runs inline, no pthreads).
# At N>1, numThreads=N for the pthread workload.
# --- N independent timing CPUs, each with private L1 I/D + private L2 ---
# All CPUs share the same coherent SystemXBar (membus). Coherence is
# handled by the standard MOESI protocol; in this workload no cacheable
# line is shared between CPUs (each Process has its own page table for
# binary/stack/heap; cmd_ring/result_buf/cmd_port MMIO are uncached),
# so coherence overhead is structurally zero.
system.cpu = [X86TimingSimpleCPU(cpu_id=i) for i in range(N)]

system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

cpu_l2bus_list = []
for i in range(N):
    cpu = system.cpu[i]

    # Per-CPU L1 I/D
    cpu.icache = MyL1ICache()
    cpu.dcache = MyL1DCache()
    cpu.icache_port = cpu.icache.cpu_side
    cpu.dcache_port = cpu.dcache.cpu_side

    # Per-CPU L2 bus (collects L1s before per-CPU L2)
    l2bus = L2XBar()
    cpu.icache.mem_side = l2bus.cpu_side_ports
    cpu.dcache.mem_side = l2bus.cpu_side_ports
    cpu_l2bus_list.append(l2bus)

    # Per-CPU L2
    cpu.l2cache = MyL2Cache()
    cpu.l2cache.cpu_side = l2bus.mem_side_ports
    cpu.l2cache.mem_side = system.membus.cpu_side_ports

    # Per-CPU interrupt controller
    cpu.createInterruptController()
    cpu.interrupts[0].pio           = system.membus.mem_side_ports
    cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.cpu_l2bus = cpu_l2bus_list

# --- Main CPU DRAM ---
system.main_mem = MemCtrl()
system.main_mem.dram = DDR5_6400_4x8()
system.main_mem.dram.range = main_range
system.main_mem.port = system.membus.mem_side_ports

# --- N OramDevices, each with its own HBM ---
oram_list, hbm_xbar_list, hbm_ctrl_list = [], [], []

for i in range(N):
    a = per_instance_addrs(i)
    oram_i = OramDevice(
        oram_freq='300MHz',
        local_pct=args.local_pct,
        num_slots=args.num_slots,
        num_ops=args.num_ops,
        hbm_base       = a['hbm'],
        host_base      = a['ddr'],
        stash_offset   = HBM_PER_INSTANCE - 0x01000000,
        cpu_driven     = True,
        num_logical_clients = 2,
        cmd_base       = a['cmd_port'],
        result_buf_base= a['result_buf'],
        result_buf_size= 0x100000,
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

# --- Single shared CXL fabric, N+1 device-side ports ---
system.cxl = CxlModel(
    gen=5,
    lanes=16,
    cxl_core_clock='1ns',
    max_tags=1024,
    max_outstanding=512,
    max_outstanding_writes=512,
    flit_credits=128,
    completion_buffer_depth=128,
    host_inject_interval='1ns',
)
system.cxl.clk_domain = SrcClockDomain(clock='1GHz', voltage_domain=VoltageDomain())

for i in range(N):
    system.oram[i].pcie_port = system.cxl.device_side_port

# --- Host xbar + memory backings ---
system.cxl_host_xbar = NoncoherentXBar(
    width=64, frontend_latency=2, forward_latency=1, response_latency=2,
)
system.cxl_host_xbar.clk_domain = SrcClockDomain(
    clock='2GHz', voltage_domain=VoltageDomain())

NUM_HOST_PORTS = 4
for _ in range(NUM_HOST_PORTS):
    system.cxl.host_side_port = system.cxl_host_xbar.cpu_side_ports

# 16 DDR5 subchannels = real 8-channel DDR5 host (each DDR5 channel has
# 2 independent x32 subchannels, modeled as separate MemCtrls).
# DDR5_6400_4x8() = one subchannel; 16 of them = 8 channels worth.
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

# v4: small_mem deleted. cmd_ring + result_buf are inside ddr_agg_range
# above and are interleaved across the 8 DDR5 channels.

# --- CPU traffic through fabric ---
# Bridge filters everything CPU-touched outside main_mem: the DDR5
# aggregate (which now contains cmd_ring + result_buf + ORAM slabs).
system.cpu_fabric_bridge = Bridge(
    delay='2ns',
    ranges=[ddr_agg_range],
)
system.cpu_fabric_bridge.cpu_side_port = system.membus.mem_side_ports
system.cpu_fabric_bridge.mem_side_port = system.cxl.device_side_port

# --- CPU MMIO path: iobridge → iobus → N cmd_ports ---
system.iobus = NoncoherentXBar(
    width=8, clk_domain=system.clk_domain,
    frontend_latency=1, forward_latency=1, response_latency=1,
)
system.iobridge = Bridge(delay='50ns', ranges=[cmd_combined_range])
system.iobridge.cpu_side_port = system.membus.mem_side_ports
system.iobridge.mem_side_port = system.iobus.cpu_side_ports

for i in range(N):
    system.oram[i].cmd_port = system.iobus.mem_side_ports


# =============================================================================
# Workload
# =============================================================================
# =============================================================================
# Workload — one Process per CPU. Each CPU runs single-threaded, drives
# its own ORAM instance via instance_id from argv. Each Process gets a
# unique pid; default pid=100 would collide at N>1.
# =============================================================================
processes = [Process(pid=100 + i) for i in range(N)]
for i, p in enumerate(processes):
    p.cmd = [args.binary, str(N), str(i), str(args.num_ops), str(args.num_slots)]

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
        proc.map(a['result_buf'], a['result_buf'], 0x100000, False)
        proc.map(a['cmd_ring'],   a['cmd_ring'],   0x1000,   False)


print('=' * 60)
print(f'oram_cmd_port_test_phaseD (CXL): N={N}')
print(f'  fabric:  CxlModel(gen=5, lanes=16, core_clock=1ns) shared')
print(f'  HBM:     {N} controllers, [0x{HBM_BASE:09x}, 0x{HBM_BASE + N*HBM_PER_INSTANCE:09x})')
print(f'  DDR5:    8 channels, [0x{DDR_AGG_BASE:09x}, 0x{DDR_AGG_BASE + ddr_agg_size:09x}) (incl. cmd_ring + result_buf)')
print(f'  ORAM cmd MMIO: [0x{ORAM_CMD_BASE:09x}, 0x{ORAM_CMD_BASE + N*0x1000:09x})')
print(f'  binary: {args.binary}')
print('=' * 60)

exit_event = m5.simulate()
print(f'Exited @ tick {m5.curTick()} because {exit_event.getCause()}')
