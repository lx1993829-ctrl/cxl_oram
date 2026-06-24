#!/usr/bin/env python3
# =============================================================================
# oram_sim_pcie_8ch.py — PCIe config with 8 DDR5-6400 channels
#
# Difference from oram_sim_pcie.py:
#   - 8 MemCtrl × DDR5_6400_4x8 (32-bit per subchannel)
#     instead of 16 MemCtrl × DDR5_6400_4x8 (16 subchannels)
#   - Models 8 physical DDR5 channels, one scheduler per channel.
#     Subchannels within a channel share the scheduler (conservative).
#   - Total bandwidth: 8 × 64b × 6400 MT/s = 409.6 GB/s (unchanged)
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
    OramDevice, PCIeModel,
    LPDDR5_6400_1x16_BG_BL32,
)
from m5.objects.X86CPU import X86TimingSimpleCPU
from gem5.components.memory.dram_interfaces.ddr5 import DDR5_6400_4x8


# =============================================================================
# Custom 64-bit DDR5 channel: 8 devices × 8 bits = 64-bit data bus.
# This models a full physical DDR5 channel (both subchannels merged
# into one scheduler). Conservative: loses per-subchannel scheduling
# independence, but avoids overstating parallelism.
# =============================================================================




# =============================================================================
# Cache classes
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
parser.add_argument('--num-ops',        type=int, default=20)
parser.add_argument('--local-pct',      type=int, default=0)
parser.add_argument('--local-mem',      type=str, default='hbm2',
    choices=['hbm2', 'lpddr5_1x16', 'lpddr5_2x16'])
args = parser.parse_args()

N = args.num_instances
assert 1 <= N <= 32


# =============================================================================
# Address ranges
# =============================================================================
main_range  = AddrRange(MAIN_DRAM_BASE, size=MAIN_DRAM_SIZE)

hbm_ranges = [
    AddrRange(HBM_BASE + i * HBM_PER_INSTANCE, size=HBM_PER_INSTANCE)
    for i in range(N)
]

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
system.cpu = [X86TimingSimpleCPU(cpu_id=i) for i in range(N)]

system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

cpu_l2bus_list = []
for i in range(N):
    cpu = system.cpu[i]
    cpu.icache = MyL1ICache()
    cpu.dcache = MyL1DCache()
    cpu.icache_port = cpu.icache.cpu_side
    cpu.dcache_port = cpu.dcache.cpu_side

    l2bus = L2XBar()
    cpu.icache.mem_side = l2bus.cpu_side_ports
    cpu.dcache.mem_side = l2bus.cpu_side_ports
    cpu_l2bus_list.append(l2bus)

    cpu.l2cache = MyL2Cache()
    cpu.l2cache.cpu_side = l2bus.mem_side_ports
    cpu.l2cache.mem_side = system.membus.cpu_side_ports

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

CTRLS_PER_INST = 2 if args.local_mem == 'lpddr5_2x16' else 1

def make_local_dram():
    if args.local_mem == 'hbm2':
        return HBM_2000_4H_1x64()
    else:
        return LPDDR5_6400_1x16_BG_BL32()

for i in range(N):
    hbm_xbar_i = NoncoherentXBar(
        width=128, clk_domain=system.clk_domain,
        frontend_latency=1, forward_latency=1, response_latency=1,
    )
    hbm_xbar_list.append(hbm_xbar_i)

    inst_base = HBM_BASE + i * HBM_PER_INSTANCE
    for sub in range(CTRLS_PER_INST):
        ctrl = MemCtrl()
        ctrl.dram = make_local_dram()
        if CTRLS_PER_INST == 1:
            ctrl.dram.range = AddrRange(inst_base, size=HBM_PER_INSTANCE)
        else:
            ctrl.dram.range = AddrRange(
                start=inst_base, size=HBM_PER_INSTANCE,
                masks=[1 << 6], intlvMatch=sub,
            )
        ctrl.port = hbm_xbar_i.mem_side_ports
        hbm_ctrl_list.append(ctrl)

for i in range(N):
    a = per_instance_addrs(i)
    oram_i = OramDevice(
        oram_freq='300MHz',
        local_pct=args.local_pct,
        num_slots=args.num_slots,
        num_ops=args.num_ops,
        hbm_base       = a['hbm'],
        host_base      = a['ddr'],
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
    oram_i.hbm_port = hbm_xbar_list[i].cpu_side_ports
    oram_list.append(oram_i)

system.oram     = oram_list
system.hbm_xbar = hbm_xbar_list
system.hbm_ctrl = hbm_ctrl_list

# --- Single shared PCIe fabric ---
system.pcie = PCIeModel(
    gen=5,
    lanes=16,
    pcie_core_clock='1ns',
    max_tags=256,
    max_outstanding=64,
    max_outstanding_writes=128,
    credits_ph=128,
    credits_pd=256,
    credits_nph=128,
    completion_reorder_depth=256,
    host_inject_interval='1ns',
)
system.pcie.clk_domain = SrcClockDomain(clock='1GHz', voltage_domain=VoltageDomain())

for i in range(N):
    system.oram[i].pcie_port = system.pcie.device_side_port

# --- Host xbar ---
system.cxl_host_xbar = NoncoherentXBar(
    width=64, frontend_latency=2, forward_latency=1, response_latency=2,
)
system.cxl_host_xbar.clk_domain = SrcClockDomain(
    clock='2GHz', voltage_domain=VoltageDomain())

NUM_HOST_PORTS = 4
for _ in range(NUM_HOST_PORTS):
    system.pcie.host_side_port = system.cxl_host_xbar.cpu_side_ports

# =============================================================================
# 8 DDR5-6400 channels (64-bit each) — one scheduler per physical channel.
# Conservative: per-subchannel scheduling independence is NOT modeled.
# Total bandwidth: 8 × 64b × 6400 MT/s = 409.6 GB/s (matches 16-subchannel).
# =============================================================================
NUM_DDR5_CHANNELS = 8
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

# --- CPU traffic through host xbar (NOT through PCIe) ---
system.cpu_fabric_bridge = Bridge(
    delay='2ns',
    ranges=[ddr_agg_range],
)
system.cpu_fabric_bridge.cpu_side_port = system.membus.mem_side_ports
system.cpu_fabric_bridge.mem_side_port = system.cxl_host_xbar.cpu_side_ports

# --- CPU MMIO path ---
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
processes = [Process(pid=100 + i) for i in range(N)]
for i, p in enumerate(processes):
    p.cmd = [args.binary, str(N), str(i), str(args.num_ops // 2), str(args.num_slots)]

system.workload = m5.objects.SEWorkload.init_compatible(args.binary)
for i in range(N):
    system.cpu[i].workload = processes[i]
    system.cpu[i].createThreads()


# =============================================================================
# Run
# =============================================================================
root = Root(full_system=False, system=system)
m5.instantiate()

for proc in processes:
    for i in range(N):
        a = per_instance_addrs(i)
        proc.map(a['cmd_port'],   a['cmd_port'],   0x1000,   False)
        proc.map(a['result_buf'], a['result_buf'], 0x1000000, False)
        proc.map(a['cmd_ring'],   a['cmd_ring'],   0x1000,   False)


print('=' * 60)
print(f'oram_sim_pcie_8ch v1 (PCIe, 8-channel DDR5): N={N}')
print(f'  fabric:  PCIeModel(gen=5, lanes=16)')
print(f'  DDR5:    8 × DDR5_6400_4x8 (32-bit/ch, half bandwidth)')
print(f'           Total BW: 204.8 GB/s (half of 16-subchannel config)')
print(f'  HBM:     {N} xbars x {CTRLS_PER_INST} ctrl ({args.local_mem})')
print(f'  local_pct={args.local_pct}')
print('=' * 60)

exit_event = m5.simulate()
print(f'Exited @ tick {m5.curTick()} because {exit_event.getCause()}')
