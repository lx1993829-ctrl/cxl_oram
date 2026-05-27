import argparse
import m5
from m5.objects import *

parser = argparse.ArgumentParser()
parser.add_argument("--binary", required=True)
parser.add_argument("--ramulator-config", required=True)
parser.add_argument("--ramulator-output-dir", default="ramulator2_out")
args = parser.parse_args()

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"
system.mem_ranges = [AddrRange("512MB")]

system.cpu = TimingSimpleCPU()
system.membus = SystemXBar()

# No cache for the first smoke test:
# CPU instruction/data ports directly connect to memory bus.
system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports

system.cpu.createInterruptController()
if hasattr(system.cpu, "interrupts") and len(system.cpu.interrupts) > 0:
    system.cpu.interrupts[0].pio = system.membus.mem_side_ports
    system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

# Ramulator2 memory backend
system.mem_ctrl = Ramulator2()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.config_path = args.ramulator_config
system.mem_ctrl.output_dir = args.ramulator_output_dir
system.mem_ctrl.port = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

process = Process()
process.cmd = [args.binary]

system.workload = SEWorkload.init_compatible(args.binary)
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system=False, system=system)

m5.instantiate()

print("Beginning simulation with gem5 + Ramulator2 HBM3...")
exit_event = m5.simulate()
print("Exiting @ tick {} because {}".format(m5.curTick(), exit_event.getCause()))
