This is for continue debugging nvme SSD and run experiments for host mem and local mem. 


Step 1 Install and build Gem5, Verilator


Step 2 Download SimpleSSD:https://github.com/SimpleSSD/SimpleSSD. Clone into gem5/src/mem/ssd/ and then build.


Step 3:Put the files here the same hierarchy as they are. For example, put oram_device.cc under your gem5/src/oram/


Step 4 cd into rtl_gem5/ and run: verilator --cc --top-module secure_oram_top -I. *.v -Wno-fatal --converge-limit 200 --threads 1 -j 1 --Mdir obj_dir


Step 5 "cd obj_dir/" &&  "make -j1 -f Vsecure_oram_top.mk"


Step 6 "cd gem5" && "scons build/ALL/gem5.opt -j8"


Step 7 "cd gem5/tests", "gcc -O2 -static -Wall -Wextra -o nvme_io_test nvme_io_test.c", "gcc -O2 -static -Wall -Wextra -o oram_nvme_test oram_nvme_test.c
"


Step 8 "cd gem5/ and compile .c "musl-gcc -O0 -static -o configs/oram_workload configs/oram_workload.c"

Step 9: bash gem5/configs/sweep_scaling.sh and gem5/configs/plot_local_pct.py



# Narrow check
./build/ALL/gem5.opt configs/oram_cmd_port_test_phaseD_ssd.py \
    --binary=tests/nvme_io_test \
    --use-nvme \
    > m5out/nvme_io_stdout.log 2>&1
grep -F "|||" m5out/nvme_io_stdout.log | grep -aE "PASS|FAIL|verify|mismatches"



cd /home/ylu18/luoresearch/cxl_oram/gem5
export LD_LIBRARY_PATH=/home/ylu18/miniconda3/lib:$LD_LIBRARY_PATH

# Use the config default NVMe BAR0. Do not set --nvme-bar0=0x600000000;
# 0x600000000 is the DDR aggregate/cmd-ring region in the current layout.

./build/ALL/gem5.opt configs/oram_cmd_port_test_phaseD_ssd.py \
    --binary=tests/nvme_io_test \
    --use-nvme \
    > m5out/nvme_io_stdout.log 2>&1

grep -F "|||" m5out/nvme_io_stdout.log | grep -aE "PASS|FAIL|verify|mismatches"




# Full pipeline
./build/ALL/gem5.opt configs/oram_cmd_port_test_phaseD_ssd.py \
    --binary=tests/oram_nvme_test \
    --use-nvme \
    --n-iters=1 \
    > m5out/oram_nvme_stdout.log 2>&1
grep -F "|||" m5out/oram_nvme_stdout.log | grep -aE "PASS|FAIL|iter_pass|iter_fail|verify"

cd /home/ylu18/luoresearch/cxl_oram/gem5        
export LD_LIBRARY_PATH=/home/ylu18/miniconda3/lib:$LD_LIBRARY_PATH

./build/ALL/gem5.opt configs/oram_cmd_port_test_phaseD_ssd.py \
    --binary=tests/oram_nvme_test \
    --use-nvme \
    --n-iters=1 \
    > m5out/oram_nvme_stdout.log 2>&1

grep -F "|||" m5out/oram_nvme_stdout.log | grep -aE "PASS|FAIL|iter_pass|iter_fail|verify"

# Full pipeline regression: crosses the default client0/client1 lease boundary.
cd /home/ylu18/luoresearch/cxl_oram/gem5
./build/ALL/gem5.opt configs/oram_cmd_port_test_phaseD_ssd.py \
    --binary=tests/oram_nvme_test \
    --use-nvme \
    --n-iters=10 \
    > m5out/oram_nvme_n10_stdout.log 2>&1

grep -F "|||" m5out/oram_nvme_n10_stdout.log | grep -aE "PASS|FAIL|iter_pass|iter_fail|verify"
