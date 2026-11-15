#!/bin/bash
# ycsb_ov.sh -- the six YCSB workloads on the overlapped controller (K = 4, run-time-K build ~/p3rx300krt), with the same
# settings as the serial Fig. 9 runs (~/bl_trace ycsb2.sh): 4 KB blocks, 1,048,572 slots, 524,286 loaded, 20% of buckets
# in HBM, A to F with seed 1 and F to A with seed 2, about 1M commands each, the card cooled before every workload.
# Run from ~/p3rx300krt/fpga_zcu208/vitis:   nohup bash ycsb_ov.sh > ycsb_ov.out 2>&1 &
# Ends with a table of time per request, overlap against the serial ycsb2 logs, in ycsb_ov_summary.txt.
source ~/cool.sh; cd "$(dirname "$0")"; export DOMA_HBM_PCT=20 DOMA_LAMBDA=7 DOMA_BLOCK=4096 DOMA_OV_K=4
L=$HOME/.u55c_card.lock; SER=${SER:-$HOME/bl_trace/fpga_zcu208/vitis}; XC=${XC:-doma_hw.xclbin}
[ -f "$XC" ] || { echo "$XC not found in $PWD"; exit 2; }
grep -q DOMA_OV_K host_ycsb.cpp || { echo "host_ycsb.cpp here has no DOMA_OV_K: wrong tree"; exit 2; }
source /opt/xilinx/xrt/setup.sh > /dev/null
if [ ! -x ./host_ycsb ] || [ host_ycsb.cpp -nt ./host_ycsb ]; then
    echo "== compiling host_ycsb"; g++ -std=c++17 -O2 -ffp-contract=off host_ycsb.cpp -I$XILINX_XRT/include -L$XILINX_XRT/lib -lxrt_coreutil -pthread -o host_ycsb || exit 2
fi
crash=0
for p in "ycsb_ov_af 1 A B C D E F" "ycsb_ov_fa 2 F E D C B A"; do set -- $p; d=$1; i=$2; shift 2; mkdir -p $d
  for w in "$@"; do it=1000000; [ $w = E ] && it=189517; [ $w = F ] && it=666331
    for t in 1 2 3 4 5 6 7 8 9 10; do grep -aq "^WORKLOAD PASSED" $d/full_$w.log 2>/dev/null && break; cool
      echo "== $d $w try $t iters $it $(date +%T)"
      for f in $d/hist.csv $d/slow.txt; do [ -f $f ] && awk -F, -v w=$w '$1!=w' $f > $f.tmp && mv $f.tmp $f; done
      flock -w 300 $L env DOMA_LAT_HIST=$d/hist.csv DOMA_SLOW_DUMP=$d/slow.txt timeout 900 ./host_ycsb $XC 1 $i $it 1048572 $w 524286 10 > $d/full_$w.log 2>&1
      rc=$?; echo "$d $w exit $rc"; [ $rc != 0 ] && crash=$((crash+1)); done; done; done
n=$(grep -l "^WORKLOAD PASSED" ycsb_ov_af/full_?.log ycsb_ov_fa/full_?.log 2>/dev/null | wc -l); echo "PASSED $n OF 12; $crash crashes"
# time per request = device cycles per command of the run phase ("RUN : ... = X cycles per command"), at 300 MHz
cpc() { sed -n 's/^RUN : .* = \([0-9.]*\) cycles per command.*/\1/p' "$1" 2>/dev/null | head -1; }
{ echo "time per request, us (device cycles per command / 300); overlap K = 4 here, serial from $SER/ycsb2_*"
  printf "%-3s %9s %9s %9s %9s %8s\n" wl ov_seed1 ov_seed2 ser_seed1 ser_seed2 speedup
  for w in A B C D E F; do
    o1=$(cpc ycsb_ov_af/full_$w.log); o2=$(cpc ycsb_ov_fa/full_$w.log); s1=$(cpc $SER/ycsb2_af/full_$w.log); s2=$(cpc $SER/ycsb2_fa/full_$w.log)
    awk -v w=$w -v o1="$o1" -v o2="$o2" -v s1="$s1" -v s2="$s2" 'function us(x){return x==""?"-":sprintf("%.2f",x/300)}
      BEGIN{sp=(o1!=""&&o2!=""&&s1!=""&&s2!="")?sprintf("%.2f",(s1+s2)/(o1+o2)):"-"; printf "%-3s %9s %9s %9s %9s %8s\n",w,us(o1),us(o2),us(s1),us(s2),sp}'
  done; } | tee ycsb_ov_summary.txt
echo YCSB_OV_DONE
