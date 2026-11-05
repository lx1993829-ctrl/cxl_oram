#!/usr/bin/env python3
"""step1_p.py RECORDS.csv [RECORDS.csv ...] -- overlap step 1 (OVERLAP=2): the start spacing P and the same-bucket
shift D_EQ from a v14d1 debug bitstream's per-access records (the card_budgets.py format: build.sh DEBUG=1 TRACE=1,
host_phases with DOMA_PHASE_TRACE=1 DOMA_ONPKG_CSV=<file>). Timestamps are access cycles from the access start.

Step 1 starts access j+1 at s(j) + P and halts unless access j's decisions are done by then (check C1). An access's
decisions end at  D = sx + dec,  sx = stash ready (the decisions start), dec = the decisions' duration, whose card
worst is measured on the on-package accesses as max(ea - sx) (their write-back is issued when the decisions end).
  P    = max(ceil(1.1 * D_max) + 74, 2,900)   (the calibration rule of card_budgets.py; never below the simulated
                                         2,900: the single GCM core decrypts and encrypts every access, ~2,400)
  D_EQ = max(0, PEQ - P)                 P + D_EQ >= PEQ (default 5,600: an off-package write ends about 4,566 cycles
                                         after its access starts on the card, plus the host's write-response latency)
Reported overall and without the first command of each launch (a cold stash open after a launch)."""
import csv, math, sys
NONE = 65535
args = sys.argv[1:]; PEQ = 5600
if args and args[0] == '--peq': PEQ = int(args[1]); args = args[2:]
rows = []
for f in args:
    for r in csv.DictReader(open(f)):
        try: rows.append({k: int(v, 0) if isinstance(v, str) and v.strip() else NONE for k, v in r.items() if k})
        except ValueError: continue
if not rows: sys.exit("step1_p.py: no records")
on = lambda r: r['flags'] & 1
dec = [r['ea'] - r['sx'] for r in rows if on(r) and NONE not in (r['ea'], r['sx'])]
if not dec: sys.exit("step1_p.py: no on-package records with ea and sx (the decisions' duration)")
dmax = max(dec)
lo = {}
for r in rows: lo[r['launch']] = min(lo.get(r['launch'], r['cmd']), r['cmd'])
first = {(l, c) for l, c in lo.items()}   # the first command of each launch
def report(name, sel):
    off = [r for r in sel if not on(r) and r['sx'] != NONE]
    if not off: print(f"{name}: no off-package records"); return
    w = max(off, key=lambda r: r['sx'])
    D = w['sx'] + dmax; P = max(math.ceil(1.1 * D) + 74, 2900); deq = max(0, PEQ - P)
    top = sorted(off, key=lambda r: -r['sx'])[:5]
    print(f"{name}: {len(off)} off-package accesses; worst stash ready {w['sx']} (launch {w['launch']} cmd {w['cmd']}), "
          f"decisions at most {dmax} -> D_max {D}, P = {P}, D_EQ = {deq}, {300e6 / P / 1e3:.1f} KQPS")
    print("    worst five (launch, cmd, stash ready): " + ', '.join(f"({r['launch']}, {r['cmd']}, {r['sx']})" for r in top))
print(f"decisions' duration on the card (on-package ea - sx): max {dmax}, over {len(dec)} accesses")
report("all accesses", rows)
report("without each launch's first command", [r for r in rows if (r['launch'], r['cmd']) not in first])
