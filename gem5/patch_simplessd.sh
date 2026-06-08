#!/bin/bash
# ================================================================
# patch_simplessd.sh — Rename SimpleSSD panic/warn/info functions
# to ssd_panic/ssd_warn/ssd_info to avoid collision with gem5's
# panic() macro defined in base/logging.hh.
#
# Run from gem5 root:  bash patch_simplessd.sh
# ================================================================

set -e

SSDIR="src/mem/ssd/simplessd"

if [ ! -d "$SSDIR" ]; then
    echo "ERROR: $SSDIR not found. Run from gem5 root." >&2
    exit 1
fi

echo "Patching SimpleSSD: panic→ssd_panic, warn→ssd_warn, info→ssd_info"

# 1. Declarations in trace.hh
sed -i 's/^void panic(const char/void ssd_panic(const char/' "$SSDIR/sim/trace.hh"
sed -i 's/^void warn(const char/void ssd_warn(const char/' "$SSDIR/sim/trace.hh"
sed -i 's/^void info(const char/void ssd_info(const char/' "$SSDIR/sim/trace.hh"

# 2. Definitions in log.cc
sed -i 's/^void panic(const char/void ssd_panic(const char/' "$SSDIR/sim/log.cc"
sed -i 's/^void warn(const char/void ssd_warn(const char/' "$SSDIR/sim/log.cc"
sed -i 's/^void info(const char/void ssd_info(const char/' "$SSDIR/sim/log.cc"

# 3. All call sites in SimpleSSD .cc and .hh files
#    Match "panic(" but NOT "ssd_panic(" — use negative lookbehind via perl
find "$SSDIR" \( -name "*.cc" -o -name "*.hh" \) -print0 | \
    xargs -0 perl -pi -e '
        s/(?<!ssd_)(?<!\w)panic\s*\(/ssd_panic(/g;
        s/(?<!ssd_)(?<!\w)warn\s*\(/ssd_warn(/g;
        s/(?<!ssd_)(?<!\w)info\s*\(/ssd_info(/g;
    '

# 4. Verify
echo ""
echo "=== Verify declarations ==="
grep "void ssd_panic\|void ssd_warn\|void ssd_info" "$SSDIR/sim/trace.hh"

echo ""
echo "=== Count renamed calls ==="
COUNT=$(grep -rn "ssd_panic\|ssd_warn\|ssd_info" "$SSDIR" --include="*.cc" --include="*.hh" | wc -l)
echo "$COUNT call sites renamed"

echo ""
echo "=== Check for missed bare calls ==="
MISSED=$(grep -rn '[^_]panic\s*(' "$SSDIR" --include="*.cc" --include="*.hh" | grep -v ssd_panic | grep -v "// " | wc -l)
if [ "$MISSED" -gt 0 ]; then
    echo "WARNING: $MISSED potential missed calls:"
    grep -rn '[^_]panic\s*(' "$SSDIR" --include="*.cc" --include="*.hh" | grep -v ssd_panic | grep -v "// "
else
    echo "Clean — no missed calls"
fi

echo ""
echo "Done. Rebuild gem5 to verify."
