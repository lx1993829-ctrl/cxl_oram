#!/usr/bin/env python3
"""Patch a BOLT value-scaling host program (host_16/64/128/256.cpp) so it can run Fig. 7's data-size sweep.
  1. INIT_SIZE (number of entries) can be set at compile time: g++ -DINIT_SIZE=5000000 ...
  2. the sudo drop-caches call is commented out (no sudo on the server)
  3. after the timed run, every GET response is checked against the latest value written to its key,
     printing "GET check: <wrong> wrong of <gets>" (their timed runs never check responses)
Usage: python3 bolt_scale_patch.py host_64.cpp      (edits the file in place, keeps a .orig copy once)
"""
import re, shutil, sys, os

path = sys.argv[1]
src = open(path).read()
if not os.path.exists(path + ".orig"):
    shutil.copy(path, path + ".orig")

# 1. INIT_SIZE overridable
if "#ifndef INIT_SIZE" not in src:
    src, n = re.subn(r"^#define INIT_SIZE (\d+)", r"#ifndef INIT_SIZE\n#define INIT_SIZE \1\n#endif", src, count=1, flags=re.M)
    assert n == 1, "no '#define INIT_SIZE' found"

# 2. no sudo
src = re.sub(r'^(\s*)(system\("sync && echo 3 \| sudo tee /proc/sys/vm/drop_caches"\);)', r"\1// \2", src, flags=re.M)

# 3. GET check after the timed run's response read-back (the last resp_bo sync in the file)
CHECK = r'''
    {   // added: every GET must return the latest value written to its key
        auto resp = resp_bo.map<Response_Packet*>();
        std::unordered_map<uint32_t, const void*> latest;
        for (const auto& t : init_tuples) latest[t.key] = t.FIELD;
        int bad = 0, gets = 0;
        for (size_t i = 0; i < commands.size(); i++) {
            const uint32_t k = commands[i].tuple.key;
            if (commands[i].type == CMD_PUT) { latest[k] = commands[i].tuple.FIELD; continue; }
            gets++;
            if (resp[i].type != RESP_ACK || std::memcmp(resp[i].FIELD, latest[k], VALUE_SIZE) != 0) bad++;
        }
        std::cout << "GET check: " << bad << " wrong of " << gets << std::endl;
    }
'''
# the value field is "blocks" (64-256 B programs) or "value" (8-16 B programs)
field = "blocks" if re.search(r"struct KV_TUPLE\s*\{[^}]*\bblocks\b", src) else "value"
CHECK = CHECK.replace("FIELD", field)
marker = "resp_bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE);"
if "GET check:" not in src:
    pos = src.rfind(marker)
    assert pos >= 0, "no response read-back found"
    pos += len(marker)
    src = src[:pos] + CHECK + src[pos:]

open(path, "w").write(src)
print("patched", path)
