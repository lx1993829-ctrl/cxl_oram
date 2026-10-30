#!/usr/bin/env python3
"""Make a copy of a BOLT 20%-HBM host program whose timed run issues a chosen number of commands (e.g. 20,000).
BOLT's kernels and the original host program are not changed.

Usage:  python3 bolt_cmds_patch.py host_64.cpp        -> writes cmd_host_64.cpp next to it
        g++ cmd_host_64.cpp -o bolt_64 -DINIT_SIZE=1000000 -DCMD_SIZE=20000 <XRT flags>
        BOLT_CMDS=20000 ./bolt_64 load_balance_1d_array_20_percent_hbm_64_value_size.xclbin

Works on hbm_distribution_experiment/20_percent_hbm/host.cpp (8 B) and value_scaling_experiment/20_percent_hbm/
value_size_{16,64,128,256}/host_*.cpp and value_size_32/padding_host_32.cpp. The copy always starts from the released
file (git HEAD if the folder is a git clone, else the .orig that bolt_scale_patch.py left, else the file itself).

Changes in the copy (and nothing else):
  1. INIT_SIZE (entries) and CMD_SIZE (largest batch) can be set with g++ -D...      (defaults stay 1M and 5000)
  2. the timed batch has BOLT_CMDS commands (environment variable, default CMD_SIZE, at most CMD_SIZE).
     The commands come from BOLT's own generate_uniform_commands(): uniform keys over all entries, each a PUT with
     probability 0.5, seed 1. Its first 5000 commands are exactly the batch of the 5000-command runs.
  3. the `sudo` drop-caches call in main is commented out                            (no sudo on the server)
  4. after the timed run (so the timing is unaffected), it prints
       TIMED commands=... seconds=... us_per_command=... kqps=...
       RESP acked=<responses of type ACK> of <commands> last_acked_index=...
            every command, GET or PUT, should be answered with ACK; a kernel that stopped early leaves the tail empty
       GET check: <wrong> wrong of <gets> (on keys initialization could not place: <n>)
            each GET must return the latest value written to its key, counting PUTs earlier in the same batch
       WORKLOAD commands=... distinct_keys=... repeat_accesses=... gets_after_put_same_key=...
            repeat_accesses: commands whose key an earlier command of the batch already used;
            gets_after_put_same_key: GETs whose key was PUT earlier in the batch (read-after-write cases checked)
       RESIDENCY before: <type>=<keys> ... after: <type>=<keys> ... entries_before=... entries_after=... missing_keys=...
            location types over all keys in the position map, as initialization wrote them and as the kernel left
            them (0 = HBM, 1 = host page, as BOLT's host code writes them; 2 = eviction buffer, set by the kernel).
            missing_keys: placed keys no longer found in the position map
       PAGES evict_entries=... max_evict_per_page=... max_page_count=... pages_over_capacity=...
            eviction-buffer entries in use, and the kernel's per-page tuple counts (capacity PAGE_CAPACITY = 8)
"""
import os, re, subprocess, sys

src_path = sys.argv[1]
d, name = os.path.split(os.path.abspath(src_path))
out_path = os.path.join(d, "cmd_" + name)

src = None
try:   # the released file, if this is a git clone
    top = subprocess.run(["git", "-C", d, "rev-parse", "--show-toplevel"], capture_output=True, text=True, check=True).stdout.strip()
    rel = os.path.relpath(os.path.join(d, name), top)
    src = subprocess.run(["git", "-C", top, "show", "HEAD:" + rel], capture_output=True, text=True, check=True).stdout
    base = "git HEAD:" + rel
except Exception:
    pass
if src is None:
    base = src_path + ".orig" if os.path.exists(src_path + ".orig") else src_path
    src = open(base).read()
if "GET check:" in src:
    sys.exit(f"{base} is already patched; restore BOLT's file first (git checkout -- {name})")

# 1. INIT_SIZE and CMD_SIZE overridable
for macro in ("INIT_SIZE", "CMD_SIZE"):
    src, n = re.subn(rf"^#define {macro} (\d+)", rf"#ifndef {macro}\n#define {macro} \1\n#endif", src, count=1, flags=re.M)
    assert n == 1, f"no '#define {macro}' found"

# 3. no sudo
src = re.sub(r'^(\s*)(system\("sync && echo 3 \| sudo tee /proc/sys/vm/drop_caches"\);)', r"\1// \2", src, flags=re.M)

# the value field is "blocks" (32-256 B programs) or "value" (8-16 B programs)
field = "blocks" if re.search(r"struct KV_TUPLE\s*\{[^}]*\bblocks\b", src) else "value"

# 2. batch size from BOLT_CMDS, in the timed function only
m = re.search(r"^void run_init_and_chain_kernel_\w*\(", src, flags=re.M)
assert m, "no timed run function found"
HELPER = r'''
// ---- added by bolt_cmds_patch.py: batch size of the timed run ----
static int bolt_cmds() {
    const char* v = std::getenv("BOLT_CMDS");
    const long n = v ? std::strtol(v, nullptr, 10) : (long)CMD_SIZE;
    if (n < 1 || n > (long)CMD_SIZE) {
        std::cerr << "BOLT_CMDS must be 1.." << CMD_SIZE << " (rebuild with a larger -DCMD_SIZE)" << std::endl;
        std::exit(2);
    }
    return (int)n;
}
// ---- end of addition ----

'''
timed_start = m.start()
head, timed = src[:timed_start], src[timed_start:]
timed, n = re.subn(r"^(\s*const int command_size\s*=\s*)CMD_SIZE(\s*;)", r"\1bolt_cmds()\2", timed, count=1, flags=re.M)
assert n == 1, "no 'const int command_size = CMD_SIZE;' in the timed function"
assert re.search(r"generate_uniform_commands\s*\(\s*command_size\s*,\s*init_tuples\s*\)", timed), \
    "the timed function does not call generate_uniform_commands(command_size, init_tuples)"

# 4. checks after the timed run's response read-back (the last resp_bo sync in the file)
CHECK = r'''
    {   // ---- added by bolt_cmds_patch.py: read back after the timed run, so the timing is unaffected ----
        const size_t ncmd = commands.size();
        std::cout << "TIMED commands=" << ncmd << " seconds=" << kernel_time_in_sec
                  << " us_per_command=" << kernel_time_in_sec / ncmd * 1e6
                  << " kqps=" << ncmd / kernel_time_in_sec / 1e3 << std::endl;
        const Response_Packet* resp = resp_bo.map<Response_Packet*>();
        size_t acked = 0; long long last_acked = -1;
        for (size_t i = 0; i < ncmd; i++) if (resp[i].type == RESP_ACK) { acked++; last_acked = (long long)i; }
        std::cout << "RESP acked=" << acked << " of " << ncmd << " last_acked_index=" << last_acked << std::endl;

        std::unordered_map<uint32_t, const unsigned char*> latest;
        latest.reserve(2 * init_tuples.size());
        for (const auto& t : init_tuples) latest[t.key] = reinterpret_cast<const unsigned char*>(&t.FIELD);
        std::unordered_set<uint32_t> touched, put_keys;
        touched.reserve(2 * ncmd);
        size_t bad = 0, bad_unplaced = 0, gets = 0, repeats = 0, raw = 0;
        // where the wrong replies are, and what they contain
        std::unordered_map<uint32_t, std::vector<const unsigned char*>> hist;   // values written to a key, oldest first
        std::unordered_map<uint32_t, size_t> last_put, last_acc;
        long long first_bad = -1;
        size_t c_stale = 0, c_zero = 0, c_other = 0, c_noack = 0, c_never_put = 0;
        std::vector<size_t> per_win((ncmd + 99999) / 100000, 0);
        std::vector<std::string> samples;
        for (size_t i = 0; i < ncmd; i++) {
            const uint32_t k = commands[i].tuple.key;
            if (!touched.insert(k).second) repeats++;
            if (commands[i].type == CMD_PUT) {
                auto& h = hist[k];
                if (h.empty()) h.push_back(latest[k]);                              // the initial value
                latest[k] = reinterpret_cast<const unsigned char*>(&commands[i].tuple.FIELD);
                h.push_back(latest[k]);
                put_keys.insert(k);
                last_put[k] = i; last_acc[k] = i;
                continue;
            }
            gets++;
            if (put_keys.count(k)) raw++;
            if (resp[i].type != RESP_ACK || std::memcmp(&resp[i].FIELD, latest[k], VALUE_SIZE) != 0) {
                bad++;
                if (insert_error_keys.count(k)) bad_unplaced++;
                if (first_bad < 0) first_bad = (long long)i;
                per_win[i / 100000]++;
                const char* cls = "other"; long back = -1;
                const unsigned char* r = reinterpret_cast<const unsigned char*>(&resp[i].FIELD);
                auto hp = hist.find(k);
                if (resp[i].type != RESP_ACK) { cls = "no_ack"; c_noack++; }
                else {
                    if (hp != hist.end())                                            // an older value of this key?
                        for (long j = (long)hp->second.size() - 2; j >= 0; j--)
                            if (std::memcmp(r, hp->second[j], VALUE_SIZE) == 0) { back = (long)hp->second.size() - 1 - j; break; }
                    bool zero = true;
                    for (int b = 0; b < VALUE_SIZE; b++) if (r[b]) { zero = false; break; }
                    if (back > 0) { cls = "stale"; c_stale++; }
                    else if (zero) { cls = "zeros"; c_zero++; }
                    else c_other++;
                }
                if (hp == hist.end()) c_never_put++;
                if (samples.size() < 20) {
                    auto lp = last_put.find(k), la = last_acc.find(k);
                    samples.push_back("GETERR_SAMPLE index=" + std::to_string(i) + " key=" + std::to_string(k) + " class=" + cls +
                        " versions_back=" + std::to_string(back) +
                        " since_last_put=" + (lp == last_put.end() ? std::string("never") : std::to_string(i - lp->second)) +
                        " since_last_access=" + (la == last_acc.end() ? std::string("never") : std::to_string(i - la->second)) +
                        " placed_at_init=" + (hbm_key_to_index.count(k) ? "hbm" : "host"));
                }
            }
            last_acc[k] = i;
        }
        std::cout << "GET check: " << bad << " wrong of " << gets << " (on keys initialization could not place: "
                  << bad_unplaced << ")" << std::endl;
        std::cout << "GETERR wrong=" << bad << " first_index=" << first_bad << " stale=" << c_stale << " zeros=" << c_zero
                  << " other=" << c_other << " no_ack=" << c_noack << " key_never_put=" << c_never_put << " per_100k_commands=";
        for (size_t w = 0; w < per_win.size(); w++) std::cout << (w ? "," : "") << per_win[w];
        std::cout << std::endl;
        for (const auto& s : samples) std::cout << s << std::endl;
        std::cout << "WORKLOAD commands=" << ncmd << " distinct_keys=" << touched.size() << " repeat_accesses=" << repeats
                  << " gets_after_put_same_key=" << raw << std::endl;

        auto by_type = [](const Attribute* pm, std::map<int, size_t>& m) {
            size_t e = 0;
            for (size_t i = 0; i < (size_t)HASH_SIZE * HBM_CAPACITY; i++)
                if (pm[i].key != (uint32_t)EMPTY_KEY) { m[pm[i].location_type]++; e++; }
            return e;
        };
        std::map<int, size_t> before, after;
        const size_t e0 = by_type(&position_map[0][0], before);       // what initialization wrote
        bos[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Attribute* pm = bos[0].map<Attribute*>();                // what the kernel left
        const size_t e1 = by_type(pm, after);
        size_t missing = 0;
        for (const auto& t : init_tuples) {
            if (insert_error_keys.count(t.key)) continue;
            const size_t h = t.key % HASH_SIZE; bool found = false;
            for (int j = 0; j < HBM_CAPACITY && !found; j++) found = pm[h * HBM_CAPACITY + j].key == t.key;
            if (!found) missing++;
        }
        std::cout << "RESIDENCY before:";
        for (const auto& kv : before) std::cout << " " << kv.first << "=" << kv.second;
        std::cout << " after:";
        for (const auto& kv : after) std::cout << " " << kv.first << "=" << kv.second;
        std::cout << " entries_before=" << e0 << " entries_after=" << e1 << " missing_keys=" << missing << std::endl;

        bos[5].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Eviction_Buffer_Block* eb = bos[5].map<Eviction_Buffer_Block*>();
        bos[3].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const uint32_t* cnt = bos[3].map<uint32_t*>();
        size_t eb_used = 0, eb_max = 0, cnt_max = 0, over = 0;
        for (size_t p = 0; p < (size_t)PAGE_SIZE; p++) {
            size_t e = 0;
            for (int j = 0; j < PAGE_CAPACITY; j++) if (eb[p * PAGE_CAPACITY + j].key != (uint32_t)EMPTY_KEY) e++;
            eb_used += e; eb_max = std::max(eb_max, e);
            cnt_max = std::max(cnt_max, (size_t)cnt[p]);
            if (cnt[p] > (uint32_t)PAGE_CAPACITY) over++;
        }
        std::cout << "PAGES evict_entries=" << eb_used << " max_evict_per_page=" << eb_max << " max_page_count=" << cnt_max
                  << " pages_over_capacity=" << over << std::endl;
    }
'''.replace("FIELD", field)
marker = "resp_bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE);"
pos = timed.rfind(marker)
assert pos >= 0, "no response read-back in the timed function"
timed = timed[:pos + len(marker)] + CHECK + timed[pos + len(marker):]

src = head + HELPER + timed
src = "#include <algorithm>\n#include <cstdlib>\n#include <map>\n#include <unordered_map>\n#include <unordered_set>\n" + src
open(out_path, "w").write(src)
print("wrote", out_path, "from", base, "(value field:", field + ")")
