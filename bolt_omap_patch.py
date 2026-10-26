#!/usr/bin/env python3
"""Make a copy of a BOLT host program (20%-HBM or all-HBM "baseline" build) for measuring what key handling costs.
BOLT's kernels (.xclbin) and the original host program are not changed; every experiment is a choice of keys and
commands made on the host.

Usage:  python3 bolt_omap_patch.py host_64.cpp        -> writes omap_host_64.cpp next to it
        g++ omap_host_64.cpp -o omap_64 -DINIT_SIZE=1000000 -DCMD_SIZE=20000 <XRT flags>
        BOLT_KEYS=collide BOLT_BATCH=slot15 BOLT_CMDS=10000 ./omap_64 <xclbin>

The copy starts from the released file (git HEAD in a git clone, else <file>.orig, else the file).
Changes in the copy:
  1. INIT_SIZE and CMD_SIZE can be set with g++ -D...; the `sudo` drop-caches call is commented out.
  2. BOLT_KEYS chooses the keys initialization stores (values, placement and everything else stay BOLT's):
       random   BOLT's own: distinct uniformly random 32-bit keys (seed 16)                         [default]
       dense    keys 0..N-1. BOLT's bucket index is key mod HASH_SIZE (3*2^20), so each key gets its own bucket
                and sits in slot 0 of it: in effect an identity hash (needs N <= 3,145,728)
       collide  keys b + m*HASH_SIZE (b = 0..N/16-1, m = 0..15): every bucket is full, and key m sits in slot m
  3. BOLT_BATCH chooses the timed commands (BOLT_CMDS of them, at most CMD_SIZE):
       uniform  BOLT's generate_uniform_commands(): uniform keys, PUT with probability 0.5, seed 1        [default]
       hbm      distinct keys that initialization placed in HBM (location type 0)
       host     distinct keys that initialization placed in host pages (type 1)
       slotM    distinct HBM keys in position-map slot M of their bucket (M = 0..15)
       miss     GETs of keys that are not stored (random keys not in the map). OFF by default in the run script:
                the released kernels were never shown handling absent keys, and a hung kernel needs a card reset.
     For hbm/host/slotM: BOLT_SEED (default 1) picks the keys and the GET/PUT sequence, BOLT_PUT_FRAC (default 0.5)
     the PUT share; keys are distinct, so each is queried once from where initialization put it.
  4. After the timed run (so the timing is unaffected) it prints:
       KEYS / BATCH        what was stored and queried
       TIMED               commands, seconds, us_per_command, kqps (one kernel call)
       RESP                replies of type ACK (every command to a stored key should be ACKed)
       GET check: <wrong> wrong of <gets> ...   each GET against the latest value written to its key
       ABSENT              GETs of keys not stored (miss batches): how many came back ERR / ACK
       WORKLOAD            distinct keys, repeats, GETs after a PUT to the same key
       RESIDENCY           location types over all keys before and after (0 HBM, 1 host page, 2 eviction buffer)
       PAGES               eviction-buffer use and the kernel's per-page counts
       FINAL               every stored key's value, found through the position map the kernel left
                           (type 0: value array; type 1: host page tuple; type 2: value array via the eviction-buffer
                           entry), compared with the latest value written: <matching>/<keys> per type
       PAGEWRITES          host-page tuples the kernel wrote during the run (bytes differ from initialization):
                           current = the position map points at that tuple and it holds the latest value in plaintext;
                           stale = a stored key whose entry now points elsewhere; unknown_key = not a stored key
       FOOTPRINT           bytes of every buffer the kernels are given, and position-map entries in use
"""
import os, re, subprocess, sys

src_path = sys.argv[1]
d, name = os.path.split(os.path.abspath(src_path))
out_path = os.path.join(d, "omap_" + name)

src = None
try:
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

for macro in ("INIT_SIZE", "CMD_SIZE"):
    src, n = re.subn(rf"^#define {macro} (\d+)", rf"#ifndef {macro}\n#define {macro} \1\n#endif", src, count=1, flags=re.M)
    assert n == 1, f"no '#define {macro}' found"
src = re.sub(r'^(\s*)(system\("sync && echo 3 \| sudo tee /proc/sys/vm/drop_caches"\);)', r"\1// \2", src, flags=re.M)
field = "blocks" if re.search(r"struct KV_TUPLE\s*\{[^}]*\bblocks\b", src) else "value"

m = re.search(r"^void run_init_and_chain_kernel_\w*\(", src, flags=re.M)
assert m, "no timed run function found"
head, timed = src[:m.start()], src[m.start():]

HELPER = r'''
// ---- added by bolt_omap_patch.py ----
static std::string omap_env(const char* n, const char* d) { const char* v = std::getenv(n); return v ? v : d; }

static void bolt_rekey(std::vector<KV_TUPLE>& t) {         // BOLT_KEYS: which keys initialization stores
    const std::string mode = omap_env("BOLT_KEYS", "random");
    if (mode == "dense") {
        if (t.size() > (size_t)HASH_SIZE) { std::cerr << "BOLT_KEYS=dense needs N <= " << HASH_SIZE << std::endl; std::exit(2); }
        for (size_t i = 0; i < t.size(); i++) t[i].key = (uint32_t)i;
    } else if (mode == "collide") {
        if (t.size() / HBM_CAPACITY > (size_t)HASH_SIZE) { std::cerr << "BOLT_KEYS=collide: N too large" << std::endl; std::exit(2); }
        for (size_t i = 0; i < t.size(); i++)
            t[i].key = (uint32_t)(i / HBM_CAPACITY + (uint64_t)(i % HBM_CAPACITY) * HASH_SIZE);
    } else if (mode != "random") { std::cerr << "BOLT_KEYS must be random, dense or collide" << std::endl; std::exit(2); }
    std::cout << "KEYS mode=" << mode << " n=" << t.size() << std::endl;
}

static long omap_find(const Attribute* pm, uint32_t k) {     // position-map entry of key k, or -1
    const size_t h = k % HASH_SIZE;
    for (int j = 0; j < HBM_CAPACITY; j++) if (pm[h * HBM_CAPACITY + j].key == k) return (long)(h * HBM_CAPACITY + j);
    return -1;
}

static int bolt_cmds() {
    const long n = std::strtol(omap_env("BOLT_CMDS", std::to_string(CMD_SIZE).c_str()).c_str(), nullptr, 10);
    if (n < 1 || n > (long)CMD_SIZE) { std::cerr << "BOLT_CMDS must be 1.." << CMD_SIZE << std::endl; std::exit(2); }
    return (int)n;
}

std::vector<Command_Packet> bolt_batch(int size, std::vector<KV_TUPLE>& init_tuples, const Attribute* pm) {
    const std::string kind = omap_env("BOLT_BATCH", "uniform");
    if (kind == "uniform") {
        std::cout << "BATCH kind=uniform size=" << size << std::endl;
        return generate_uniform_commands(size, init_tuples);
    }
    const uint32_t seed = (uint32_t)std::strtoul(omap_env("BOLT_SEED", "1").c_str(), nullptr, 10);
    double put_frac = std::atof(omap_env("BOLT_PUT_FRAC", "0.5").c_str());
    std::vector<uint32_t> pool;
    int want_slot = -1;
    if (kind == "miss") {
        put_frac = 0.0;
        std::unordered_set<uint32_t> stored; stored.reserve(2 * init_tuples.size());
        for (const auto& t : init_tuples) stored.insert(t.key);
        std::mt19937 g(seed * 2654435761u + 777u);
        std::unordered_set<uint32_t> picked;
        while ((int)pool.size() < size) {
            const uint32_t k = g();
            if (k != (uint32_t)EMPTY_KEY && !stored.count(k) && picked.insert(k).second) pool.push_back(k);
        }
    } else {
        if (kind.rfind("slot", 0) == 0) want_slot = std::atoi(kind.c_str() + 4);
        else if (kind != "hbm" && kind != "host") { std::cerr << "BOLT_BATCH: uniform, hbm, host, slot<M> or miss" << std::endl; std::exit(2); }
        for (const auto& t : init_tuples) {
            const long e = omap_find(pm, t.key);
            if (e < 0) continue;
            const int ty = pm[e].location_type, slot = (int)(e % HBM_CAPACITY);
            if (kind == "host" ? ty == 1 : (ty == 0 && (want_slot < 0 || slot == want_slot))) pool.push_back(t.key);
        }
        if ((int)pool.size() < size) {
            std::cerr << "only " << pool.size() << " keys for BOLT_BATCH=" << kind << "; lower BOLT_CMDS" << std::endl;
            std::exit(2);
        }
        std::mt19937 kg(seed * 2654435761u + 12345u);
        for (int i = 0; i < size; i++) {
            std::uniform_int_distribution<size_t> pick(i, pool.size() - 1);
            std::swap(pool[i], pool[pick(kg)]);
        }
    }
    std::mt19937 og(seed);
    std::uniform_real_distribution<double> op(0.0, 1.0);
    std::uniform_int_distribution<int> byte(0, 255);
    std::vector<Command_Packet> cmds(size);
    int gets = 0, puts = 0;
    for (int i = 0; i < size; i++) {
        Command_Packet& p = cmds[i];
        std::memset(&p, 0, sizeof(p));
        p.tuple.key = pool[i];
        if (op(og) < put_frac) {
            p.type = CMD_PUT; puts++;
            unsigned char* v = reinterpret_cast<unsigned char*>(&p.tuple.FIELD);
            for (int b = 0; b < VALUE_SIZE; b++) v[b] = (unsigned char)byte(og);
        } else { p.type = CMD_GET; gets++; }
    }
    std::cout << "BATCH kind=" << kind << " size=" << size << " get=" << gets << " put=" << puts
              << " pool=" << pool.size() << " seed=" << seed << std::endl;
    return cmds;
}
// ---- end of addition ----

'''.replace("FIELD", field)

timed, n = re.subn(r"^(\s*const int command_size\s*=\s*)CMD_SIZE(\s*;)", r"\1bolt_cmds()\2", timed, count=1, flags=re.M)
assert n == 1, "no 'const int command_size = CMD_SIZE;' in the timed function"
timed, n = re.subn(r"generate_uniform_commands\s*\(\s*command_size\s*,\s*init_tuples\s*\)",
                   "bolt_batch(command_size, init_tuples, &position_map[0][0])", timed, count=1)
assert n == 1, "the timed function does not call generate_uniform_commands(command_size, init_tuples)"
timed, n = re.subn(r"^(\s*std::vector<KV_TUPLE>\s+init_tuples\s*=\s*generate_distributed_init_tuples\s*\(\s*INIT_SIZE\s*\)\s*;)",
                   r"\1\n    bolt_rekey(init_tuples);", timed, count=1, flags=re.M)
assert n == 1, "main does not call generate_distributed_init_tuples(INIT_SIZE)"

CHECK = r'''
    {   // ---- added by bolt_omap_patch.py: read back after the timed run, so the timing is unaffected ----
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
        size_t bad = 0, bad_unplaced = 0, gets = 0, repeats = 0, raw = 0, absent = 0, absent_err = 0, absent_ack = 0;
        for (size_t i = 0; i < ncmd; i++) {
            const uint32_t k = commands[i].tuple.key;
            if (!touched.insert(k).second) repeats++;
            auto it = latest.find(k);
            if (it == latest.end()) {                                  // key not stored (miss batches)
                absent++; if (resp[i].type == RESP_ACK) absent_ack++; else if (resp[i].type == RESP_ERR) absent_err++;
                continue;
            }
            if (commands[i].type == CMD_PUT) {
                it->second = reinterpret_cast<const unsigned char*>(&commands[i].tuple.FIELD);
                put_keys.insert(k);
                continue;
            }
            gets++;
            if (put_keys.count(k)) raw++;
            if (resp[i].type != RESP_ACK || std::memcmp(&resp[i].FIELD, it->second, VALUE_SIZE) != 0) {
                bad++;
                if (insert_error_keys.count(k)) bad_unplaced++;
            }
        }
        std::cout << "GET check: " << bad << " wrong of " << gets << " (on keys initialization could not place: "
                  << bad_unplaced << ")" << std::endl;
        if (absent) std::cout << "ABSENT gets=" << absent << " answered_err=" << absent_err << " answered_ack=" << absent_ack << std::endl;
        std::cout << "WORKLOAD commands=" << ncmd << " distinct_keys=" << touched.size() << " repeat_accesses=" << repeats
                  << " gets_after_put_same_key=" << raw << std::endl;

        auto by_type = [](const Attribute* pm, std::map<int, size_t>& m) {
            size_t e = 0;
            for (size_t i = 0; i < (size_t)HASH_SIZE * HBM_CAPACITY; i++)
                if (pm[i].key != (uint32_t)EMPTY_KEY) { m[pm[i].location_type]++; e++; }
            return e;
        };
        std::map<int, size_t> before, after;
        const size_t e0 = by_type(&position_map[0][0], before);
        bos[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Attribute* pm = bos[0].map<Attribute*>();
        const size_t e1 = by_type(pm, after);
        size_t missing = 0;
        for (const auto& t : init_tuples)
            if (!insert_error_keys.count(t.key) && omap_find(pm, t.key) < 0) missing++;
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

        // final stored values, located through the position map the kernel left
        bos[1].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const unsigned char* vraw = bos[1].map<unsigned char*>();
        const size_t vslot = bos[1].size() / INIT_SIZE;
        bos[4].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Page* pg = bos[4].map<Page*>();
        size_t n_t[3] = {0, 0, 0}, ok_t[3] = {0, 0, 0}, other = 0, out_of_range = 0;
        for (const auto& t : init_tuples) {
            if (insert_error_keys.count(t.key)) continue;
            const long e = omap_find(pm, t.key);
            if (e < 0) continue;
            const Attribute& a = pm[e];
            const unsigned char* have = nullptr;
            const int ty = a.location_type;
            if (ty == 0 && a.location_index < (uint32_t)INIT_SIZE) have = vraw + (size_t)a.location_index * vslot;
            else if (ty == 1 && a.page_index < (uint32_t)PAGE_SIZE && a.location_index < (uint32_t)PAGE_CAPACITY)
                have = reinterpret_cast<const unsigned char*>(&pg[a.page_index].tuples[a.location_index].FIELD);
            else if (ty == 2 && a.page_index < (uint32_t)PAGE_SIZE && a.location_index < (uint32_t)PAGE_CAPACITY) {
                const uint32_t vi = eb[(size_t)a.page_index * PAGE_CAPACITY + a.location_index].value_index;
                if (vi < (uint32_t)INIT_SIZE) have = vraw + (size_t)vi * vslot;
            }
            if (ty > 2) { other++; continue; }
            if (!have) { out_of_range++; continue; }
            n_t[ty]++;
            if (std::memcmp(have, latest[t.key], VALUE_SIZE) == 0) ok_t[ty]++;
        }
        std::cout << "FINAL hbm=" << ok_t[0] << "/" << n_t[0] << " page=" << ok_t[1] << "/" << n_t[1]
                  << " evict=" << ok_t[2] << "/" << n_t[2] << " other_types=" << other << " out_of_range=" << out_of_range << std::endl;

        size_t written = 0, current = 0, stale = 0, unknown = 0;
        for (size_t p = 0; p < (size_t)PAGE_SIZE; p++)
            for (int j = 0; j < PAGE_CAPACITY; j++) {
                if (std::memcmp(&pg[p].tuples[j], &pages[p].tuples[j], sizeof(KV_TUPLE)) == 0) continue;
                const uint32_t k = pg[p].tuples[j].key;
                if (k == (uint32_t)EMPTY_KEY) continue;
                written++;
                auto it = latest.find(k);
                const long e = it == latest.end() ? -1 : omap_find(pm, k);
                if (e < 0) { unknown++; continue; }
                if (pm[e].location_type == 1 && pm[e].page_index == p && pm[e].location_index == (uint32_t)j &&
                    std::memcmp(&pg[p].tuples[j].FIELD, it->second, VALUE_SIZE) == 0) current++;
                else stale++;
            }
        std::cout << "PAGEWRITES kernel_written_tuples=" << written << " current_plaintext=" << current << " stale=" << stale
                  << " unknown_key=" << unknown << std::endl;
        std::cout << "FOOTPRINT position_map=" << bos[0].size() << " value=" << bos[1].size() << " queue=" << bos[2].size()
                  << " page_counts=" << bos[3].size() << " host_pages=" << bos[4].size() << " eviction_buffer=" << bos[5].size()
                  << " map_entry_bytes=" << sizeof(Attribute) << " buckets=" << HASH_SIZE << " slots_per_bucket=" << HBM_CAPACITY
                  << " entries_used=" << e0 << std::endl;
    }
'''.replace("FIELD", field)
marker = "resp_bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE);"
pos = timed.rfind(marker)
assert pos >= 0, "no response read-back in the timed function"
timed = timed[:pos + len(marker)] + CHECK + timed[pos + len(marker):]

src = head + HELPER + timed
src = ("#include <algorithm>\n#include <cstdlib>\n#include <map>\n#include <string>\n#include <unordered_map>\n"
       "#include <unordered_set>\n") + src
open(out_path, "w").write(src)
print("wrote", out_path, "from", base, "(value field:", field + ")")
