#!/usr/bin/env python3
"""Make a copy of a BOLT 20%-HBM host program that times query batches whose keys all start in HBM or all start
in host memory. BOLT's kernels and the original host program are not changed.

Usage:  python3 bolt_location_patch.py host_64.cpp        -> writes loc_host_64.cpp next to it
        (reads host_64.cpp.orig instead if bolt_scale_patch.py left one, so the copy starts from the release)

Changes in the copy (and nothing else):
  1. INIT_SIZE can be set with g++ -DINIT_SIZE=...            (default stays 1M)
  2. the `sudo` drop-caches call in main is commented out     (no sudo on the server)
  3. after the timed run, every GET reply is compared with the latest value written to its key:
     prints "GET check: <wrong> wrong of <gets>"
  4. the timed run's command list comes from generate_location_batch() instead of generate_uniform_commands().
     Environment variables choose the batch:
       BOLT_BATCH    hbm | host | mixed | uniform   (uniform = the original generator, unchanged)
       BOLT_SEED     integer, default 1             (which keys, and the GET/PUT sequence)
       BOLT_PUT_FRAC fraction of PUTs, default 0.5  (the original generator's write ratio)
     hbm:   CMD_SIZE distinct keys, all placed in HBM by the host's initialization (location_type 0)
     host:  CMD_SIZE distinct keys, all placed in host pages (location_type 1)
     mixed: CMD_SIZE distinct keys drawn from all placed keys (about 20% HBM, as in the paper's runs)
     Each key is queried once, so every query finds its key where initialization put it (no earlier remap).
     The GET/PUT sequence and the PUT values depend only on BOLT_SEED, so batches with the same seed differ only
     in where their keys live. The program prints one line describing the batch:
       BATCH kind=hbm seed=3 size=5000 get=2489 put=2511 hbm_resident=5000 host_resident=0
  5. after the timed run (so the timing is unaffected), the program reads back the host pages, the position map,
     the eviction buffer and the per-page counts, compares them with what initialization wrote, and prints
       STATE pages_changed=... of_which_start_pages=... start_pages=... eviction_buffer_entries=...
             page_counts_changed=... queried_keys_location_type: 0=... 1=... missing=...
     pages_changed: host pages whose bytes differ after the run. The kernel reaches host pages only through its
     `pages` port (HOST[0]); a page whose bytes changed was written by the kernel. A write that stores the same
     bytes, and any read, leave no trace here.
     start_pages: distinct pages that held a queried key at the start; of_which_start_pages: how many of those
     changed (a check that the method sees the writes queries to host keys should make).
     queried_keys_location_type: where the position map says each queried key is after the run
     (0 = HBM, 1 = host page; other codes are the kernel's own, e.g. the eviction buffer).
"""
import os, re, shutil, sys

src_path = sys.argv[1]
base = src_path + ".orig" if os.path.exists(src_path + ".orig") else src_path
src = open(base).read()
out_path = os.path.join(os.path.dirname(src_path), "loc_" + os.path.basename(src_path))

# 1. INIT_SIZE overridable
src, n = re.subn(r"^#define INIT_SIZE (\d+)", r"#ifndef INIT_SIZE\n#define INIT_SIZE \1\n#endif", src, count=1, flags=re.M)
assert n == 1, "no '#define INIT_SIZE' found"

# 2. no sudo
src = re.sub(r'^(\s*)(system\("sync && echo 3 \| sudo tee /proc/sys/vm/drop_caches"\);)', r"\1// \2", src, flags=re.M)

# the value field is "blocks" (64-256 B programs) or "value" (8-16 B programs)
field = "blocks" if re.search(r"struct KV_TUPLE\s*\{[^}]*\bblocks\b", src) else "value"

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
    {   // added: what the run changed, read back after the timed run
        std::unordered_set<uint32_t> start_pages;   // pages that held a queried key at the start
        for (const auto& c : commands) {
            const uint32_t k = c.tuple.key; const size_t h = k % HASH_SIZE;
            for (int j = 0; j < HBM_CAPACITY; j++)
                if (position_map[h][j].key == k) {
                    if (position_map[h][j].location_type == 1) start_pages.insert(position_map[h][j].page_index);
                    break;
                }
        }
        bos[4].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Page* pg = bos[4].map<Page*>();
        size_t changed = 0, changed_start = 0;
        for (size_t i = 0; i < (size_t)PAGE_SIZE; i++)
            if (std::memcmp(&pg[i], &pages[i], sizeof(Page)) != 0) { changed++; if (start_pages.count(i)) changed_start++; }
        bos[0].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Attribute* pm = bos[0].map<Attribute*>();
        std::map<int, int> now; int missing = 0;
        for (const auto& c : commands) {
            const uint32_t k = c.tuple.key; const size_t h = k % HASH_SIZE; bool found = false;
            for (int j = 0; j < HBM_CAPACITY; j++) {
                const Attribute& a = pm[h * HBM_CAPACITY + j];
                if (a.key == k) { now[a.location_type]++; found = true; break; }
            }
            if (!found) missing++;
        }
        bos[5].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const Eviction_Buffer_Block* eb = bos[5].map<Eviction_Buffer_Block*>();
        size_t eb_used = 0;
        for (size_t i = 0; i < (size_t)PAGE_SIZE * PAGE_CAPACITY; i++) if (eb[i].key != (uint32_t)EMPTY_KEY) eb_used++;
        bos[3].sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        const uint32_t* cnt = bos[3].map<uint32_t*>();
        size_t cnt_changed = 0;
        for (size_t i = 0; i < (size_t)PAGE_SIZE; i++) if (cnt[i] != (uint32_t)page_tuple_count[i]) cnt_changed++;
        std::cout << "STATE pages_changed=" << changed << " of_which_start_pages=" << changed_start
                  << " start_pages=" << start_pages.size() << " eviction_buffer_entries=" << eb_used
                  << " page_counts_changed=" << cnt_changed << " queried_keys_location_type:";
        for (const auto& kv : now) std::cout << " " << kv.first << "=" << kv.second;
        std::cout << " missing=" << missing << std::endl;
    }
'''.replace("FIELD", field)
marker = "resp_bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE);"
pos = src.rfind(marker)
assert pos >= 0, "no response read-back found"
src = src[:pos + len(marker)] + CHECK + src[pos + len(marker):]

# 4. location-controlled batches
GEN = r'''
// ---- added by bolt_location_patch.py: query batches whose keys all start in HBM or all in host pages ----
static const char* loc_env(const char* name, const char* dflt) { const char* v = std::getenv(name); return v ? v : dflt; }

std::vector<Command_Packet> generate_location_batch(int size, const std::vector<KV_TUPLE>& init_tuples) {
    const std::string kind = loc_env("BOLT_BATCH", "uniform");
    if (kind == "uniform") return generate_uniform_commands(size, init_tuples);
    const uint32_t seed = (uint32_t)std::strtoul(loc_env("BOLT_SEED", "1"), nullptr, 10);
    const double put_frac = std::atof(loc_env("BOLT_PUT_FRAC", "0.5"));

    // candidate keys by the location the host's initialization gave them
    std::vector<uint32_t> pool;
    if (kind == "hbm") pool = hbm_sets;
    else if (kind == "host") pool = host_sets;
    else if (kind == "mixed") { pool = hbm_sets; pool.insert(pool.end(), host_sets.begin(), host_sets.end()); }
    else { std::cerr << "BOLT_BATCH must be hbm, host, mixed or uniform" << std::endl; std::exit(2); }
    if ((int)pool.size() < size) { std::cerr << "only " << pool.size() << " keys of kind " << kind << std::endl; std::exit(2); }

    // size distinct keys (partial Fisher-Yates), from a key generator separate from the GET/PUT generator
    std::mt19937 key_gen(seed * 2654435761u + 12345u);
    for (int i = 0; i < size; i++) {
        std::uniform_int_distribution<size_t> pick(i, pool.size() - 1);
        std::swap(pool[i], pool[pick(key_gen)]);
    }

    // GET/PUT sequence and PUT values: depend only on the seed, identical for hbm, host and mixed batches
    std::mt19937 op_gen(seed);
    std::uniform_real_distribution<double> op_dis(0.0, 1.0);
    std::uniform_int_distribution<int> byte_dis(0, 255);
    std::vector<Command_Packet> commands(size);
    int gets = 0, puts = 0, in_hbm = 0, in_host = 0;
    for (int i = 0; i < size; i++) {
        Command_Packet& p = commands[i];
        std::memset(&p, 0, sizeof(p));
        const uint32_t k = pool[i];
        p.tuple.key = k;
        auto h = hbm_key_to_index.find(k);
        if (h != hbm_key_to_index.end()) { record[i] = h->second; in_hbm++; }
        else { record[i] = host_key_to_index.at(k); in_host++; }
        if (op_dis(op_gen) < put_frac) {
            p.type = CMD_PUT; puts++;
            unsigned char* v = reinterpret_cast<unsigned char*>(&p.tuple.FIELD);
            for (int b = 0; b < VALUE_SIZE; b++) v[b] = (unsigned char)byte_dis(op_gen);
        } else {
            p.type = CMD_GET; gets++;
        }
    }
    std::cout << "BATCH kind=" << kind << " seed=" << seed << " size=" << size << " get=" << gets << " put=" << puts
              << " hbm_resident=" << in_hbm << " host_resident=" << in_host << std::endl;
    return commands;
}
// ---- end of addition ----

'''.replace("FIELD", field)
m = re.search(r"^void run_init_and_chain_kernel_\w*\(", src, flags=re.M)
assert m, "no timed run function found"
timed_start = m.start()
src = src[:timed_start] + GEN + src[timed_start:]

# the timed function's call to the original generator -> the new one (exactly one active call)
timed = src[timed_start + len(GEN):]
pat = re.compile(r"^(\s*std::vector<Command_Packet>\s+commands\s*=\s*)generate_uniform_commands(\s*\(\s*command_size\s*,\s*init_tuples\s*\)\s*;)", re.M)
timed, n = pat.subn(r"\1generate_location_batch\2", timed)
assert n == 1, f"expected one generate_uniform_commands call in the timed function, found {n}"
src = src[:timed_start + len(GEN)] + timed

# headers the addition uses
src = "#include <algorithm>\n#include <cstdlib>\n#include <map>\n#include <string>\n#include <unordered_set>\n" + src

open(out_path, "w").write(src)
print("wrote", out_path, "from", base, "(value field:", field + ")")
