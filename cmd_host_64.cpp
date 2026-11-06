#include <algorithm>
#include <cstdlib>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <xrt/xrt_uuid.h>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <chrono>
#include <numeric>
#include <vector>
#include <xrt/xrt_bo.h>
#include <iomanip>
#include <vector>
#include <cstring> // For memset
#include <xrt/xrt_bo.h>
#include <CL/cl_ext_xilinx.h>
#define CMD_GET 0x01
#define CMD_PUT 0x02
#define RESP_ACK 0x10
#define RESP_ERR 0x11
#ifndef CMD_SIZE
#define CMD_SIZE 5000
#endif
#define VALUE_SIZE 64   // bytes 
#define PAGE_SIZE 2097152
#define PAGE_CAPACITY 8
#define HBM_CAPACITY 16
#define HASH_SIZE 3145728
#define EMPTY_KEY -1
#ifndef INIT_SIZE
#define INIT_SIZE 1000000
#endif
#define QUEUE_SIZE 10000000
#define BLOCK_SIZE (VALUE_SIZE/16)
using namespace std;

struct Attribute{
    uint32_t key;
    uint8_t  location_type;
    uint32_t page_index; //It is allocated to which page. For kv_tuples inside the host memory or eviction buffer
    uint32_t location_index; // If it is inside the host_mem, then this is its index inside the page. Otherwise, it is the index inside the hbm bank.
};


struct Value_Block{
    unsigned char value[16];
};

struct KV_TUPLE{
    uint32_t key;
    Value_Block blocks[BLOCK_SIZE];
};


struct Page
{
    KV_TUPLE tuples[PAGE_CAPACITY];
};


struct Command_Packet{
    uint8_t type; //command type
    KV_TUPLE tuple; //key-value pair
};

struct Response_Packet{
    uint8_t type; //response type
    Value_Block blocks[BLOCK_SIZE];
};

struct Eviction_Buffer_Block{
    uint32_t key;
    uint32_t value_index;
};

static int bucket[HASH_SIZE];
static int head=0;
static int tail=0;
static std::unordered_set<uint32_t>insert_error_keys;
static int record[CMD_SIZE];// used to record the command number -> index in tuple for test correctness
static int get_cmd_count=0;
static int put_cmd_count=0;
static int hbm_cmd_count=0;
static int host_cmd_count=0;
static std::vector <uint32_t>hbm_sets; //key sets for hbm
static std::vector <uint32_t>host_sets; // key sets for the host memory
static std::unordered_map<uint32_t, int> hbm_key_to_index;  // Maps HBM keys to their indices in init_tuples
static std::unordered_map<uint32_t, int> host_key_to_index; // Maps host keys to their indices in init_tuples

void print_Tuple(KV_TUPLE tuple) {
    printf("The key is %d and the first block's first byte is %d\n", tuple.key, tuple.blocks[0].value[0]);
}

void print_Command_Packet(Command_Packet packet) {
    uint8_t type = packet.type;
    printf("The type is 0x%02X\n", type);
    uint32_t key = packet.tuple.key;
    printf("The key is %d\n", key);
    char value0 = packet.tuple.blocks[0].value[0];
    printf("The first block's first byte is %d\n", value0);
}

void print_Response_Packet(Response_Packet packet) {
    uint8_t type = packet.type;
    printf("The type is 0x%02X\n", type);
    char value0 = packet.blocks[0].value[0];
    printf("The first block's first byte is %d\n", value0);
}

std::vector<xrt::bo> generate_bos_host_key_value_seperate_complete(xrt::device &device,xrt::kernel &init_kernel){
    std::vector<xrt::bo>bos;
    xrt::bo position_map_bo= xrt::bo(device,HASH_SIZE*HBM_CAPACITY*sizeof(Attribute), init_kernel.group_id(0));
    bos.push_back(position_map_bo);
    xrt::bo value_bo= xrt::bo(device,INIT_SIZE* sizeof(Value_Block)*BLOCK_SIZE, init_kernel.group_id(1));
    bos.push_back(value_bo);
    xrt::bo queue_bo=xrt::bo(device,sizeof(uint32_t)*QUEUE_SIZE,init_kernel.group_id(2));
    bos.push_back(queue_bo);
    xrt::bo page_tuple_count_bo=xrt::bo(device,PAGE_SIZE*sizeof(uint32_t),init_kernel.group_id(3));
    bos.push_back(page_tuple_count_bo);
    xrt::bo host_page_bo=xrt::bo(device,PAGE_SIZE*sizeof(Page),xrt::bo::flags::host_only,init_kernel.group_id(4));
    bos.push_back(host_page_bo);
    xrt::bo eviction_buffer_bo=xrt::bo(device,PAGE_CAPACITY*PAGE_SIZE*sizeof(Eviction_Buffer_Block),init_kernel.group_id(5));
    bos.push_back(eviction_buffer_bo);
    printf("Finish to generate bos\n");
    return bos;
}


std::vector<KV_TUPLE> generate_distributed_init_tuples(uint32_t size) {
    std::vector<KV_TUPLE> tuples;
    tuples.reserve(size); // Reserve space to avoid reallocation during insertion
    uint32_t seed = 16;
    std::mt19937 gen(seed);
    std::uniform_int_distribution<uint32_t> key_dis(0, 4294967295); // For generating random keys
    std::uniform_int_distribution<uint8_t> value_dis(0, 255); // For generating random values
    std::unordered_set<uint32_t> used_keys;

    while(tuples.size() < size) {    
        KV_TUPLE tuple;
        uint32_t key = key_dis(gen);
        if(used_keys.insert(key).second) {
            tuple.key = key;
            for(int b = 0; b < BLOCK_SIZE; b++) {
                for(int i = 0; i < 16; i++) {
                    tuple.blocks[b].value[i] = value_dis(gen);
                }
            }
            tuples.push_back(tuple);
        }
    }
    return tuples;
}


std::vector<Command_Packet> generate_accuracy_test_commands(std::vector<KV_TUPLE> init_tuples) {
    std::vector<Command_Packet> commands;
    for(int i = 0; i < init_tuples.size(); i++) {
        Command_Packet packet;
        packet.type = CMD_GET;
        packet.tuple = init_tuples[i];
        commands.push_back(packet);
    }
    for(int i = 0; i < init_tuples.size(); i++) {
        Command_Packet packet;
        packet.type = CMD_GET;
        packet.tuple = init_tuples[i];
        commands.push_back(packet);
    }
    return commands;
}



// ---- added by bolt_cmds_patch.py: seed of the command generator (BOLT_SEED, default 1 = BOLT's own) ----
static uint32_t bolt_seed() {
    const char* v = std::getenv("BOLT_SEED");
    return v ? (uint32_t)std::strtoul(v, nullptr, 10) : 1u;
}
// ---- end of addition ----

std::vector<Command_Packet> generate_uniform_commands(int size, std::vector<KV_TUPLE> init_tuples) {
    std::vector<Command_Packet> commands;
    uint32_t seed = bolt_seed();
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> op_dis(0.0, 1.0); // Decide read or write type
    std::uniform_int_distribution<int> key_dis(0, init_tuples.size() - 1); // Select from all keys
    std::uniform_int_distribution<uint8_t> value_dis(0, 255); // For generating random values
    
    double write_ratio = 0.5;
    get_cmd_count = 0;
    put_cmd_count = 0;
    
    for (int i = 0; i < size; i++) {
        Command_Packet packet;
        double read_write_op = op_dis(gen);
        int key_index = key_dis(gen);
        
        packet.tuple.key = init_tuples[key_index].key;
        record[i] = key_index; // Record which tuple this command refers to
        
        if (read_write_op < write_ratio) {
            packet.type = CMD_PUT;
            

            for(int i=0;i<BLOCK_SIZE;i++){
                for(int j=0;j<16;j++){
                    packet.tuple.blocks[i].value[j]=value_dis(gen);
                }
            }
            ++put_cmd_count;
            
            // Track if it's HBM or host memory
            if (hbm_key_to_index.find(packet.tuple.key) != hbm_key_to_index.end()) {
                hbm_cmd_count++;
            } else if (host_key_to_index.find(packet.tuple.key) != host_key_to_index.end()) {
                host_cmd_count++;
            }
        } else {
            packet.type = CMD_GET;
            
            ++get_cmd_count;
            
            // Track if it's HBM or host memory
            if (hbm_key_to_index.find(packet.tuple.key) != hbm_key_to_index.end()) {
                hbm_cmd_count++;
            } else if (host_key_to_index.find(packet.tuple.key) != host_key_to_index.end()) {
                host_cmd_count++;
            }
        }
        
        commands.push_back(packet);
    }
    
    std::cout << "Generated " << commands.size() << " uniform commands:" << std::endl;
    std::cout << "  GET commands: " << get_cmd_count << " (" << (100.0 * get_cmd_count / commands.size()) << "%)" << std::endl;
    std::cout << "  PUT commands: " << put_cmd_count << " (" << (100.0 * put_cmd_count / commands.size()) << "%)" << std::endl;
    std::cout << "  HBM access: " << hbm_cmd_count << " (" << (100.0 * hbm_cmd_count / commands.size()) << "%)" << std::endl;
    std::cout << "  Host access: " << host_cmd_count << " (" << (100.0 * host_cmd_count / commands.size()) << "%)" << std::endl;
    
    return commands;
}

void run_accuracy_test(xrt::device &device, xrt::kernel &init_kernel, xrt::kernel &chain_kernel, std::vector<KV_TUPLE> init_tuples, std::vector<xrt::bo> bos) {
    // Allocate memory dynamically
    Attribute (*position_map)[HBM_CAPACITY] = new Attribute[HASH_SIZE][HBM_CAPACITY];
    Value_Block (*value_blocks)[BLOCK_SIZE] = new Value_Block[INIT_SIZE][BLOCK_SIZE];
    uint8_t* bitmap = new uint8_t[INIT_SIZE];
    uint32_t* page_tuple_count = new uint32_t[PAGE_SIZE]();
    uint32_t* queue = new uint32_t[QUEUE_SIZE];
    Page* pages = new Page[PAGE_SIZE];
    Eviction_Buffer_Block (*eviction_buffer)[PAGE_CAPACITY] = new Eviction_Buffer_Block[PAGE_SIZE][PAGE_CAPACITY];
    
    auto init_start = std::chrono::system_clock::now();
    
    // Initialize data structures with empty values
    for(int i = 0; i < HASH_SIZE; i++) {
        for(int j = 0; j < HBM_CAPACITY; j++) {
            position_map[i][j].key = EMPTY_KEY;
            position_map[i][j].location_type = EMPTY_KEY;
        }
    }
    
    // Initialize eviction buffer
    for(int i = 0; i < PAGE_SIZE; i++) {
        for(int j = 0; j < PAGE_CAPACITY; j++) {
            eviction_buffer[i][j].key = EMPTY_KEY;
        }
    }

    // Initialize bitmap, pages, and eviction buffer
    memset(bitmap, 0, INIT_SIZE * sizeof(uint8_t));
    for(int i = 0; i < PAGE_SIZE; i++) {
        for(int j = 0; j < PAGE_CAPACITY; j++) {
            pages[i].tuples[j].key = EMPTY_KEY;
            eviction_buffer[i][j].key = EMPTY_KEY;
        }
    }

    // Set up random number generation
    uint32_t seed = 1;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> prob_dist(0.0, 1.0);
    std::uniform_int_distribution<> page_dist(0, PAGE_SIZE - 1);
    
    int free_index = 0;  // Points to next free index in HBM
    const int MAX_PAGE_RETRIES = 100;  // Maximum number of retries for finding a valid page

    // Process each tuple
    for(int i = 0; i < init_tuples.size(); i++) {
        const auto& tuple = init_tuples[i];
        double prob = prob_dist(gen);
        int hash_index = tuple.key % HASH_SIZE;
        bool inserted = false;

        if(prob < 0.2) {  // 20% chance for HBM
            // Try to insert into HBM
            for(int j = 0; j < HBM_CAPACITY; j++) {
                if(position_map[hash_index][j].key == EMPTY_KEY) {
                    // Insert into HBM
                    position_map[hash_index][j].key = tuple.key;
                    position_map[hash_index][j].location_type = 0;
                    
                    // Copy all value blocks
                    for(int b = 0; b < BLOCK_SIZE; b++) {
                        memcpy(&value_blocks[free_index][b], &tuple.blocks[b], sizeof(Value_Block));
                    }
                    
                    position_map[hash_index][j].location_index = free_index;
                    bitmap[free_index] = 1;
                    free_index++;
                    inserted = true;
                    hbm_sets.push_back(tuple.key);
                    hbm_key_to_index[tuple.key] = i;  // Store mapping
                    break;
                }
            }
        } else {  // 80% chance for page
            // Try to find a suitable page randomly
            int retry_count = 0;
            while (retry_count < MAX_PAGE_RETRIES) {
                int page_idx = page_dist(gen);
                if(page_tuple_count[page_idx] < 4) {
                    // Find empty slot in the page
                    for(int j = 0; j < PAGE_CAPACITY; j++) {
                        if(pages[page_idx].tuples[j].key == EMPTY_KEY) {
                            // Insert into page
                            pages[page_idx].tuples[j] = tuple;
                            page_tuple_count[page_idx]++;
                            
                            // Update position map
                            for(int k = 0; k < HBM_CAPACITY; k++) {
                                if(position_map[hash_index][k].key == EMPTY_KEY) {
                                    position_map[hash_index][k].key = tuple.key;
                                    position_map[hash_index][k].location_type = 1;
                                    position_map[hash_index][k].page_index = page_idx;
                                    position_map[hash_index][k].location_index = j;
                                    inserted = true;
                                    host_sets.push_back(tuple.key);
                                    host_key_to_index[tuple.key] = i;  // Store mapping
                                    
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    if(inserted) break;
                }
                retry_count++;
            }
        }

        if(!inserted) {
            insert_error_keys.insert(tuple.key);
        }
    }

    // Initialize queue with remaining free indices
    tail = 0;
    for(int i = 0; i < INIT_SIZE; i++) {
        if(bitmap[i] == 0) {
            queue[tail++] = i;
        }
    }

    std::cout << "HBM count is " << hbm_sets.size() << std::endl;
    std::cout << "Host count is " << host_sets.size() << std::endl;
    std::cout << "Insert error count is " << insert_error_keys.size() << std::endl;
    std::cout << "The hbm_key_to_index size is " << hbm_key_to_index.size() << std::endl;
    std::cout << "The host_key_to_index size is " << host_key_to_index.size() << std::endl;

    // Map and copy data to BOBs
    auto position_map_bo_map = bos[0].map<Attribute*>();
    std::memcpy(position_map_bo_map, position_map, HASH_SIZE*HBM_CAPACITY*sizeof(Attribute));
    bos[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto value_bo_map = bos[1].map<Value_Block*>();
    std::memcpy(value_bo_map, value_blocks, INIT_SIZE*BLOCK_SIZE*sizeof(Value_Block));
    bos[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto queue_bo_map = bos[2].map<uint32_t*>();
    std::memcpy(queue_bo_map, queue, QUEUE_SIZE*sizeof(uint32_t));
    bos[2].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto page_tuple_count_bo_map = bos[3].map<uint32_t*>();
    std::memcpy(page_tuple_count_bo_map, page_tuple_count, PAGE_SIZE*sizeof(uint32_t));
    bos[3].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto page_bo_map = bos[4].map<Page*>();
    std::memcpy(page_bo_map, pages, PAGE_SIZE*sizeof(Page));
    bos[4].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto eviction_buffer_bo_map = bos[5].map<Eviction_Buffer_Block*>();
    std::memcpy(eviction_buffer_bo_map, eviction_buffer, PAGE_CAPACITY*PAGE_SIZE*sizeof(Eviction_Buffer_Block));
    bos[5].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto init_end = std::chrono::system_clock::now();
    std::chrono::duration<double> init_time = init_end - init_start;
    std::cout << "The init time is " << init_time.count() << "s" << std::endl;
    
    // Generate Commands
    const int command_size = CMD_SIZE;
    std::vector<Command_Packet> commands = generate_accuracy_test_commands(init_tuples);
    std::vector<Response_Packet> responses;
    
    // Allocate and initialize buffers
    xrt::bo cmd_bo = xrt::bo(device, commands.size()*sizeof(Command_Packet), chain_kernel.group_id(6));
    auto cmd_bo_map = cmd_bo.map<Command_Packet*>();
    std::memcpy(cmd_bo_map, commands.data(), commands.size()*sizeof(Command_Packet));
    cmd_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    xrt::bo resp_bo = xrt::bo(device, commands.size()*sizeof(Response_Packet), chain_kernel.group_id(7));
    
    auto start = std::chrono::system_clock::now();
    auto chain_kernel_run = chain_kernel(bos[0], bos[1], bos[2], bos[3], bos[4], bos[5], cmd_bo, resp_bo, commands.size(), head, tail);
    chain_kernel_run.wait();
    auto end = std::chrono::system_clock::now();
    
    double result = 0;
    double kernel_time_in_sec = 0;
    std::chrono::duration<double> kernel_time(0);
    kernel_time = std::chrono::duration<double>(end - start);
    kernel_time_in_sec = kernel_time.count();
    
    std::cout << "Command size is " << commands.size() << std::endl;
    double avg_latency = kernel_time_in_sec/(commands.size());
    std::cout << "total time = " << kernel_time_in_sec << "s" << std::endl;
    std::cout << "AVERAGE LATENCY = " << avg_latency << "s" << std::endl;
    
    resp_bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    auto resp_bo_map = resp_bo.map<Response_Packet*>();
    int count = 0;
    int not_insert_count = 0;
    int mistake = 0;

    // Test accuracy for the first INIT_SIZE commands
    for(int i = 0; i < init_tuples.size(); i++) {
        if(resp_bo_map[i].type == RESP_ACK) {
            // Compare the response to the init_tuples
            KV_TUPLE tuple = init_tuples[i];
            if(commands[i].type == CMD_GET) {
                // For GET commands, check all value blocks
                bool values_match = true;
                for(int b = 0; b < BLOCK_SIZE; b++) {
                    if(std::memcmp(resp_bo_map[i].blocks[b].value, tuple.blocks[b].value, 16) != 0) {
                        values_match = false;
                        break;
                    }
                }
                
                if(values_match) {
                    count++;
                } else {
                    std::cout << "The response value " << i << " is not equal to the init_tuples value" << std::endl;
                    print_Tuple(tuple);
                    printf("The key is %d, first block's first byte in response is %d\n", 
                           tuple.key, resp_bo_map[i].blocks[0].value[0]);
                    mistake++;
                }
            } else {
                count++;
            }
        } else {
            // During initialization, it is not inserted into the hbm or host page because of the full capacity generated by seed
            if(insert_error_keys.find(init_tuples[record[i]].key) != insert_error_keys.end()) {
                not_insert_count++;
            }
        }
    }
    
    // Test accuracy for the second INIT_SIZE commands
    for(int i = 0; i < init_tuples.size(); i++) {
        if(resp_bo_map[i + INIT_SIZE].type == RESP_ACK) {
            // Compare the response to the init_tuples
            KV_TUPLE tuple = init_tuples[i];
            if(commands[i + INIT_SIZE].type == CMD_GET) {
                // For GET commands, check all value blocks
                Response_Packet resp = resp_bo_map[i + INIT_SIZE];
                bool values_match = true;
                for(int b = 0; b < BLOCK_SIZE; b++) {
                    if(std::memcmp(resp_bo_map[i].blocks[b].value, tuple.blocks[b].value, 16) != 0) {
                        values_match = false;
                        break;
                    }
                }   

                if(values_match) {
                    count++;
                } else {
                    std::cout << "The response value " << (i + INIT_SIZE) << " is not equal to the init_tuples value" << std::endl;
                    print_Tuple(tuple);
                    printf("The key is %d, first block's first byte in response is %d\n", 
                           tuple.key, resp_bo_map[i + INIT_SIZE].blocks[0].value[0]);
                    mistake++;
                }
            } else {
                count++;
            }
        } else {
            // During initialization, it is not inserted into the hbm or host page because of the full capacity generated by seed
            if(insert_error_keys.find(init_tuples[record[i + INIT_SIZE]].key) != insert_error_keys.end()) {
                not_insert_count++;
            }
        }
    }
    
    std::cout << "The number of response is " << count << std::endl;
    std::cout << "The number of mistake is " << mistake << std::endl;
    std::cout << "Error caused by not inserted into the hbm or host page (random seed) is " << not_insert_count << std::endl;
    std::cout << "The number of get command is " << get_cmd_count << std::endl;
    std::cout << "The number of put command is " << put_cmd_count << std::endl;
    
    // Clean up
    // delete[] position_map;
    // delete[] value_blocks;
    // delete[] bitmap;
    // delete[] page_tuple_count;
    // delete[] queue;
    // delete[] pages;
    // delete[] eviction_buffer;
}




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


// ---- added by bolt_cmds_patch.py: share of entries placed in HBM at initialization ----
static double bolt_hbm_frac() {
    static const double f = [] {
        const char* v = std::getenv("BOLT_HBM_FRAC");
        const double x = v ? std::strtod(v, nullptr) : 0.2;
        if (!(x >= 0.0 && x <= 1.0)) { std::cerr << "BOLT_HBM_FRAC must be 0..1" << std::endl; std::exit(2); }
        return x;
    }();
    return f;
}
// ---- end of addition ----

void run_init_and_chain_kernel_20_percent_hbm_with_probility(xrt::device &device, xrt::kernel &init_kernel, xrt::kernel &chain_kernel, std::vector<KV_TUPLE> init_tuples, std::vector<xrt::bo> bos) {
    // Allocate memory dynamically
    Attribute (*position_map)[HBM_CAPACITY] = new Attribute[HASH_SIZE][HBM_CAPACITY];
    Value_Block (*value_blocks)[BLOCK_SIZE] = new Value_Block[INIT_SIZE][BLOCK_SIZE];
    uint8_t* bitmap = new uint8_t[INIT_SIZE];
    uint32_t* page_tuple_count = new uint32_t[PAGE_SIZE]();
    uint32_t* queue = new uint32_t[QUEUE_SIZE];
    Page* pages = new Page[PAGE_SIZE];
    Eviction_Buffer_Block (*eviction_buffer)[PAGE_CAPACITY] = new Eviction_Buffer_Block[PAGE_SIZE][PAGE_CAPACITY];
    
    auto init_start = std::chrono::system_clock::now();
    
    // Initialize data structures with empty values
    for(int i = 0; i < HASH_SIZE; i++) {
        for(int j = 0; j < HBM_CAPACITY; j++) {
            position_map[i][j].key = EMPTY_KEY;
            position_map[i][j].location_type = EMPTY_KEY;
        }
    }
    
    // Initialize eviction buffer
    for(int i = 0; i < PAGE_SIZE; i++) {
        for(int j = 0; j < PAGE_CAPACITY; j++) {
            eviction_buffer[i][j].key = EMPTY_KEY;
        }
    }

    // Initialize bitmap, pages, and eviction buffer
    memset(bitmap, 0, INIT_SIZE * sizeof(uint8_t));
    for(int i = 0; i < PAGE_SIZE; i++) {
        for(int j = 0; j < PAGE_CAPACITY; j++) {
            pages[i].tuples[j].key = EMPTY_KEY;
            eviction_buffer[i][j].key = EMPTY_KEY;
        }
    }

    // Set up random number generation
    uint32_t seed = 1;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> prob_dist(0.0, 1.0);
    std::uniform_int_distribution<> page_dist(0, PAGE_SIZE - 1);
    
    int free_index = 0;  // Points to next free index in HBM
    const int MAX_PAGE_RETRIES = 100;  // Maximum number of retries for finding a valid page

    // Process each tuple
    for(int i = 0; i < init_tuples.size(); i++) {
        const auto& tuple = init_tuples[i];
        double prob = prob_dist(gen);
        int hash_index = tuple.key % HASH_SIZE;
        bool inserted = false;

        if(prob < bolt_hbm_frac()) {  // BOLT_HBM_FRAC: HBM
            // Try to insert into HBM
            for(int j = 0; j < HBM_CAPACITY; j++) {
                if(position_map[hash_index][j].key == EMPTY_KEY) {
                    // Insert into HBM
                    position_map[hash_index][j].key = tuple.key;
                    position_map[hash_index][j].location_type = 0;
                    
                    // Copy all value blocks
                    for(int b = 0; b < BLOCK_SIZE; b++) {
                        memcpy(&value_blocks[free_index][b], &tuple.blocks[b], sizeof(Value_Block));
                    }
                    
                    position_map[hash_index][j].location_index = free_index;
                    bitmap[free_index] = 1;
                    free_index++;
                    inserted = true;
                    hbm_sets.push_back(tuple.key);
                    hbm_key_to_index[tuple.key] = i;  // Store mapping
                    break;
                }
            }
        } else {  // the rest: host pages
            // Try to find a suitable page randomly
            int retry_count = 0;
            while (retry_count < MAX_PAGE_RETRIES) {
                int page_idx = page_dist(gen);
                if(page_tuple_count[page_idx] < 4) {
                    // Find empty slot in the page
                    for(int j = 0; j < PAGE_CAPACITY; j++) {
                        if(pages[page_idx].tuples[j].key == EMPTY_KEY) {
                            // Insert into page
                            pages[page_idx].tuples[j] = tuple;
                            page_tuple_count[page_idx]++;
                            
                            // Update position map
                            for(int k = 0; k < HBM_CAPACITY; k++) {
                                if(position_map[hash_index][k].key == EMPTY_KEY) {
                                    position_map[hash_index][k].key = tuple.key;
                                    position_map[hash_index][k].location_type = 1;
                                    position_map[hash_index][k].page_index = page_idx;
                                    position_map[hash_index][k].location_index = j;
                                    inserted = true;
                                    host_sets.push_back(tuple.key);
                                    host_key_to_index[tuple.key] = i;  // Store mapping
                    
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    if(inserted) break;
                }
                retry_count++;
            }
        }

        if(!inserted) {
            insert_error_keys.insert(tuple.key);
        }
    }

    // Initialize queue with remaining free indices
    tail = 0;
    for(int i = 0; i < INIT_SIZE; i++) {
        if(bitmap[i] == 0) {
            queue[tail++] = i;
        }
    }

    std::cout << "HBM count is " << hbm_sets.size() << std::endl;
    std::cout << "Host count is " << host_sets.size() << std::endl;
    std::cout << "Insert error count is " << insert_error_keys.size() << std::endl;
    std::cout << "The hbm_key_to_index size is " << hbm_key_to_index.size() << std::endl;
    std::cout << "The host_key_to_index size is " << host_key_to_index.size() << std::endl;

    // Map and copy data to BOBs
    auto position_map_bo_map = bos[0].map<Attribute*>();
    std::memcpy(position_map_bo_map, position_map, HASH_SIZE*HBM_CAPACITY*sizeof(Attribute));
    bos[0].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto value_bo_map = bos[1].map<Value_Block*>();
    std::memcpy(value_bo_map, value_blocks, INIT_SIZE*BLOCK_SIZE*sizeof(Value_Block));
    bos[1].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto queue_bo_map = bos[2].map<uint32_t*>();
    std::memcpy(queue_bo_map, queue, QUEUE_SIZE*sizeof(uint32_t));
    bos[2].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto page_tuple_count_bo_map = bos[3].map<uint32_t*>();
    std::memcpy(page_tuple_count_bo_map, page_tuple_count, PAGE_SIZE*sizeof(uint32_t));
    bos[3].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto page_bo_map = bos[4].map<Page*>();
    std::memcpy(page_bo_map, pages, PAGE_SIZE*sizeof(Page));
    bos[4].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto eviction_buffer_bo_map = bos[5].map<Eviction_Buffer_Block*>();
    std::memcpy(eviction_buffer_bo_map, eviction_buffer, PAGE_CAPACITY*PAGE_SIZE*sizeof(Eviction_Buffer_Block));
    bos[5].sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    auto init_end = std::chrono::system_clock::now();
    std::chrono::duration<double> init_time = init_end - init_start;
    std::cout << "The init time is " << init_time.count() << "s" << std::endl;
    
    // Generate Commands
    const int command_size = bolt_cmds();
    std::vector<Command_Packet> commands = generate_uniform_commands(command_size,init_tuples);
    std::vector<Response_Packet> responses;
    
    // Allocate and initialize buffers
    xrt::bo cmd_bo = xrt::bo(device, commands.size()*sizeof(Command_Packet), chain_kernel.group_id(6));
    auto cmd_bo_map = cmd_bo.map<Command_Packet*>();
    std::memcpy(cmd_bo_map, commands.data(), commands.size()*sizeof(Command_Packet));
    cmd_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    
    xrt::bo resp_bo = xrt::bo(device, commands.size()*sizeof(Response_Packet), chain_kernel.group_id(7));
    
    auto start = std::chrono::system_clock::now();
    auto chain_kernel_run = chain_kernel(bos[0], bos[1], bos[2], bos[3], bos[4], bos[5], cmd_bo, resp_bo, commands.size(), head, tail);
    chain_kernel_run.wait();
    auto end = std::chrono::system_clock::now();
    
    double result = 0;
    double kernel_time_in_sec = 0;
    std::chrono::duration<double> kernel_time(0);
    kernel_time = std::chrono::duration<double>(end - start);
    kernel_time_in_sec = kernel_time.count();
    
    std::cout << "Command size is " << commands.size() << std::endl;
    double avg_latency = kernel_time_in_sec/(commands.size());
    std::cout << "total time = " << kernel_time_in_sec << "s" << std::endl;
    std::cout << "AVERAGE LATENCY = " << avg_latency << "s" << std::endl;
    
    resp_bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    {   // ---- added by bolt_cmds_patch.py: read back after the timed run, so the timing is unaffected ----
        const size_t ncmd = commands.size();
        std::cout << "SEED command_seed=" << bolt_seed() << std::endl;
        std::cout << "HBM_FRAC start_share=" << bolt_hbm_frac() << " hbm_keys_at_init=" << hbm_sets.size()
                  << " free_hbm_slots=" << ((size_t)INIT_SIZE - hbm_sets.size()) << std::endl;
        if ((size_t)INIT_SIZE - hbm_sets.size() < ncmd)
            std::cout << "WARNING fewer free HBM slots than commands: BOLT never ran this case" << std::endl;
        std::cout << "TIMED commands=" << ncmd << " seconds=" << kernel_time_in_sec
                  << " us_per_command=" << kernel_time_in_sec / ncmd * 1e6
                  << " kqps=" << ncmd / kernel_time_in_sec / 1e3 << std::endl;
        const Response_Packet* resp = resp_bo.map<Response_Packet*>();
        size_t acked = 0; long long last_acked = -1;
        for (size_t i = 0; i < ncmd; i++) if (resp[i].type == RESP_ACK) { acked++; last_acked = (long long)i; }
        std::cout << "RESP acked=" << acked << " of " << ncmd << " last_acked_index=" << last_acked << std::endl;

        std::unordered_map<uint32_t, const unsigned char*> latest;
        latest.reserve(2 * init_tuples.size());
        for (const auto& t : init_tuples) latest[t.key] = reinterpret_cast<const unsigned char*>(&t.blocks);
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
                latest[k] = reinterpret_cast<const unsigned char*>(&commands[i].tuple.blocks);
                h.push_back(latest[k]);
                put_keys.insert(k);
                last_put[k] = i; last_acc[k] = i;
                continue;
            }
            gets++;
            if (put_keys.count(k)) raw++;
            if (resp[i].type != RESP_ACK || std::memcmp(&resp[i].blocks, latest[k], VALUE_SIZE) != 0) {
                bad++;
                if (insert_error_keys.count(k)) bad_unplaced++;
                if (first_bad < 0) first_bad = (long long)i;
                per_win[i / 100000]++;
                const char* cls = "other"; long back = -1;
                const unsigned char* r = reinterpret_cast<const unsigned char*>(&resp[i].blocks);
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

    // Clean up
    // delete[] position_map;
    // delete[] value_blocks;
    // delete[] bitmap;
    // delete[] page_tuple_count;
    // delete[] queue;
    // delete[] pages;
    // delete[] eviction_buffer;
}


int main(int argc, char* argv[]) {
    // system("sync && echo 3 | sudo tee /proc/sys/vm/drop_caches");
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <xclbin>" << std::endl;
        return 1;
    }
    
    std::string xclbin_file(argv[1]);
    xrt::device device = xrt::device(0);
    xrt::uuid uuid = xrt::uuid(device.load_xclbin(xclbin_file));
    
    xrt::kernel init_kernel = xrt::kernel(device, uuid, "init_kernel");
    xrt::kernel chain_kernel = xrt::kernel(device, uuid, "chain_kernel");
    printf("Start testing 20% HBM\n");
    std::vector<KV_TUPLE> init_tuples = generate_distributed_init_tuples(INIT_SIZE);
    printf("Init tuples size is %ld\n",init_tuples.size());
    printf("The value size is %ld\n",VALUE_SIZE);
    printf("The block size is %ld\n",BLOCK_SIZE);
    std::vector<xrt::bo> bos = generate_bos_host_key_value_seperate_complete(device, init_kernel);
    // run_accuracy_test(device, init_kernel, chain_kernel, init_tuples, bos);
    run_init_and_chain_kernel_20_percent_hbm_with_probility(device, init_kernel, chain_kernel, init_tuples, bos);
    
    return 0;
}