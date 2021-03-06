#include "hmc.h"

namespace dramsim3 {

HMCRequest::HMCRequest(HMCReqType req_type, uint64_t hex_addr1, int vault, uint64_t hex_addr2, uint64_t hex_addr3)
    : type(req_type), mem_operand1(hex_addr1), mem_operand2(hex_addr2), mem_operand3(hex_addr3), vault(vault) {
    is_write = type >= HMCReqType::WR0 && type <= HMCReqType::P_WR256;
    is_read = type >= HMCReqType::RD0 && type <= HMCReqType::RD256;

    
    // given that vaults could be 16 (Gen1) or 32(Gen2), using % 4
    // to partition vaults to quads
   
    quad = vault % 4;
    switch (req_type) {
        case HMCReqType::RD0:
        case HMCReqType::WR0:
            flits = 0;
            break;
        case HMCReqType::RD16:
        case HMCReqType::RD32:
        case HMCReqType::RD48:
        case HMCReqType::RD64:
        case HMCReqType::RD80:
        case HMCReqType::RD96:
        case HMCReqType::RD112:
        case HMCReqType::RD128:
        case HMCReqType::RD256:
            flits = 1;
            break;
        case HMCReqType::WR16:
        case HMCReqType::P_WR16:
            flits = 2;
            break;
        case HMCReqType::WR32:
        case HMCReqType::P_WR32:
            flits = 3;
            break;
        case HMCReqType::WR48:
        case HMCReqType::P_WR48:
            flits = 4;
            break;
        case HMCReqType::WR64:
        case HMCReqType::P_WR64:
            flits = 5;
            break;
        case HMCReqType::WR80:
        case HMCReqType::P_WR80:
            flits = 6;
            break;
        case HMCReqType::WR96:
        case HMCReqType::P_WR96:
            flits = 7;
            break;
        case HMCReqType::WR112:
        case HMCReqType::P_WR112:
            flits = 8;
            break;
        case HMCReqType::WR128:
        case HMCReqType::P_WR128:
            flits = 9;
            break;
        case HMCReqType::WR256:
        case HMCReqType::P_WR256:
            flits = 17;
            break;
        case HMCReqType::ADD8:
        case HMCReqType::ADD16:
            flits = 2;
            break;
        case HMCReqType::P_2ADD8:
        case HMCReqType::P_ADD16:
            flits = 2;
            break;
        case HMCReqType::ADDS8R:
        case HMCReqType::ADDS16R:
            flits = 2;
            break;
        case HMCReqType::INC8:
            flits = 1;
            break;
        case HMCReqType::P_INC8:
            flits = 1;
            break;
        case HMCReqType::XOR16:
        case HMCReqType::OR16:
        case HMCReqType::NOR16:
        case HMCReqType::AND16:
        case HMCReqType::NAND16:
        case HMCReqType::CASGT8:
        case HMCReqType::CASGT16:
        case HMCReqType::CASLT8:
        case HMCReqType::CASLT16:
        case HMCReqType::CASEQ8:
        case HMCReqType::CASZERO16:
            flits = 2;
            break;
        case HMCReqType::EQ8:
        case HMCReqType::EQ16:
        case HMCReqType::BWR:
            flits = 2;
            break;
        case HMCReqType::P_BWR:
            flits = 2;
            break;
        case HMCReqType::BWR8R:
        case HMCReqType::SWAP16:
            flits = 2;
            break;
        case HMCReqType::CIM_FETCH:
        case HMCReqType::CIM_STORE:
        case HMCReqType::CIM_ADD:
        case HMCReqType::CIM_SWAP:
        case HMCReqType::CIM_XOR:
            flits = 2;
            break;
        default:
            AbruptExit(__FILE__, __LINE__);
            break;
    }
}

HMCResponse::HMCResponse(uint64_t id, HMCReqType req_type, int dest_link,
                         int src_quad)
    : resp_id(id), link(dest_link), quad(src_quad) {
    switch (req_type) {
        case HMCReqType::RD0:
            type = HMCRespType::RD_RS;
            flits = 0;
            break;
        case HMCReqType::RD16:
            type = HMCRespType::RD_RS;
            flits = 2;
            break;
        case HMCReqType::RD32:
            type = HMCRespType::RD_RS;
            flits = 3;
            break;
        case HMCReqType::RD48:
            type = HMCRespType::RD_RS;
            flits = 4;
            break;
        case HMCReqType::RD64:
            type = HMCRespType::RD_RS;
            flits = 5;
            break;
        case HMCReqType::RD80:
            type = HMCRespType::RD_RS;
            flits = 6;
            break;
        case HMCReqType::RD96:
            type = HMCRespType::RD_RS;
            flits = 7;
            break;
        case HMCReqType::RD112:
            type = HMCRespType::RD_RS;
            flits = 8;
            break;
        case HMCReqType::RD128:
            type = HMCRespType::RD_RS;
            flits = 9;
            break;
        case HMCReqType::RD256:
            type = HMCRespType::RD_RS;
            flits = 17;
            break;
        case HMCReqType::WR0:
            flits = 0;
            type = HMCRespType::WR_RS;
            break;
        case HMCReqType::WR16:
        case HMCReqType::WR32:
        case HMCReqType::WR48:
        case HMCReqType::WR64:
        case HMCReqType::WR80:
        case HMCReqType::WR96:
        case HMCReqType::WR112:
        case HMCReqType::WR128:
        case HMCReqType::WR256:
            type = HMCRespType::WR_RS;
            flits = 1;
            break;
        case HMCReqType::P_WR16:
        case HMCReqType::P_WR32:
        case HMCReqType::P_WR48:
        case HMCReqType::P_WR64:
        case HMCReqType::P_WR80:
        case HMCReqType::P_WR96:
        case HMCReqType::P_WR112:
        case HMCReqType::P_WR128:
        case HMCReqType::P_WR256:
            type = HMCRespType::NONE;
            flits = 0;
            break;
        case HMCReqType::ADD8:
        case HMCReqType::ADD16:
            type = HMCRespType::WR_RS;
            flits = 1;
            break;
        case HMCReqType::P_2ADD8:
        case HMCReqType::P_ADD16:
            type = HMCRespType::NONE;
            flits = 0;
            break;
        case HMCReqType::ADDS8R:
        case HMCReqType::ADDS16R:
            type = HMCRespType::RD_RS;
            flits = 2;
            break;
        case HMCReqType::INC8:
            type = HMCRespType::WR_RS;
            flits = 1;
            break;
        case HMCReqType::P_INC8:
            type = HMCRespType::NONE;
            flits = 0;
            break;
        case HMCReqType::XOR16:
        case HMCReqType::OR16:
        case HMCReqType::NOR16:
        case HMCReqType::AND16:
        case HMCReqType::NAND16:
        case HMCReqType::CASGT8:
        case HMCReqType::CASGT16:
        case HMCReqType::CASLT8:
        case HMCReqType::CASLT16:
        case HMCReqType::CASEQ8:
        case HMCReqType::CASZERO16:
            type = HMCRespType::RD_RS;
            flits = 2;
            break;
        case HMCReqType::EQ8:
        case HMCReqType::EQ16:
        case HMCReqType::BWR:
            type = HMCRespType::WR_RS;
            flits = 1;
            break;
        case HMCReqType::P_BWR:
            type = HMCRespType::NONE;
            flits = 0;
            break;
        case HMCReqType::BWR8R:
        case HMCReqType::SWAP16:
            type = HMCRespType::RD_RS;
            flits = 2;
            break;
        case HMCReqType::CIM_FETCH:
            type = HMCRespType :: CIM_FETCH_RS;
            flits = 0;
            break;
        case HMCReqType::CIM_STORE:
            type = HMCRespType::CIM_STORE_RS;
            flits = 0;
            break;
        case HMCReqType::CIM_ADD:
            type = HMCRespType::CIM_ADD_RS;
            flits = 0;
            break;
        case HMCReqType::CIM_SWAP:
            type = HMCRespType::CIM_SWAP_RS;
            flits = 0;
            break;
        case HMCReqType::CIM_XOR:
            type = HMCRespType::CIM_XOR_RS;
            flits = 0;
            break;
        default:
            AbruptExit(__FILE__, __LINE__);
            break;
    }
    return;
}

HMCMemorySystem::HMCMemorySystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback),
      logic_clk_(0),
      logic_ps_(0),
      dram_ps_(0),
      next_link_(0),
      req_id_(1000){
    // sanity check, this constructor should only be intialized using HMC
    if (!config_.IsHMC()) {
        std::cerr << "Initialzed an HMC system without an HMC config file!"
                  << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }

    // setting up clock
    SetClockRatio();

    ctrls_.reserve(config_.channels);
    for (int i = 0; i < config_.channels; i++) {
#ifdef THERMAL
        ctrls_.push_back(new Controller(i, config_, timing_, thermal_calc_));
#else
        ctrls_.push_back(new Controller(i, config_, timing_));
#endif  // THERMAL
    }
    // initialize vaults and crossbar
    // the first layer of xbar will be num_links * 4 (4 for quadrants)
    // the second layer will be a 1:8 xbar
    // (each quadrant has 8 vaults and each quadrant can access any ohter
    // quadrant)
    queue_depth_ = static_cast<size_t>(config_.xbar_queue_depth);
    links_ = config_.num_links;
    link_req_queues_.reserve(links_);
    link_resp_queues_.reserve(links_);
    for (int i = 0; i < links_; i++) {
        link_req_queues_.push_back(std::vector<HMCRequest *>());
        link_resp_queues_.push_back(std::vector<HMCResponse *>());
    }

    // don't want to hard coding it but there are 4 quads so it's kind of fixed
    quad_req_queues_.reserve(4);
    quad_resp_queues_.reserve(4);
    for (int i = 0; i < 4; i++) {
        quad_req_queues_.push_back(std::vector<HMCRequest *>());
        quad_resp_queues_.push_back(std::vector<HMCResponse *>());
    }

    link_busy_.reserve(links_);
    link_age_counter_.reserve(links_);
    for (int i = 0; i < links_; i++) {
        link_busy_.push_back(0);
        link_age_counter_.push_back(0);
    }
}

HMCMemorySystem::~HMCMemorySystem() {
    for (auto &&vault_ptr : ctrls_) {
        delete (vault_ptr);
    }
}

void HMCMemorySystem::SetClockRatio() {
    // There are 3 clock domains here, Link (super fast), logic (fast), DRAM
    // (slow) We assume the logic process 1 flit per logic cycle and since the
    // link takes several cycles to process 1 flit (128b), we can deduce logic
    // speed according to link speed
    ps_per_dram_ = 800;  // 800 ps
    int link_cycles_per_flit = 128 / config_.link_width;
    int logic_speed = config_.link_speed / link_cycles_per_flit;  // MHz
    ps_per_logic_ =
        static_cast<uint64_t>(1000000 / static_cast<double>(logic_speed));
    if (ps_per_logic_ > ps_per_dram_) {
        ps_per_logic_ = ps_per_dram_;
    }
    return;
}

inline void HMCMemorySystem::IterateNextLink() {
    // determinining which link a request goes to has great impact on
    // performance round robin , we can implement other schemes here later such
    // as random but there're only at most 4 links so I suspect it would make a
    // difference
    next_link_ = (next_link_ + 1) % links_;
    return;
}

bool HMCMemorySystem::WillAcceptTransaction(uint64_t hex_addr,
                                            bool is_write) const {
    bool insertable = false;
    for (auto link_queue = link_req_queues_.begin();
         link_queue != link_req_queues_.end(); link_queue++) {
        if ((*link_queue).size() < queue_depth_) {
            insertable = true;
            break;
        }
    }
    return insertable;
}

bool HMCMemorySystem::AddTransaction(uint64_t hex_addr, bool is_write) {
    // to be compatible with other protocol we have this interface
    // when using this intreface the size of each transaction will be block_size
    HMCReqType req_type;
    if (is_write) {
        switch (config_.block_size) {
            case 0:
                req_type = HMCReqType::WR0;
                break;
            case 32:
                req_type = HMCReqType::WR32;
                break;
            case 64:
                req_type = HMCReqType::WR64;
                break;
            case 128:
                req_type = HMCReqType::WR128;
                break;
            case 256:
                req_type = HMCReqType::WR256;
                break;
            default:
                req_type = HMCReqType::SIZE;
                AbruptExit(__FILE__, __LINE__);
                break;
        }
    } else {
        switch (config_.block_size) {
            case 0:
                req_type = HMCReqType::RD0;
                break;
            case 32:
                req_type = HMCReqType::RD32;
                break;
            case 64:
                req_type = HMCReqType::RD64;
                break;
            case 128:
                req_type = HMCReqType::RD128;
                break;
            case 256:
                req_type = HMCReqType::RD256;
                break;
            default:
                req_type = HMCReqType::SIZE;
                AbruptExit(__FILE__, __LINE__);
                break;
        }
    }
    int vault = GetChannel(hex_addr);
    HMCRequest *req = new HMCRequest(req_type, hex_addr, vault);
    return InsertHMCReq(req);
}

/*Overloading for CIM*/
bool HMCMemorySystem::WillAcceptTransaction(Transaction& trans) const {
    /* */
    /* CIM for certain commands we need two or more request*/
    int no_of_requests = 1;
    if (trans.is_cim_add || trans.is_cim_xor) {
        no_of_requests = 3; //2 CIM_FETCH and 1 CIM_STORE
    }
    if (trans.is_cim_swap)
    {
        no_of_requests = 4; //2 CIM_FETCH and 2 CIM_STORE
    }

    /* */
    bool insertable = false;
    for (auto link_queue = link_req_queues_.begin();
        link_queue != link_req_queues_.end(); link_queue++) {
        if ((*link_queue).size()+ no_of_requests < queue_depth_) {
            insertable = true;
            break;
        }
    }

    return insertable;
}

/*Overloading for CIM*/
bool HMCMemorySystem::AddTransaction(Transaction& trans) {
    // to be compatible with other protocol we have this interface
    // when using this intreface the size of each transaction will be block_size
    HMCReqType req_type;
    if (trans.is_write) {
        switch (config_.block_size) {
        case 0:
            req_type = HMCReqType::WR0;
            break;
        case 32:
            req_type = HMCReqType::WR32;
            break;
        case 64:
            req_type = HMCReqType::WR64;
            break;
        case 128:
            req_type = HMCReqType::WR128;
            break;
        case 256:
            req_type = HMCReqType::WR256;
            break;
        default:
            req_type = HMCReqType::SIZE;
            AbruptExit(__FILE__, __LINE__);
            break;
        }
    }

    else if(trans.is_read) {
        switch (config_.block_size) {
        case 0:
            req_type = HMCReqType::RD0;
            break;
        case 32:
            req_type = HMCReqType::RD32;
            break;
        case 64:
            req_type = HMCReqType::RD64;
            break;
        case 128:
            req_type = HMCReqType::RD128;
            break;
        case 256:
            req_type = HMCReqType::RD256;
            break;
        default:
            req_type = HMCReqType::SIZE;
            AbruptExit(__FILE__, __LINE__);
            break;
        }
    }
    else {
        if (trans.is_cim_fetch)
            req_type = HMCReqType::CIM_FETCH;
        if (trans.is_cim_store)
            req_type = HMCReqType::CIM_STORE;
        if (trans.is_cim_add)
            req_type = HMCReqType::CIM_ADD;
        if (trans.is_cim_swap)
            req_type = HMCReqType::CIM_SWAP;
        if (trans.is_cim_xor)
            req_type = HMCReqType::CIM_XOR;
    }
    if (trans.is_write || trans.is_read || trans.is_cim_fetch || trans.is_cim_store) {
        int vault = GetChannel(trans.addr);
        HMCRequest* req = new HMCRequest(req_type, trans.addr, vault);
        return InsertHMCReq(req);
    }
    else if(trans.is_cim_add || trans.is_cim_xor){
        int vault = GetChannel(trans.addr);
        /* Sanity check. Both the mem operands must be in the same vault*/
        /*if (vault != GetChannel(trans.addr2)) {
            AbruptExit(__FILE__, __LINE__);
        }*/
        HMCRequest* req = new HMCRequest(req_type, trans.addr, vault, trans.addr2, trans.addr3);
        return InsertHMCReq(req);
    }
    else if (trans.is_cim_swap) {
        int vault = GetChannel(trans.addr);
        HMCRequest* req = new HMCRequest(req_type, trans.addr, vault, trans.addr2, trans.addr3);
        return InsertHMCReq(req);
    }
}

/* ****** */

bool HMCMemorySystem::InsertReqToLink(HMCRequest *req, int link) {
    // These things need to happen when an HMC request is inserted to a link:
    // 1. check if link queue full
    // 2. set link field in the request packet
    // 3. create corresponding response
    // 4. increment link_age_counter_ so that arbitrate logic works
    if (link_req_queues_[link].size() < queue_depth_) {
        req->link = link;
        link_req_queues_[link].push_back(req);
            HMCResponse* resp =
                new HMCResponse(req->mem_operand1, req->type, link, req->quad);
            resp_lookup_table_.insert(
                std::pair<uint64_t, HMCResponse*>(resp->resp_id, resp));


            link_age_counter_[link] = 1;
            // stats_.interarrival_latency.AddValue(clk_ - last_req_clk_);
            last_req_clk_ = clk_;
        return true;
    } else {
        return false;
    }
}

bool HMCMemorySystem::InsertHMCReq(HMCRequest *req) {
    // most CPU models does not support simultaneous insertions
    // if you want to actually simulate the multi-link feature
    // then you have to call this function multiple times in 1 cycle
    // TODO put a cap limit on how many times you can call this function per
    // cycle
    bool is_inserted = InsertReqToLink(req, next_link_);
    if (!is_inserted) {
        int start_link = next_link_;
        IterateNextLink();
        while (start_link != next_link_) {
            if (InsertReqToLink(req, next_link_)) {
                IterateNextLink();
                return true;
            } else {
                IterateNextLink();
            }
        }
        return false;
    } else {
        IterateNextLink();
        return true;
    }
}

void HMCMemorySystem::DrainRequests() {
    // drain quad request queue to vaults
    for (int i = 0; i < 4; i++) {
        if (!quad_req_queues_[i].empty() &&
            quad_resp_queues_[i].size() < queue_depth_) {
            HMCRequest *req = quad_req_queues_[i].front();
            if (req->exit_time <= logic_clk_) {
                /*CIM*/
                //Check how many read and write transactions a given request requires
                int no_of_reads = 0, no_of_writes = 0;
                if (req->is_read || req->type == HMCReqType::CIM_FETCH)
                    no_of_reads = 1;
                if (req->is_write || req->type == HMCReqType::CIM_STORE)
                    no_of_writes = 1;
                if (req->type == HMCReqType::CIM_ADD) {
                    no_of_reads = 2;
                    no_of_writes = 1;
                }
                if (req->type == HMCReqType::CIM_SWAP) {
                    no_of_reads = 2;
                    no_of_writes = 2;
                }
                if (req->type == HMCReqType::CIM_XOR) {
                    no_of_reads = 2;
                    no_of_writes = 1;
                }
                if (ctrls_[req->vault]->WillAcceptTransaction(req->mem_operand1, no_of_reads, no_of_writes)) {
                    InsertReqToDRAM(req);
                    delete (req);
                    quad_req_queues_[i].erase(quad_req_queues_[i].begin());
                }
            }
        }
    }

    // drain xbar
    for (auto &&i : quad_busy_) {
        if (i > 0) {
            i -= 2;
        }
    }

    // drain requests from link to quad buffers
    std::vector<int> age_queue = BuildAgeQueue(link_age_counter_);
    while (!age_queue.empty()) {
        int src_link = age_queue.front();
        int dest_quad = link_req_queues_[src_link].front()->quad;
        if (quad_req_queues_[dest_quad].size() < queue_depth_ &&
            quad_busy_[dest_quad] <= 0) {
            HMCRequest *req = link_req_queues_[src_link].front();
            link_req_queues_[src_link].erase(
                link_req_queues_[src_link].begin());
            quad_req_queues_[dest_quad].push_back(req);
            quad_busy_[dest_quad] = req->flits;
            req->exit_time = logic_clk_ + req->flits;
            if (link_req_queues_[src_link].empty()) {
                link_age_counter_[src_link] = 0;
            } else {
                link_age_counter_[src_link] = 1;
            }
        } else {  // stalled this cycle, update age counter
            link_age_counter_[src_link]++;
        }
        age_queue.erase(age_queue.begin());
    }
    age_queue.clear();
}

void HMCMemorySystem::DrainResponses() {
    // Link resp to CPU
    for (int i = 0; i < links_; i++) {
        if (!link_resp_queues_[i].empty()) {
            HMCResponse *resp = link_resp_queues_[i].front();


                if (resp->exit_time <= logic_clk_) {
                    if (resp->type == HMCRespType::RD_RS) {
                        read_callback_(resp->resp_id);
                    }
                    else {
                        write_callback_(resp->resp_id);
                    }
                }
                delete (resp);
                link_resp_queues_[i].erase(link_resp_queues_[i].begin());
            
        }
    }

    // drain xbar
    for (auto &&i : link_busy_) {
        if (i > 0) {
            i -= 2;
        }
    }

    // drain responses from quad to link buffers
    auto age_queue = BuildAgeQueue(quad_age_counter_);
    while (!age_queue.empty()) {
        int src_quad = age_queue.front();
        int dest_link = quad_resp_queues_[src_quad].front()->link;
        if (link_resp_queues_[dest_link].size() < queue_depth_ &&
            link_busy_[dest_link] <= 0) {
            HMCResponse *resp = quad_resp_queues_[src_quad].front();
            quad_resp_queues_[src_quad].erase(
                quad_resp_queues_[src_quad].begin());
            link_resp_queues_[dest_link].push_back(resp);
            link_busy_[dest_link] = resp->flits;
            resp->exit_time = logic_clk_ + resp->flits;
            if (quad_resp_queues_[src_quad].size() == 0) {
                quad_age_counter_[src_quad] = 0;
            } else {
                quad_age_counter_[src_quad] = 1;
            }
        } else {  // stalled this cycle, update age counter
            quad_age_counter_[src_quad]++;
        }
        age_queue.erase(age_queue.begin());
    }
    age_queue.clear();
}

void HMCMemorySystem::DRAMClockTick() {

    for (size_t i = 0; i < ctrls_.size(); i++) {
        // look ahead and return earlier
        while (true) {
            auto pair = ctrls_[i]->ReturnDoneTrans(clk_);
            if (pair.second == 1) {  // write
                VaultCallback(pair.first);
            } else if (pair.second == 0) {  // read
                VaultCallback(pair.first);
            }
            else if (pair.second == CIM) {
                VaultCallback(pair.first);
            }
            else {
                //use a map to see if all the transactions have been completed for the given request id;
                //update the stats file 
                //std::cout << "Neither read nor write\n";
                break;
            }
        }
    }
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ClockTick();
    }
    clk_++;

    if (clk_ % config_.epoch_period == 0) {
        PrintEpochStats();
    }
    return;
}

void HMCMemorySystem::ClockTick() {
    if (dram_ps_ == logic_ps_) {
        DrainResponses();
        DRAMClockTick();
        DrainRequests();
        logic_ps_ += ps_per_logic_;
        logic_clk_ += 1;
    } else {
        DRAMClockTick();
    }
    while (logic_ps_ < dram_ps_ + ps_per_dram_) {
        DrainResponses();
        DrainRequests();
        logic_ps_ += ps_per_logic_;
        logic_clk_ += 1;
    }
    dram_ps_ += ps_per_dram_;
    return;
}

std::vector<int> HMCMemorySystem::BuildAgeQueue(std::vector<int> &age_counter) {
    // return a vector of indices sorted in decending order
    // meaning that the oldest age link/quad should be processed first
    std::vector<int> age_queue;
    int queue_len = age_counter.size();
    age_queue.reserve(queue_len);
    int start_pos = logic_clk_ % queue_len;  // round robin start pos
    for (int i = 0; i < queue_len; i++) {
        int pos = (i + start_pos) % queue_len;
        if (age_counter[pos] > 0) {
            bool is_inserted = false;
            for (auto it = age_queue.begin(); it != age_queue.end(); it++) {
                if (age_counter[pos] > *it) {
                    age_queue.insert(it, pos);
                    is_inserted = true;
                    break;
                }
            }
            if (!is_inserted) {
                age_queue.push_back(pos);
            }
        }
    }
    return age_queue;
}

void HMCMemorySystem::InsertReqToDRAM(HMCRequest *req) {
    std::cout << "";//For some reason the code does not work properly without this 
    //Transaction trans(req->mem_operand1, req->is_write);
    /* Rewriting for CIM transactions */
    //Steps:
    //1) Break the HMC requests into reads and writes
    //2) Maintain an ID(req_id_) to keep track of transactions belonging to a particular request.
    //   This is a global ID. Have to come up with someother way to keep track.
    if (req->type == HMCReqType::CIM_FETCH || req->type == HMCReqType::CIM_STORE) {
        //Store => write
        //Fetch => read
        int req_id = req_id_;
        Transaction trans(req->mem_operand1, req->type == HMCReqType::CIM_STORE);
        trans.is_cim = true;
        trans.is_cim_fetch = req->type == HMCReqType::CIM_FETCH;
        trans.is_cim_store = req->type == HMCReqType::CIM_STORE;
        trans.req_id = req_id;
        trans.is_cim_add = false;
        trans.is_cim_xor = false;
        trans.is_cim_swap = false;
        ctrls_[req->vault]->cim_transactions[req_id] = 1;
        ctrls_[req->vault]->AddTransaction(trans);
        id_to_cim_mappings[req_id] = (req->type == HMCReqType::CIM_FETCH) ? HMCReqType::CIM_FETCH : HMCReqType::CIM_STORE;
        req_id_++;
        return;
    }
    else if (req->type == HMCReqType::CIM_ADD || req->type == HMCReqType::CIM_XOR) { //2 reads followed by 1 write
        int req_id = req_id_;
        uint64_t address;
        ctrls_[req->vault]->cim_transactions[req_id] = 0;
        clock_cycle_recordings[req_id].push_back(logic_clk_);
        for (int i = 0; i < 3; i++) {
            if (i == 1)
                address = req->mem_operand1;
            else if(i == 2)
                address = req->mem_operand2;
            else
                address = req->mem_operand3;
            Transaction trans(address, i > 1);
            trans.is_cim_add = req->type == HMCReqType::CIM_ADD;
            trans.is_cim_xor = req->type == HMCReqType::CIM_XOR;
            trans.req_id = req_id;
            trans.is_cim = true;
            trans.is_cim_fetch = false;
            trans.is_cim_store = false;
            trans.is_cim_swap = false;
            ctrls_[req->vault]->AddTransaction(trans);
            ctrls_[req->vault]->cim_transactions[req_id]++;
        }
        id_to_cim_mappings[req_id] = (req->type == HMCReqType::CIM_ADD) ? HMCReqType::CIM_ADD : HMCReqType::CIM_XOR;
        req_id_++;

    }
    else if (req->type == HMCReqType::CIM_SWAP) {
        int req_id = req_id_;
        uint64_t address;
        for (int i = 0; i < 2; i++) {
            if (i == 0) {
                address = req->mem_operand1;
                cim_swap_store_addresses[req_id].first = address;
            }
            else {
                address = req->mem_operand2;
                cim_swap_store_addresses[req_id].second = address;
            }
            Transaction trans(address, false); //Read the values, swap happens in the vault callback function
            trans.is_cim = true;
            trans.is_cim_swap = true;
            trans.is_cim_fetch = trans.is_cim_store = trans.is_cim_add= trans.is_cim_xor=false;
            trans.req_id = req_id;
            ctrls_[req->vault]->AddTransaction(trans);
            ctrls_[req->vault]->cim_transactions[req_id]++;
        }
        id_to_cim_mappings[req_id] = HMCReqType::CIM_SWAP;
        cim_swap_call_backs[req_id] = 2;
        req_to_vault[req_id_] = req->vault;
        clock_cycle_recordings[req_id].push_back(logic_clk_);
        req_id_++;
    }
    else if (req->is_write || req->is_read) {
        Transaction trans(req->mem_operand1, req->is_write);
        trans.is_cim = false;
        trans.is_cim_fetch = trans.is_cim_store = trans.is_cim_add = trans.is_cim_xor = trans.is_cim_swap = false;
        if (req->is_read) {
            trans.is_write = false;
            trans.is_read = true;
        }
        else {
            trans.is_write = true;
            trans.is_read = false;
        }
        ctrls_[req->vault]->AddTransaction(trans);
        return;
    }
    else {
    AbruptExit(__FILE__, __LINE__);
    }
    return;
}

void HMCMemorySystem::VaultCallback(uint64_t req_id) {
    // we will use hex addr as the req_id and use a multimap to lookup the
    // requests the vaults cannot directly talk to the CPU so this callback will
    // be passed to the vaults and is responsible to put the responses back to
    // response queues
    /*CIM modifications*/
    //1) Add cim stats
    //2) Do not send response for CIM Operations
    auto it = resp_lookup_table_.find(req_id);
    HMCResponse *resp = it->second;
    if (resp->type <= HMCRespType::SIZE) {
        // all data from dram received, put packet in xbar and return
        resp_lookup_table_.erase(it);
        // put it in xbar
        quad_resp_queues_[resp->quad].push_back(resp);
        quad_age_counter_[resp->quad] = 1;
    }
    else if (id_to_cim_mappings[req_id] == HMCReqType::CIM_ADD || id_to_cim_mappings[req_id] == HMCReqType::CIM_XOR) {
            clock_cycle_recordings[req_id].push_back(logic_clk_);
            if (id_to_cim_mappings[req_id] == HMCReqType::CIM_ADD) {
                std::cout << "Clock Cycles for Add: " << clock_cycle_recordings[req_id].back() - clock_cycle_recordings[req_id].front()+ add_delay << "\n";
            }
            else {
                std::cout << "Clock Cycles for Xor: " << clock_cycle_recordings[req_id].back() - clock_cycle_recordings[req_id].front()+ xor_delay << "\n";
            }
        }

    else if(id_to_cim_mappings[req_id] == HMCReqType::CIM_SWAP) {//The stores are done in the callback
            if (cim_swap_call_backs[req_id]-- == 2) {
                //std::cout << "first call back for CiM_SWAP\n";
                uint64_t address;
                for (int i = 0; i < 2; i++) {
                    if (i == 0) {
                        address = cim_swap_store_addresses[req_id].first;
                    }
                    else {
                        address = cim_swap_store_addresses[req_id].second;
                    }
                    Transaction trans(address, false); //Read the values, swap happens in the vault callback function
                    trans.is_cim = true;
                    trans.is_cim_swap = true;
                    trans.is_cim_fetch = trans.is_cim_store = trans.is_cim_add = trans.is_cim_xor = false;
                    trans.req_id = req_id;
                    int vault = req_to_vault[req_id];
                    ctrls_[vault]->AddTransaction(trans);
                    ctrls_[vault]->cim_transactions[req_id]++;
                }
            }
            else {
                //std::cout << "second call back for CiM_SWAP\n";
                clock_cycle_recordings[req_id].push_back(logic_clk_);
                std::cout << "Clock cycles for CiM_Swap: " << clock_cycle_recordings[req_id].back() - clock_cycle_recordings[req_id].front() << "\n";
            }
        }
    return;
}

bool ConvertCIMtoRW(Transaction& trans) {
    return true;
}
}  // namespace dramsim3
