#include "dram_system.h"

#include <assert.h>

namespace dramsim3 {

// alternative way is to assign the id in constructor but this is less
// destructive
int BaseDRAMSystem::total_channels_ = 0;

BaseDRAMSystem::BaseDRAMSystem(Config &config, const std::string &output_dir,
                               std::function<void(uint64_t)> read_callback,
                               std::function<void(uint64_t)> write_callback)
    : read_callback_(read_callback),
      write_callback_(write_callback),
      last_req_clk_(0),
      config_(config),
      timing_(config_),
#ifdef THERMAL
      thermal_calc_(config_),
#endif  // THERMAL
      clk_(0) {
    total_channels_ += config_.channels;

#ifdef ADDR_TRACE
    std::string addr_trace_name = config_.output_prefix + "addr.trace";
    address_trace_.open(addr_trace_name);
#endif
}

int BaseDRAMSystem::GetChannel(uint64_t hex_addr) const {
    hex_addr >>= config_.shift_bits;
    return (hex_addr >> config_.ch_pos) & config_.ch_mask;
}

void BaseDRAMSystem::PrintEpochStats() {
    // first epoch, print bracket
    if (clk_ - config_.epoch_period == 0) {
        std::ofstream epoch_out(config_.json_epoch_name, std::ofstream::out);
        epoch_out << "[";
    }
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->PrintEpochStats();
        std::ofstream epoch_out(config_.json_epoch_name, std::ofstream::app);
        epoch_out << "," << std::endl;
    }
#ifdef THERMAL
    thermal_calc_.PrintTransPT(clk_);
#endif  // THERMAL
    return;
}

void BaseDRAMSystem::PrintStats() {
    // Finish epoch output, remove last comma and append ]
    std::ofstream epoch_out(config_.json_epoch_name, std::ios_base::in |
                                                         std::ios_base::out |
                                                         std::ios_base::ate);
    epoch_out.seekp(-2, std::ios_base::cur);
    epoch_out.write("]", 1);
    epoch_out.close();

    std::ofstream json_out(config_.json_stats_name, std::ofstream::out);
    json_out << "{";

    // close it now so that each channel can handle it
    json_out.close();
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->PrintFinalStats();
        if (i != ctrls_.size() - 1) {
            std::ofstream chan_out(config_.json_stats_name, std::ofstream::app);
            chan_out << "," << std::endl;
        }
    }
    json_out.open(config_.json_stats_name, std::ofstream::app);
    json_out << "}";

#ifdef THERMAL
    thermal_calc_.PrintFinalPT(clk_);
#endif  // THERMAL
}

void BaseDRAMSystem::ResetStats() {
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ResetStats();
    }
}

void BaseDRAMSystem::RegisterCallbacks(
    std::function<void(uint64_t)> read_callback,
    std::function<void(uint64_t)> write_callback) {
    // TODO this should be propagated to controllers
    read_callback_ = read_callback;
    write_callback_ = write_callback;
}

JedecDRAMSystem::JedecDRAMSystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback) {
    if (config_.IsHMC()) {
        std::cerr << "Initialized a memory system with an HMC config file!"
                  << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }

    ctrls_.reserve(config_.channels);
    //Get the CiM values from the config file
    CiM_Add_Delay = config_.CiM_Add_Delay;
    CiM_Xor_Delay = config_.CiM_Xor_Delay;
    CiM_Swap_Delay = config_.CiM_Swap_Delay;
    for (auto i = 0; i < config_.channels; i++) {
#ifdef THERMAL
        ctrls_ctrls_.push_back(new Controller(i, config_, timing_, thermal_calc_));
#else
        ctrls_.push_back(new Controller(i, config_, timing_));
#endif  // THERMAL
    }
}

JedecDRAMSystem::~JedecDRAMSystem() {
    for (auto it = ctrls_.begin(); it != ctrls_.end(); it++) {
        delete (*it);
    }
}

bool JedecDRAMSystem::WillAcceptTransaction(uint64_t hex_addr,
                                            bool is_write) const {

    int channel = GetChannel(hex_addr);
    return ctrls_[channel]->WillAcceptTransaction(hex_addr, is_write);
}

bool JedecDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
// Record trace - Record address trace for debugging or other purposes
#ifdef ADDR_TRACE
    address_trace_ << std::hex << hex_addr << std::dec << " "
                   << (is_write ? "WRITE " : "READ ") << clk_ << std::endl;
#endif
    int channel = GetChannel(hex_addr);
    bool ok = ctrls_[channel]->WillAcceptTransaction(hex_addr, is_write);

    assert(ok);
    if (ok) {
        Transaction trans = Transaction(hex_addr, is_write);
        ctrls_[channel]->AddTransaction(trans);
    }
    last_req_clk_ = clk_;
    return ok;
}


/* Overloading for CIM. */
bool JedecDRAMSystem::WillAcceptTransaction(Transaction& trans) const {
    int channel_1, channel_2, channel_3;
    if (trans.is_cim_add  || trans.is_cim_xor) { //CiM_Add or CiM_Xor
        channel_1 = GetChannel(trans.addr);
        channel_2 = GetChannel(trans.addr2);
        channel_3 = GetChannel(trans.addr3);
        return ctrls_[channel_1]->WillAcceptTransaction(trans.addr, false) && ctrls_[channel_2]->WillAcceptTransaction(trans.addr2, false)
            && ctrls_[channel_3]->WillAcceptTransaction(trans.addr, true);
    }
    if (trans.is_cim_swap) { //CiM_Swap
        channel_1 = GetChannel(trans.addr);
        channel_2 = GetChannel(trans.addr2);
        return ctrls_[channel_1]->WillAcceptTransaction(trans.addr, false) && ctrls_[channel_1]->WillAcceptTransaction(trans.addr, true)
            && ctrls_[channel_2]->WillAcceptTransaction(trans.addr2, false) && ctrls_[channel_2]->WillAcceptTransaction(trans.addr2, true);
    }
    return true;
}
/************************************************************/

/*Overloading for CIM*/
bool JedecDRAMSystem::AddTransaction(Transaction& trans) { 
#ifdef ADDR_TRACE
    address_trace_ << std::hex << hex_addr << std::dec << " "
        << (is_write ? "WRITE " : "READ ") << clk_ << std::endl;
#endif
    bool ok = true;
    if (trans.is_cim_add || trans.is_cim_xor) {
        int channel_1 = GetChannel(trans.addr);
        int channel_2 = GetChannel(trans.addr2);
        int channel_3 = GetChannel(trans.addr3);
        ok = ctrls_[channel_1]->WillAcceptTransaction(trans.addr, false) && 
                    ctrls_[channel_2]->WillAcceptTransaction(trans.addr2, false) &&
                    ctrls_[channel_3]->WillAcceptTransaction(trans.addr3, true);
        assert(ok);
        if (ok) {
            Transaction dram_trans;
            for (int i = 0; i < 2; i++) {
                if (i == 0)
                    dram_trans = Transaction(trans.addr, false); //Issue two fetches
                else
                    dram_trans = Transaction(trans.addr2, false);
                dram_trans.is_cim_add = trans.is_cim_add;
                dram_trans.is_cim_xor = trans.is_cim_xor;
                dram_trans.is_cim_swap = false;
                dram_trans.is_cim = true;
                dram_trans.req_id = req_id_;
                if (i == 0)
                    ctrls_[channel_1]->AddTransaction(dram_trans);
                else
                    ctrls_[channel_2]->AddTransaction(dram_trans);

            }
        }
        no_of_reads_and_writes_for_cim[req_id_] = 2;
        pending_callbacks[req_id_] = 2;
        address_map_for_addxor[req_id_] = trans.addr3;
        req_id_to_cim[req_id_] = trans.is_cim_add ? CiMReqType::CiM_Add : CiMReqType::CiM_Xor;
        clock_cycle_record[req_id_].first = clk_;
        req_id_++;
    }
    else if (trans.is_cim_swap) { //2 fetches and 2 stores
        int channel_1 = GetChannel(trans.addr);
        int channel_2 = GetChannel(trans.addr2);
        ok = ctrls_[channel_1]->WillAcceptTransaction(trans.addr, false) &&
            ctrls_[channel_2]->WillAcceptTransaction(trans.addr2, false) &&
            ctrls_[channel_1]->WillAcceptTransaction(trans.addr, true) && ctrls_[channel_1]->WillAcceptTransaction(trans.addr2, true);
        assert(ok);
        if (ok) {
            Transaction dram_trans;
            for (int i = 0; i < 2; i++) {
                if (i == 0)
                    dram_trans = Transaction(trans.addr, false); //Issue two fetches
                else
                    dram_trans = Transaction(trans.addr2, false);
                dram_trans.is_cim_add = false;
                dram_trans.is_cim_xor = false;
                dram_trans.is_cim_swap = true;
                dram_trans.is_cim = true;
                dram_trans.req_id = req_id_;
                if (i == 0)
                    ctrls_[channel_1]->AddTransaction(dram_trans);
                else
                    ctrls_[channel_2]->AddTransaction(dram_trans);

            }
        }
        address_map_for_swap[req_id_].first = trans.addr;
        address_map_for_swap[req_id_].second = trans.addr2;
        no_of_reads_and_writes_for_cim[req_id_] = 2;
        pending_callbacks[req_id_] = 2;
        req_id_to_cim[req_id_] = CiMReqType::CiM_Swap;
        clock_cycle_record[req_id_].first = clk_;
        req_id_++;
    }
    last_req_clk_ = clk_;
    return ok;
}

/************************************************************/
void JedecDRAMSystem::ClockTick() {

    for (size_t i = 0; i < ctrls_.size(); i++) {
        // look ahead and return earlier
        while (true) {
            auto pair = ctrls_[i]->ReturnDoneTrans(clk_);
            if (pair.second == 1) {
                write_callback_(pair.first);
            } else if (pair.second == 0) {
                read_callback_(pair.first);
            } else if(pair.second == CIM) {
                no_of_reads_and_writes_for_cim[pair.first]--;
                if (no_of_reads_and_writes_for_cim[pair.first] == 0) {
                    CiM_CallBack(pair.first);
                }
                else
                    break;
            }
            else {
                break;
            }
        }
    }
    issue_pending_transactions(clk_);
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ClockTick();
    }
    clk_++;

    if (clk_ % config_.epoch_period == 0) {
        PrintEpochStats();
    }
    return;
}

/* Call back for CiM Type Transactions*/
void JedecDRAMSystem::CiM_CallBack(uint64_t req_id) {
    if (req_id_to_cim[req_id] == CiMReqType::CiM_Add || req_id_to_cim[req_id] == CiMReqType::CiM_Xor || req_id_to_cim[req_id] == CiMReqType::CiM_Swap) {
        int delay = 0;
        if (req_id_to_cim[req_id] == CiMReqType::CiM_Add)
            delay = CiM_Add_Delay;
        else if (req_id_to_cim[req_id] == CiMReqType::CiM_Xor)
            delay = CiM_Xor_Delay;
        else if (req_id_to_cim[req_id] == CiMReqType::CiM_Swap)
            delay = CiM_Swap_Delay;
        if (pending_callbacks[req_id] == 2) {
            pending_transactions[clk_ + delay].push_back(req_id);
            pending_callbacks[req_id]--;
        }
        else if(pending_callbacks[req_id] == 1) {
            clock_cycle_record[req_id].second = clk_;
            pending_callbacks[req_id]--;
            if(req_id_to_cim[req_id] == CiMReqType::CiM_Add)
            std::cout <<"Request no: "<<req_id<<" type: CiM_Add, no of clock cycles= "
                <<clock_cycle_record[req_id].second - clock_cycle_record[req_id].first << "\n";
            else if(req_id_to_cim[req_id] == CiMReqType::CiM_Xor)
                std::cout << "Request no: " << req_id << " type: CiM_Xor, no of clock cycles= "
                << clock_cycle_record[req_id].second - clock_cycle_record[req_id].first << "\n";
            else if(req_id_to_cim[req_id] == CiMReqType::CiM_Swap)
                std::cout << "Request no: " << req_id << " type: CiM_Swap, no of clock cycles= "
                << clock_cycle_record[req_id].second - clock_cycle_record[req_id].first << "\n";
        }

    }

}

void JedecDRAMSystem::issue_pending_transactions(uint64_t clk) {
    if (pending_transactions[clk].begin() != pending_transactions[clk].end()) {
        auto it = pending_transactions[clk].begin();
        while (it != pending_transactions[clk].end()) {
            uint64_t req_id = *it;
            if (req_id_to_cim[req_id] == CiMReqType::CiM_Add || req_id_to_cim[req_id] == CiMReqType::CiM_Xor) {
                uint64_t addr = address_map_for_addxor[req_id];
                int channel = GetChannel(addr);
                Transaction trans = Transaction(addr, true);
                trans.is_cim_add = req_id_to_cim[req_id] == CiMReqType::CiM_Add;
                trans.is_cim_xor = req_id_to_cim[req_id] == CiMReqType::CiM_Xor;
                trans.is_cim = true;
                trans.req_id = req_id;
                ctrls_[channel]->AddTransaction(trans);
                no_of_reads_and_writes_for_cim[req_id] = 1;
            }
            else {
                uint64_t addr1 = address_map_for_swap[req_id].first;
                uint64_t addr2 = address_map_for_swap[req_id].second;
                int channel1 = GetChannel(addr1);
                int channel2 = GetChannel(addr2);
                Transaction trans1 = Transaction(addr1, true);
                Transaction trans2 = Transaction(addr2, true);
                trans1.is_cim_add = false;
                trans2.is_cim_add = false;
                trans1.is_cim_xor = false;
                trans2.is_cim_xor = false;
                trans1.is_cim_swap = true;
                trans2.is_cim_swap = true;
                trans1.is_cim = true;
                trans2.is_cim = true;
                trans1.req_id = req_id;
                trans2.req_id = req_id;
                ctrls_[channel1]->AddTransaction(trans1);
                ctrls_[channel2]->AddTransaction(trans2);
                no_of_reads_and_writes_for_cim[req_id] = 2;
            }
            it++;
        }

    }

}


IdealDRAMSystem::IdealDRAMSystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback),
      latency_(config_.ideal_memory_latency) {}

IdealDRAMSystem::~IdealDRAMSystem() {}

bool IdealDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
    auto trans = Transaction(hex_addr, is_write);
    trans.added_cycle = clk_;
    infinite_buffer_q_.push_back(trans);
    return true;
}

void IdealDRAMSystem::ClockTick() {
    for (auto trans_it = infinite_buffer_q_.begin();
         trans_it != infinite_buffer_q_.end();) {
        if (clk_ - trans_it->added_cycle >= static_cast<uint64_t>(latency_)) {
            if (trans_it->is_write) {
                write_callback_(trans_it->addr);
            } else {
                read_callback_(trans_it->addr);
            }
            trans_it = infinite_buffer_q_.erase(trans_it++);
        }
        if (trans_it != infinite_buffer_q_.end()) {
            ++trans_it;
        }
    }

    clk_++;
    return;
}


}  // namespace dramsim3
