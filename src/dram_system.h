#ifndef __DRAM_SYSTEM_H
#define __DRAM_SYSTEM_H

#include <fstream>
#include <string>
#include <vector>

#include "common.h"
#include "configuration.h"
#include "controller.h"
#include "timing.h"

#ifdef THERMAL
#include "thermal.h"
#endif  // THERMAL

namespace dramsim3 {

enum class CiMReqType {
    CiM_Add,
    CiM_Xor,
    CiM_Swap
    };

class BaseDRAMSystem {
   public:
    BaseDRAMSystem(Config &config, const std::string &output_dir,
                   std::function<void(uint64_t)> read_callback,
                   std::function<void(uint64_t)> write_callback);
    virtual ~BaseDRAMSystem() {}
    void RegisterCallbacks(std::function<void(uint64_t)> read_callback,
                           std::function<void(uint64_t)> write_callback);
    void PrintEpochStats();
    void PrintStats();
    void ResetStats();

    virtual bool WillAcceptTransaction(uint64_t hex_addr,
                                       bool is_write) const = 0;
    virtual bool AddTransaction(uint64_t hex_addr, bool is_write) = 0;
    //Overloading for CIM
    virtual bool WillAcceptTransaction(Transaction& trans) const = 0;
    virtual bool AddTransaction(Transaction& trans) = 0;
    
    virtual void ClockTick() = 0;
    int GetChannel(uint64_t hex_addr) const;

    std::function<void(uint64_t req_id)> read_callback_, write_callback_;
    static int total_channels_;

   protected:
    uint64_t req_id_;
    uint64_t last_req_clk_;
    Config &config_;
    Timing timing_;
    uint64_t parallel_cycles_;
    uint64_t serial_cycles_;

#ifdef THERMAL
    ThermalCalculator thermal_calc_;
#endif  // THERMAL

    uint64_t clk_;
    std::vector<Controller*> ctrls_;

#ifdef ADDR_TRACE
    std::ofstream address_trace_;
#endif  // ADDR_TRACE
};

// hmmm not sure this is the best naming...
class JedecDRAMSystem : public BaseDRAMSystem {
   std::unordered_map<uint64_t,int> no_of_reads_and_writes_for_cim;
   std::unordered_map<uint64_t,int> pending_callbacks;
   std::unordered_map<uint64_t, uint64_t> address_map_for_addxor;
   std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> address_map_for_swap;
   std::unordered_map<uint64_t, CiMReqType> req_id_to_cim;
   std::unordered_map<uint64_t,std::pair<uint64_t,uint64_t>> clock_cycle_record;
   std::unordered_map<uint64_t, std::vector<uint64_t>> pending_transactions;
   int CiM_Add_Delay;
   int CiM_Xor_Delay;
   int CiM_Swap_Delay;
   public:
    JedecDRAMSystem(Config &config, const std::string &output_dir,
                    std::function<void(uint64_t)> read_callback,
                    std::function<void(uint64_t)> write_callback);
    ~JedecDRAMSystem();
    bool WillAcceptTransaction(uint64_t hex_addr, bool is_write) const override;
    bool AddTransaction(uint64_t hex_addr, bool is_write) override;
    //Overloading for CIM
    bool WillAcceptTransaction(Transaction& trans) const override;
    bool AddTransaction(Transaction& trans) ;
    void CiM_CallBack(uint64_t req_id);
    void issue_pending_transactions(uint64_t clk);
    void ClockTick() override;
};

// Model a memorysystem with an infinite bandwidth and a fixed latency (possibly
// zero) To establish a baseline for what a 'good' memory standard can and
// cannot do for a given application
class IdealDRAMSystem : public BaseDRAMSystem {
   public:
    IdealDRAMSystem(Config &config, const std::string &output_dir,
                    std::function<void(uint64_t)> read_callback,
                    std::function<void(uint64_t)> write_callback);
    ~IdealDRAMSystem();
    bool WillAcceptTransaction(uint64_t hex_addr,
                               bool is_write) const override {
        return true;
    };
    bool AddTransaction(uint64_t hex_addr, bool is_write) override;
    void ClockTick() override;

   private:
    int latency_;
    std::vector<Transaction> infinite_buffer_q_;
};

}  // namespace dramsim3
#endif  // __DRAM_SYSTEM_H
