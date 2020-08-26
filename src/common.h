#ifndef __COMMON_H
#define __COMMON_H

#include <stdint.h>
#include <iostream>
#include <vector>

namespace dramsim3 {
#define CIM 2
enum CIM_OPERATIONS{CIM_FETCH,CIM_STORE,CIM_ADD,CIM_XOR,CIM_SWAP};
struct Address {
    Address()
        : channel(-1), rank(-1), bankgroup(-1), bank(-1), row(-1), column(-1) {}
    Address(int channel, int rank, int bankgroup, int bank, int row, int column)
        : channel(channel),
          rank(rank),
          bankgroup(bankgroup),
          bank(bank),
          row(row),
          column(column) {}
    Address(const Address& addr)
        : channel(addr.channel),
          rank(addr.rank),
          bankgroup(addr.bankgroup),
          bank(addr.bank),
          row(addr.row),
          column(addr.column) {}
    int channel;
    int rank;
    int bankgroup;
    int bank;
    int row;
    int column;
};

inline uint32_t ModuloWidth(uint64_t addr, uint32_t bit_width, uint32_t pos) {
    addr >>= pos;
    auto store = addr;
    addr >>= bit_width;
    addr <<= bit_width;
    return static_cast<uint32_t>(store ^ addr);
}

// extern std::function<Address(uint64_t)> AddressMapping;
int GetBitInPos(uint64_t bits, int pos);
// it's 2017 and c++ std::string still lacks a split function, oh well
std::vector<std::string> StringSplit(const std::string& s, char delim);
template <typename Out>
void StringSplit(const std::string& s, char delim, Out result);

int LogBase2(int power_of_two);
void AbruptExit(const std::string& file, int line);
bool DirExist(std::string dir);

enum class CommandType {
    READ,
    READ_PRECHARGE,
    WRITE,
    WRITE_PRECHARGE,
    ACTIVATE,
    PRECHARGE,
    REFRESH_BANK,
    REFRESH,
    SREF_ENTER,
    SREF_EXIT,
    SIZE
    
};

struct Command {
    Command() : cmd_type(CommandType::SIZE), hex_addr(0) {}
    Command(CommandType cmd_type, const Address& addr, uint64_t hex_addr)
        : cmd_type(cmd_type), addr(addr), hex_addr(hex_addr) {}
    // Command(const Command& cmd) {}

    bool IsValid() const { return cmd_type != CommandType::SIZE; }
    bool IsRefresh() const {
        return cmd_type == CommandType::REFRESH ||
               cmd_type == CommandType::REFRESH_BANK;
    }
    bool IsRead() const {
        return cmd_type == CommandType::READ ||
               cmd_type == CommandType ::READ_PRECHARGE;
    }
    bool IsWrite() const {
        return cmd_type == CommandType ::WRITE ||
               cmd_type == CommandType ::WRITE_PRECHARGE;
    }
    bool IsReadWrite() const { return IsRead() || IsWrite(); }
    bool IsRankCMD() const {
        return cmd_type == CommandType::REFRESH ||
               cmd_type == CommandType::SREF_ENTER ||
               cmd_type == CommandType::SREF_EXIT;
    }
    
    CommandType cmd_type;
    Address addr;
    uint64_t hex_addr;

    int Channel() const { return addr.channel; }
    int Rank() const { return addr.rank; }
    int Bankgroup() const { return addr.bankgroup; }
    int Bank() const { return addr.bank; }
    int Row() const { return addr.row; }
    int Column() const { return addr.column; }

    friend std::ostream& operator<<(std::ostream& os, const Command& cmd);
};

struct Transaction {
    Transaction() {}
    Transaction(uint64_t addr, bool is_write)
        : addr(addr),
          added_cycle(0),
          complete_cycle(0),
          is_write(is_write),
          addr2(0),
          addr3(0),
          req_id(0),
          is_read(!is_write) {is_cim=0;}
    Transaction(const Transaction& tran)
        : addr(tran.addr),
          added_cycle(tran.added_cycle),
          complete_cycle(tran.complete_cycle),
          is_write(tran.is_write),req_id(0),is_read(false),is_cim_fetch(false),is_cim_store(false),is_cim_add(false),is_cim_swap(false),is_cim_xor(false),is_cim(false) {
          req_id = tran.req_id;
          is_read = tran.is_read;
          is_cim_fetch = tran.is_cim_fetch;
          is_cim_store = tran.is_cim_store;
          is_cim_add = tran.is_cim_add;
          is_cim_xor = tran.is_cim_xor;
          is_cim_swap = tran.is_cim_swap;
          is_cim = tran.is_cim;
          }
          
    //Creating new constructors for CIM
    //For CIM transactions with one memory operands
    /*Transaction(uint64_t addr1, bool is_cim_fetch, bool is_cim_store)
                : addr(addr1),added_cycle(0),complete_cycle(0),
                 addr2(0),addr3(0), is_write(false), is_read(false), is_cim_fetch(is_cim_fetch),
                is_cim_store(is_cim_store), is_cim_add(false), is_cim_swap(false),is_cim_xor(false) {is_cim=0;} */
    //For CIM transactions with two memory operands
    /*Transaction(uint64_t addr1, uint64_t addr2,bool is_cim_add, bool is_cim_swap, bool is_cim_xor)
                : addr(addr1),added_cycle(0),complete_cycle(0),
                 addr2(addr2),addr3(addr3), is_write(false), is_read(false), is_cim_fetch(false),
                is_cim_store(false), is_cim_add(is_cim_add), is_cim_swap(is_cim_swap),is_cim_xor(is_cim_xor) {is_cim=0;}*/
                
                
    uint64_t addr;
    uint64_t added_cycle;
    uint64_t complete_cycle;
    bool is_write;
    friend std::ostream& operator<<(std::ostream& os, const Transaction& trans);
    friend std::istream& operator>>(std::istream& is, Transaction& trans);
    
    //Adding member variables for CIM operations
    uint64_t addr2; //the address of the second operand for CIM_Add and CIM_xor operations
    uint64_t addr3;
    uint64_t req_id;
    bool is_read;
    bool is_cim_fetch;
    bool is_cim_store;
    bool is_cim_add;
    bool is_cim_swap;
    bool is_cim_xor;
    bool is_cim; 
    
};

}  // namespace dramsim3
#endif
