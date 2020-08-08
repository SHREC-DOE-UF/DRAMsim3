#include "common.h"
#include "fmt/format.h"
#include <sstream>
#include <unordered_set>
#include <sys/stat.h>

namespace dramsim3 {

std::ostream& operator<<(std::ostream& os, const Command& cmd) {
    std::vector<std::string> command_string = {
        "read",
        "read_p",
        "write",
        "write_p",
        "activate",
        "precharge",
        "refresh_bank",  // verilog model doesn't distinguish bank/rank refresh
        "refresh",
        "self_refresh_enter",
        "self_refresh_exit",
        "WRONG"};
    os << fmt::format("{:<20} {:>3} {:>3} {:>3} {:>3} {:>#8x} {:>#8x}",
                      command_string[static_cast<int>(cmd.cmd_type)],
                      cmd.Channel(), cmd.Rank(), cmd.Bankgroup(), cmd.Bank(),
                      cmd.Row(), cmd.Column());
    return os;
}

std::ostream& operator<<(std::ostream& os, const Transaction& trans) {
    const std::string trans_type = trans.is_write ? "WRITE" : "READ";
    os << fmt::format("{:<30} {:>8}", trans.addr, trans_type);
    return os;
}

std::istream& operator>>(std::istream& is, Transaction& trans) {
    std::unordered_set<std::string> write_types = {"WRITE", "write", "P_MEM_WR",
                                                   "BOFF"};
    /*Some CIM operations have two addresses. Ex: CIM_ADD, CIM_XOR, CIM_SWAP*/
    std::unordered_set<std::string> cim_operations_with_two_addresses = {"CIM_ADD","CIM_SWAP","CIM_XOR" };
    std::unordered_set<std::string> cim_operations = { "CIM_ADD","CIM_SWAP","CIM_XOR" };
    std::string mem_op;
    is >> std::hex >> trans.addr ;
    trans.addr3 = 0;
    is >> mem_op;
    if (cim_operations_with_two_addresses.count(mem_op) == 1)//Store the second address
        is >> std::hex >> trans.addr2;
    if(mem_op == "CIM_ADD" || mem_op == "CIM_XOR")
        is >> std::hex >> trans.addr3;
    is>> std::dec >> trans.added_cycle;
    trans.is_write = write_types.count(mem_op) == 1;
    trans.is_read = mem_op == "READ";
    trans.is_cim_add = mem_op == "CIM_ADD";
    trans.is_cim_swap = mem_op == "CIM_SWAP";
    trans.is_cim_xor = mem_op == "CIM_XOR";
    trans.is_cim_fetch = mem_op == "CIM_FETCH";
    trans.is_cim_store = mem_op == "CIM_STORE";
    return is;
}

int GetBitInPos(uint64_t bits, int pos) {
    // given a uint64_t value get the binary value of pos-th bit
    // from MSB to LSB indexed as 63 - 0
    return (bits >> pos) & 1;
}

int LogBase2(int power_of_two) {
    int i = 0;
    while (power_of_two > 1) {
        power_of_two /= 2;
        i++;
    }
    return i;
}

std::vector<std::string> StringSplit(const std::string& s, char delim) {
    std::vector<std::string> elems;
    StringSplit(s, delim, std::back_inserter(elems));
    return elems;
}

template <typename Out>
void StringSplit(const std::string& s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) {
            *(result++) = item;
        }
    }
}

void AbruptExit(const std::string& file, int line) {
    std::cerr << "Exiting Abruptly - " << file << ":" << line << std::endl;
    std::exit(-1);
}

bool DirExist(std::string dir) {
    // courtesy to stackoverflow
    struct stat info;
    if (stat(dir.c_str(), &info) != 0) {
        return false;
    } else if (info.st_mode & S_IFDIR) {
        return true;
    } else {  // exists but is file
        return false;
    }
}

}  // namespace dramsim3
