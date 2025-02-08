#include <vector>

#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"

namespace Ramulator{
  class RoRaCoBaBgCh final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoRaCoBaBgCh, "RoRaCoBaBgCh", "Applies a RoRaCoBaBgCh mapping to the address. (Default ChampSim)");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;
    }


    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      //channel
      req.addr_vec[m_dram->m_levels("channel")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);
      //bank group
      if(m_dram->m_organization.count.size() > 5)
      req.addr_vec[m_dram->m_levels("bankgroup")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]);
      //bank
      req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]);
      //column
      req.addr_vec[m_dram->m_levels("column")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("column")]);
      //rank
      req.addr_vec[m_dram->m_levels("rank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);
      //row
      req.addr_vec[m_dram->m_levels("row")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);
    }
    
  };

  class AldrLake final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, AldrLake, "AldrLake", "Applies a AldrLake mapping to the address. (Default ChampSim)");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1; 

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;


      //sanity check
      Request R(268435456,0,0,nullptr);
      bool bitmap[64];
      for(int i = 0; i < 64; i++)
        bitmap[i] = false;
      for(int i = 0; i < 64; i++)
      {
        R.addr = 268435456 + i;
        apply(R);
        int rank_num = R.addr_vec[m_dram->m_levels("rank")];
        int bg_num = R.addr_vec[m_dram->m_levels("bankgroup")];
        int bank_num = R.addr_vec[m_dram->m_levels("bank")];

        int index = (rank_num * 32) + (bg_num * 4) + bank_num;
        assert(!bitmap[index]);
        bitmap[index] = true;
      }
    }

    bool gitBit(unsigned long BitIndex, Addr_t address){
      return (((1<<BitIndex) & address ) !=0); // return the selected bit from the adress
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t saved = req.addr;
      Addr_t addr = req.addr >> m_tx_offset;
      //Addr_t saved = addr; //save given adress with offsett
      //channel
      req.addr_vec[m_dram->m_levels("channel")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);
      //column
      req.addr_vec[m_dram->m_levels("column")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("column")]);
      //bank
      req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]);
      //bank group
      if(m_dram->m_organization.count.size() > 5)
      req.addr_vec[m_dram->m_levels("bankgroup")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]);

      //rank
      req.addr_vec[m_dram->m_levels("rank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);
      //row
      req.addr_vec[m_dram->m_levels("row")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      req.addr_vec[m_dram->m_levels("rank")] = req.addr_vec[m_dram->m_levels("rank")] ^ (gitBit(27, saved) ^ gitBit(31,saved));

      req.addr_vec[m_dram->m_levels("bankgroup")] = req.addr_vec[m_dram->m_levels("bank")] ^ ((gitBit(24, saved) ^ gitBit(28, saved) ^ gitBit(32, saved)) |
                                                ((gitBit(25, saved) ^ gitBit(29, saved) ^ gitBit(33, saved))<<1)) |
                                                ((gitBit(26, saved) ^ gitBit(30, saved) ^ gitBit(34, saved))<<2);
      
       req.addr_vec[m_dram->m_levels("bank")] = req.addr_vec[m_dram->m_levels("bankgroup")] ^  ((gitBit(10, saved) ^ gitBit(19, saved)) |
                                                ((gitBit(9, saved) ^ gitBit(20, saved))<<1));
    }
    
  };

class ZEN4 final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ZEN4, "ZEN4", "Applies a ZEN4 mapping to the address");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1; 

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;


      //sanity check, check that we don't map into ourselves anywhere
      Request R(268435456,0,0,nullptr);
      int groups = 128;
      bool bitmap[groups*2];
      for(int i = 0; i < groups * 2; i++)
        bitmap[i] = false;
      for(int i = 0; i < groups; i++)
      {
        R.addr = 268435456 + i*tx_bytes;
        apply(R);
        int rank_num = R.addr_vec[m_dram->m_levels("rank")];
        int bg_num = R.addr_vec[m_dram->m_levels("bankgroup")];
        int bank_num = R.addr_vec[m_dram->m_levels("bank")];
        int channel_num = R.addr_vec[m_dram->m_levels("channel")];
        int column_num = R.addr_vec[m_dram->m_levels("column")];
        int index = bank_num;
        index |= (bg_num << m_addr_bits[m_dram->m_levels("bank")]);
        index |= (rank_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]));
        index |= (channel_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]) + m_addr_bits[m_dram->m_levels("rank")]);
        if(bitmap[index+(column_num*groups)]) {
          fmt::print("addr: {} index: {} column: {} assert: {}\n",R.addr,index, column_num,!bitmap[index+(column_num*groups)]);
          std::abort();
        }
        
        bitmap[index+(column_num*groups)] = true;
      }
    }

    bool gitBit(unsigned long BitIndex, Addr_t address){
      return (((1<<BitIndex) & address ) !=0); // return the selected bit from the adress
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t reference_addr = req.addr;
      Addr_t addr = req.addr >> m_tx_offset;

      //channel (6)
      long int channel_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);

      //column bit 0 (7)
      long int column_bits = slice_lower_bits(addr, 1);
      
      //bankgroup bits 0 and 1 (8-9)
      long int bg_bits = slice_lower_bits(addr,2);

      //bank bits (10-11)
      long int bank_bits = slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("bank")]);

      //remaining bankgroup bits (12)
      bg_bits |= slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("bankgroup")] - 2) << 2;

      //remaining column bits (13-17)
      column_bits |= slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("column")] - 1) << 1;

      //rank bits (18)
      long int rank_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);

      //row bits  (19-34)
      long int row_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      //XOR time

      //channel xors with all row bits
      for(int i = 0; i < m_addr_bits[m_dram->m_levels("row")]; i++)
        channel_bits ^= gitBit(row_bits,i);

      //bankgroup gets specific bits
      int bg_key = 0x1084;
      bg_bits ^= (gitBit(row_bits,2) ^ gitBit(row_bits,7) ^ gitBit(row_bits,12));
      bg_bits ^= (gitBit(row_bits,3) ^ gitBit(row_bits,8) ^ gitBit(row_bits,13)) << 1;
      bg_bits ^= (gitBit(row_bits,4) ^ gitBit(row_bits,9) ^ gitBit(row_bits,14)) << 2;

      //banks get specific bits
      int b_key = 0x8421;
      bank_bits ^= (gitBit(row_bits,0) ^ gitBit(row_bits,5) ^ gitBit(row_bits,10) ^ gitBit(row_bits,15));
      bank_bits ^= ( gitBit(row_bits,1) ^ gitBit(row_bits,6) ^ gitBit(row_bits,11)) << 1;

      //assign
      req.addr_vec[m_dram->m_levels("channel")] = channel_bits;
      req.addr_vec[m_dram->m_levels("column")] = column_bits;
      req.addr_vec[m_dram->m_levels("bank")] = bank_bits;
      req.addr_vec[m_dram->m_levels("bankgroup")] = bg_bits;
      req.addr_vec[m_dram->m_levels("rank")] = rank_bits;
      req.addr_vec[m_dram->m_levels("row")] = row_bits;
    }
  };

  class ZEN4_8GB final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ZEN4_8GB, "ZEN4_8GB", "Applies a ZEN4 8GB mapping to the address");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1; 

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;


      //sanity check, check that we don't map into ourselves anywhere
      Request R(268435456,0,0,nullptr);
      int groups = 32;
      bool bitmap[groups*2];
      for(int i = 0; i < groups * 2; i++)
        bitmap[i] = false;
      for(int i = 0; i < groups; i++)
      {
        R.addr = 268435456 + i*tx_bytes;
        apply(R);
        int rank_num = R.addr_vec[m_dram->m_levels("rank")];
        int bg_num = R.addr_vec[m_dram->m_levels("bankgroup")];
        int bank_num = R.addr_vec[m_dram->m_levels("bank")];
        int channel_num = R.addr_vec[m_dram->m_levels("channel")];
        int column_num = R.addr_vec[m_dram->m_levels("column")];
        int index = bank_num;
        index |= (bg_num << m_addr_bits[m_dram->m_levels("bank")]);
        index |= (rank_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]));
        index |= (channel_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]) + m_addr_bits[m_dram->m_levels("rank")]);
        if(bitmap[index+(column_num*groups)]) {
          fmt::print("addr: {} index: {} column: {} assert: {}\n",R.addr,index, column_num,!bitmap[index+(column_num*groups)]);
          std::abort();
        }
        
        bitmap[index+(column_num*groups)] = true;
      }
    }

    bool gitBit(unsigned long BitIndex, Addr_t address){
      return (((1<<BitIndex) & address ) !=0); // return the selected bit from the adress
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t reference_addr = req.addr;
      Addr_t addr = req.addr >> m_tx_offset;

      //channel (6)
      long int channel_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);

      //column bit 0 (7)
      long int column_bits = slice_lower_bits(addr, 1);
      
      //bankgroup bits (8-9)
      long int bg_bits = slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("bankgroup")]);

      //bank bits (10-11)
      long int bank_bits = slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("bank")]);

      //remaining column bits (12-16)
      column_bits |= slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("column")] - 1) << 1;

      //row bits (17-32)
      long int row_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      //XOR time

      //channel xors with all row bits
      for(int i = 0; i < m_addr_bits[m_dram->m_levels("row")]; i++)
        channel_bits ^= gitBit(row_bits,i);

      //bankgroup gets specific bits
      int bg_key = 0x2222;
      bg_bits ^= (gitBit(row_bits,2) ^ gitBit(row_bits,6) ^ gitBit(row_bits,10) ^ gitBit(row_bits,14));
      bg_bits ^= (gitBit(row_bits,3) ^ gitBit(row_bits,7) ^ gitBit(row_bits,11) ^ gitBit(row_bits,15)) << 1;

      //banks get specific bits
      int b_key = 0x1111;
      bank_bits ^= (gitBit(row_bits,0) ^ gitBit(row_bits,4) ^ gitBit(row_bits,8) ^ gitBit(row_bits,12));
      bank_bits ^= (gitBit(row_bits,1) ^ gitBit(row_bits,5) ^ gitBit(row_bits,9) ^ gitBit(row_bits,13)) << 1;

      //assign
      req.addr_vec[m_dram->m_levels("channel")] = channel_bits;
      req.addr_vec[m_dram->m_levels("column")] = column_bits;
      req.addr_vec[m_dram->m_levels("bank")] = bank_bits;
      req.addr_vec[m_dram->m_levels("bankgroup")] = bg_bits;
      req.addr_vec[m_dram->m_levels("rank")] = 0;
      req.addr_vec[m_dram->m_levels("row")] = row_bits;
    }
  };

  class ZEN4_8GB_RAF final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ZEN4_8GB_RAF, "ZEN4_8GB_RAF", "Applies a ZEN4 8GB mapping to the address");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1; 

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;


      //sanity check, check that we don't map into ourselves anywhere
      Request R(268435456,0,0,nullptr);
      int groups = 32;
      bool bitmap[groups*2];
      for(int i = 0; i < groups * 2; i++)
        bitmap[i] = false;
      for(int i = 0; i < groups; i++)
      {
        R.addr = 268435456 + i*tx_bytes;
        apply(R);
        int rank_num = R.addr_vec[m_dram->m_levels("rank")];
        int bg_num = R.addr_vec[m_dram->m_levels("bankgroup")];
        int bank_num = R.addr_vec[m_dram->m_levels("bank")];
        int channel_num = R.addr_vec[m_dram->m_levels("channel")];
        int column_num = R.addr_vec[m_dram->m_levels("column")];
        int index = bank_num;
        index |= (bg_num << m_addr_bits[m_dram->m_levels("bank")]);
        index |= (rank_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]));
        index |= (channel_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]) + m_addr_bits[m_dram->m_levels("rank")]);
        //if(bitmap[index+(column_num*groups)]) {
        //  fmt::print("addr: {} index: {} column: {} assert: {}\n",R.addr,index, column_num,!bitmap[index+(column_num*groups)]);
        //  std::abort();
        //}
        
        bitmap[index+(column_num*groups)] = true;
      }
    }

    bool gitBit(unsigned long BitIndex, Addr_t address){
      return (((1<<BitIndex) & address ) !=0); // return the selected bit from the adress
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t reference_addr = req.addr;
      Addr_t addr = req.addr >> m_tx_offset;

      //channel
      long int channel_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);

      //interleave column and bankgroup/bank bits

      //column bit 0
      long int column_bits = slice_lower_bits(addr, 1);
      //bankgroup bit 0
      long int bg_bits = slice_lower_bits(addr,1);
      //column bit 1
      column_bits |= slice_lower_bits(addr, 1) << 1;
      //bankgroup bit 1
      bg_bits |= slice_lower_bits(addr,1) << 1;
      //column bit 2
      column_bits |= slice_lower_bits(addr, 1) << 2;
      //bank bit 0
      long int bank_bits = slice_lower_bits(addr,1);
      //column bit 3
      column_bits |= slice_lower_bits(addr, 1) << 3;
      //bank bit 1
      bank_bits |= slice_lower_bits(addr,1) << 1;
      //column bits 4-6
      column_bits |= slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("column")] - 4) << 4;

      //row bits
      long int row_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      //XOR time

      //channel xors with all row bits
      for(int i = 0; i < m_addr_bits[m_dram->m_levels("row")]; i++)
        channel_bits ^= gitBit(row_bits,i);

      //bankgroup gets specific bits
      int bg_key = 0x2222;
      bg_bits ^= (gitBit(row_bits,2) ^ gitBit(row_bits,6) ^ gitBit(row_bits,10) ^ gitBit(row_bits,14));
      bg_bits ^= (gitBit(row_bits,3) ^ gitBit(row_bits,7) ^ gitBit(row_bits,11) ^ gitBit(row_bits,15)) << 1;

      //banks get specific bits
      int b_key = 0x1111;
      bank_bits ^= (gitBit(row_bits,0) ^ gitBit(row_bits,4) ^ gitBit(row_bits,8) ^ gitBit(row_bits,12));
      bank_bits ^= (gitBit(row_bits,1) ^ gitBit(row_bits,5) ^ gitBit(row_bits,9) ^ gitBit(row_bits,13)) << 1;

      //assign
      req.addr_vec[m_dram->m_levels("channel")] = channel_bits;
      req.addr_vec[m_dram->m_levels("column")] = column_bits;
      req.addr_vec[m_dram->m_levels("bank")] = bank_bits;
      req.addr_vec[m_dram->m_levels("bankgroup")] = bg_bits;
      req.addr_vec[m_dram->m_levels("rank")] = 0;
      req.addr_vec[m_dram->m_levels("row")] = row_bits;
    }
  };

  class ZEN4_8GB_RAF_2 final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ZEN4_8GB_RAF_2, "ZEN4_8GB_RAF_2", "Applies a ZEN4 8GB mapping to the address");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1; 

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;


      //sanity check, check that we don't map into ourselves anywhere
      Request R(268435456,0,0,nullptr);
      int groups = 32;
      bool bitmap[groups*2];
      for(int i = 0; i < groups * 2; i++)
        bitmap[i] = false;
      for(int i = 0; i < groups; i++)
      {
        R.addr = 268435456 + i*tx_bytes;
        apply(R);
        int rank_num = R.addr_vec[m_dram->m_levels("rank")];
        int bg_num = R.addr_vec[m_dram->m_levels("bankgroup")];
        int bank_num = R.addr_vec[m_dram->m_levels("bank")];
        int channel_num = R.addr_vec[m_dram->m_levels("channel")];
        int column_num = R.addr_vec[m_dram->m_levels("column")];
        int index = bank_num;
        index |= (bg_num << m_addr_bits[m_dram->m_levels("bank")]);
        index |= (rank_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]));
        index |= (channel_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]) + m_addr_bits[m_dram->m_levels("rank")]);
        //if(bitmap[index+(column_num*groups)]) {
        //  fmt::print("addr: {} index: {} column: {} assert: {}\n",R.addr,index, column_num,!bitmap[index+(column_num*groups)]);
        //  std::abort();
        //}
        
        bitmap[index+(column_num*groups)] = true;
      }
    }

    bool gitBit(unsigned long BitIndex, Addr_t address){
      return (((1<<BitIndex) & address ) !=0); // return the selected bit from the adress
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t reference_addr = req.addr;
      Addr_t addr = req.addr >> m_tx_offset;

      //channel
      long int channel_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);

      //interleave column and bankgroup/bank bits

      //column bits
      long int column_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("column")]);

      //bankgroup bits
      long int bg_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]);

      //bank bits
      long int bank_bits = slice_lower_bits(addr,m_addr_bits[m_dram->m_levels("bank")]);

      //row bits
      long int row_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      //XOR time

      //channel xors with all row bits
      for(int i = 0; i < m_addr_bits[m_dram->m_levels("row")]; i++)
        channel_bits ^= gitBit(row_bits,i);

      //bankgroup gets specific bits
      int bg_key = 0x2222;
      bg_bits ^= (gitBit(row_bits,2) ^ gitBit(row_bits,6) ^ gitBit(row_bits,10) ^ gitBit(row_bits,14));
      bg_bits ^= (gitBit(row_bits,3) ^ gitBit(row_bits,7) ^ gitBit(row_bits,11) ^ gitBit(row_bits,15)) << 1;

      //banks get specific bits
      int b_key = 0x1111;
      bank_bits ^= (gitBit(row_bits,0) ^ gitBit(row_bits,4) ^ gitBit(row_bits,8) ^ gitBit(row_bits,12));
      bank_bits ^= (gitBit(row_bits,1) ^ gitBit(row_bits,5) ^ gitBit(row_bits,9) ^ gitBit(row_bits,13)) << 1;

      //assign
      req.addr_vec[m_dram->m_levels("channel")] = channel_bits;
      req.addr_vec[m_dram->m_levels("column")] = column_bits;
      req.addr_vec[m_dram->m_levels("bank")] = bank_bits;
      req.addr_vec[m_dram->m_levels("bankgroup")] = bg_bits;
      req.addr_vec[m_dram->m_levels("rank")] = 0;
      req.addr_vec[m_dram->m_levels("row")] = row_bits;
    }
  };

class ZEN4_8GB_RAF_3 final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ZEN4_8GB_RAF_3, "ZEN4_8GB_RAF_3", "Applies a ZEN4 8GB mapping to the address");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1; 

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;


      //sanity check, check that we don't map into ourselves anywhere
      Request R(268435456,0,0,nullptr);
      int groups = 32;
      bool bitmap[groups*2];
      for(int i = 0; i < groups * 2; i++)
        bitmap[i] = false;
      for(int i = 0; i < groups; i++)
      {
        R.addr = 268435456 + i*tx_bytes;
        apply(R);
        int rank_num = R.addr_vec[m_dram->m_levels("rank")];
        int bg_num = R.addr_vec[m_dram->m_levels("bankgroup")];
        int bank_num = R.addr_vec[m_dram->m_levels("bank")];
        int channel_num = R.addr_vec[m_dram->m_levels("channel")];
        int column_num = R.addr_vec[m_dram->m_levels("column")];
        int index = bank_num;
        index |= (bg_num << m_addr_bits[m_dram->m_levels("bank")]);
        index |= (rank_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]));
        index |= (channel_num << (m_addr_bits[m_dram->m_levels("bank")] + m_addr_bits[m_dram->m_levels("bankgroup")]) + m_addr_bits[m_dram->m_levels("rank")]);
        //if(bitmap[index+(column_num*groups)]) {
        //  fmt::print("addr: {} index: {} column: {} assert: {}\n",R.addr,index, column_num,!bitmap[index+(column_num*groups)]);
        //  std::abort();
        //}
        
        bitmap[index+(column_num*groups)] = true;
      }
    }

    bool gitBit(unsigned long BitIndex, Addr_t address){
      return (((1<<BitIndex) & address ) !=0); // return the selected bit from the adress
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t reference_addr = req.addr;
      Addr_t addr = req.addr >> m_tx_offset;

      //channel
      long int channel_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);

      //interleave column and bankgroup/bank bits

      //column bit 0, 1
      long int column_bits = slice_lower_bits(addr, 2);
      //bankgroup bit 0
      long int bg_bits = slice_lower_bits(addr,1);
      //column bit 2, 3
      column_bits |= slice_lower_bits(addr, 2) << 2;
      //bankgroup bit 1
      bg_bits |= slice_lower_bits(addr,1) << 1;
      //column bit 4, 5
      column_bits |= slice_lower_bits(addr, 2) << 4;
      //bank bit 0
      long int bank_bits = slice_lower_bits(addr,1);
      //column bit 6
      column_bits |= slice_lower_bits(addr, 1) << 6;
      //bank bit 1
      bank_bits |= slice_lower_bits(addr,1) << 1;

      //row bits
      long int row_bits = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      //XOR time

      //channel xors with all row bits
      for(int i = 0; i < m_addr_bits[m_dram->m_levels("row")]; i++)
        channel_bits ^= gitBit(row_bits,i);

      //bankgroup gets specific bits
      int bg_key = 0x2222;
      bg_bits ^= (gitBit(row_bits,2) ^ gitBit(row_bits,6) ^ gitBit(row_bits,10) ^ gitBit(row_bits,14));
      bg_bits ^= (gitBit(row_bits,3) ^ gitBit(row_bits,7) ^ gitBit(row_bits,11) ^ gitBit(row_bits,15)) << 1;

      //banks get specific bits
      int b_key = 0x1111;
      bank_bits ^= (gitBit(row_bits,0) ^ gitBit(row_bits,4) ^ gitBit(row_bits,8) ^ gitBit(row_bits,12));
      bank_bits ^= (gitBit(row_bits,1) ^ gitBit(row_bits,5) ^ gitBit(row_bits,9) ^ gitBit(row_bits,13)) << 1;

      //assign
      req.addr_vec[m_dram->m_levels("channel")] = channel_bits;
      req.addr_vec[m_dram->m_levels("column")] = column_bits;
      req.addr_vec[m_dram->m_levels("bank")] = bank_bits;
      req.addr_vec[m_dram->m_levels("bankgroup")] = bg_bits;
      req.addr_vec[m_dram->m_levels("rank")] = 0;
      req.addr_vec[m_dram->m_levels("row")] = row_bits;
    }
  };
  

  class PBPI_Mapping final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, PBPI_Mapping, "PBPI_Mapping", "Applies a PBPI Mapping to the address. (Alternate ChampSim)");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;
    }


    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);

      Addr_t col1_bits = 12 - m_tx_offset - m_addr_bits[m_dram->m_levels("bankgroup")] - m_addr_bits[m_dram->m_levels("bank")] - m_addr_bits[m_dram->m_levels("channel")];
      Addr_t col2_bits = m_addr_bits[m_dram->m_levels("column")] - col1_bits;
      Addr_t addr = req.addr >> m_tx_offset;
      Addr_t xor_bits = req.addr >> 17;

      //channel
      req.addr_vec[m_dram->m_levels("channel")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);
      //col 1
      req.addr_vec[m_dram->m_levels("column")] = slice_lower_bits(addr, col1_bits);
      //bank group and bank
      if(m_dram->m_organization.count.size() > 5)
      {
        int bankgroup_val = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]) ^ xor_bits;
        req.addr_vec[m_dram->m_levels("bankgroup")] = slice_lower_bits(bankgroup_val, m_addr_bits[m_dram->m_levels("bankgroup")]);

        int bank_val = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]) ^ (xor_bits >> m_addr_bits[m_dram->m_levels("bankgroup")]);
        req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(bank_val,m_addr_bits[m_dram->m_levels("bank")]);
      }
      else
      {
        int bank_val = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]) ^ xor_bits;
        req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(bank_val, m_addr_bits[m_dram->m_levels("bank")]);
      }
      //col 2
      req.addr_vec[m_dram->m_levels("column")] += slice_lower_bits(addr, col2_bits) << col1_bits;
      //rank
      req.addr_vec[m_dram->m_levels("rank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);
      //row
      req.addr_vec[m_dram->m_levels("row")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);
    }
    
  };
}