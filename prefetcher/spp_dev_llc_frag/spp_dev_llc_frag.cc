#include "spp_dev_llc_frag.h"
#include "dram_controller.h"

uint32_t spp_dev_llc_frag::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                             uint32_t metadata_in)
{
  //fmt::print("SPP_DEV_LLC_FRAG: OPERATING CACHE\n");
  //on miss
  if(cache_hit == 0) {
    //update row state table
    update_row_state_table(addr);
  }
  if(useful_prefetch) {
      last_useful++;
    }

  prefetch_column(addr,false);//!((cache_hit == 0 || is_row_open(addr)) && type == access_type::PREFETCH));

  //do next-line
  /*
  champsim::block_number pf_addr{addr};
  for(std::size_t offset = 1; offset <= 4; offset++) {
    //get pf address
    champsim::address pf_addr{champsim::block_number{addr} + offset};

    //get if row for prefetch is open
    bool row_is_open = is_row_open(pf_addr);

    //if our offset is below threshold, prefetch always. Otherwise, only if row is open
    if((offset <= 4 || row_is_open)) {

      //issue next line prefetch
      bool success = prefetch_line(pf_addr,true,metadata_in);

      if(success) {
        prefetch_column(pf_addr,0);
        next_line_prefetches_issued++;
      }

      //if it was a success, prefetch the next columns as well
      //if(success) {
      //  next_line_prefetches_issued++;
      //  if(row_is_open)
      //    prefetch_column(pf_addr);
      //  else
      //    column_prefetches_dropped+=4;
      //}
      //else
      //  full_queue++;
    
    }
  }*/

  return metadata_in;
}
uint32_t spp_dev_llc_frag::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{

  //look at fills, grab next columns from fills
  if(prefetch != 0 && evicted_addr != champsim::address{} && metadata_in != 0)
    last_evictions++;
  return metadata_in;
}

bool spp_dev_llc_frag::is_row_open(champsim::address addr) {
  return row_table.check_hit(row_state_table_entry{get_dram_group(addr),get_dram_row(addr)}).has_value();
}

void spp_dev_llc_frag::update_row_state_table(champsim::address addr) {
  row_table.fill(row_state_table_entry{get_dram_group(addr),get_dram_row(addr)});
}

void spp_dev_llc_frag::prefetch_column(champsim::address pf_addr, bool hit) {
  uint64_t target_column = get_dram_column(pf_addr);
  int radius = aggression_factor;
  for(int offset = 0; offset <= radius; offset++) {
    if(offset == 0)
      continue;
    
    if(hit) {
      column_prefetches_dropped++;
      continue;
    }

    champsim::address new_address = compose_base_and_column(pf_addr,target_column + offset);
    //fmt::print("Address: {} Expected Row: {} Actual Row: {} Expected Column: {} Actual Column: {}\n",new_address,get_dram_row(pf_addr),get_dram_row(new_address),target_column + offset,get_dram_column(new_address));
    if(offset < 0 && target_column >= std::abs(offset)) {
      if(prefetch_line(compose_base_and_column(pf_addr,target_column + offset),true,1))
        column_prefetches_issued++;
      else
        full_queue++;
    }
    else if(target_column + offset < BLOCKS_PER_RB - 1){
      if(prefetch_line(compose_base_and_column(pf_addr,target_column + offset),true,1))
        column_prefetches_issued++;
      else
        full_queue++;
    }
  }
  
}

//DRAM ADDRESS DECODING
uint64_t spp_dev_llc_frag::get_dram_column(champsim::address pf_addr) {
  uint64_t column_block = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->dram_get_column(pf_addr);
  //fmt::print("Converted pf_addr:{} to column:{}\n",pf_addr,column_block);
  return column_block;
}
uint64_t spp_dev_llc_frag::get_dram_row(champsim::address pf_addr) {
  uint64_t row = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->dram_get_row(pf_addr);
  //fmt::print("Converted pf_addr:{} to row:{}\n",pf_addr,row);
  return row;
}
unsigned int spp_dev_llc_frag::get_dram_group(champsim::address pf_addr) {
  std::size_t channel = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->dram_get_channel(pf_addr);
  std::size_t rank = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->dram_get_rank(pf_addr);
  std::size_t bank = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->dram_get_bank(pf_addr);

  std::size_t ranks = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->channels[0].ranks();
  std::size_t banks = MEMORY_CONTROLLER::DRAM_CONTROLLER.value()->channels[0].banks();

  std::size_t rb_id = 0;
        rb_id += bank;
        rb_id += rank * banks;
        rb_id += channel * ranks * banks;
  //id of which rowbuffer we hit
  
  //fmt::print("Converted pf_addr:{} to group:{}\n",pf_addr,rb_id);
  return(rb_id);
}

//DRAM ADDRESS RECODING
champsim::address spp_dev_llc_frag::compose_base_and_column(champsim::address base, uint64_t column) {
  //1. iterate through all column bits in the base
  //2. set each bit to the matching bits in the column
  uint64_t base_temp = base.to<uint64_t>();
  
  //8GB
  std::vector<uint64_t> column_bits = {7,12,13,14,15,16};
  //32GB
  //std::vector<uint64_t> column_bits = {7,13,14,15,16,17};

  for(std::size_t i = 0; i < column_bits.size(); i++) {
    if(column & (1ull << i))
      base_temp |= 1ull << column_bits[i];
    else
      base_temp &= ~(1ull << column_bits[i]);
  }
  return champsim::address{base_temp};
}

void spp_dev_llc_frag::prefetcher_final_stats() {
  fmt::print("SPP_LLC_STATS:\n");
  fmt::print("\tNEXT-COLUMN PREFETCHES ISSUED: {}\tDROPPED: {}\n",column_prefetches_issued,column_prefetches_dropped);
  fmt::print("\tNEXT-LINE PREFETCHES ISSUED: {}\tDROPPED: {}\n",next_line_prefetches_issued,next_line_prefetches_dropped);
  fmt::print("\tFULL QUEUE: {}\n",full_queue);
}

void spp_dev_llc_frag::prefetcher_cycle_operate() {

  current_cycle++;
  if(current_cycle % update_period == 0) {
    if((last_last_evictions*0.8) > last_useful) {
      if(aggression_factor > 0)
        aggression_factor--;
    }
    else {
      if(aggression_factor < max_aggression)
        aggression_factor++;
    }
    fmt::print("aggression factor: {} evicted: {} useful: {}\n",aggression_factor, last_evictions, last_useful);
    last_last_evictions = last_evictions;
    last_evictions = 0;
    last_useful = 0;
    
    
  }
  
}