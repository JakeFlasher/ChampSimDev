#include "spp_raf_llc.h"
#include "../prefetcher/spp_raf_l2c/spp_raf_l2c.h"

uint32_t spp_raf_llc::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                             uint32_t metadata_in)
{
  
  //FOR BLACKLIST PREFETCH LLC MISSES APPROACH
  //update filters for spp_raf_l2c
  if(type == access_type::PREFETCH && !cache_hit)
  for(auto spp_l2c : spp_raf_l2c::spp_impls)
  {
      
      spp_l2c->FILTER.raf_bloom_mark(addr);
  }

  //FOR ROW TABLE APPROACH
  if(cache_hit == 0)
  for(auto spp_l2c : spp_raf_l2c::spp_impls)
  {
      spp_l2c->FILTER.set_row_table(addr);
  }

  //FOR RAM TABLE APPROACH
  if(cache_hit == 0)
  for(auto spp_l2c : spp_raf_l2c::spp_impls)
  {
      spp_l2c->FILTER.set_ram_table(addr);
  }

  if(cache_hit == 0 && type == access_type::PREFETCH)
  for(auto spp_l2c : spp_raf_l2c::spp_impls)
  {
      spp_l2c->FILTER.set_nram_table(addr);
  }

  return metadata_in;
}

uint32_t spp_raf_llc::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  return metadata_in;
}
