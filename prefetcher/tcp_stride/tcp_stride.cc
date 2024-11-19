#include "tcp_stride.h"

#include "cache.h"

std::vector<tcp_stride*> tcp_stride::instances;

uint32_t tcp_stride::prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                             uint32_t metadata_in)
{
  champsim::block_number cl_addr{addr};
  champsim::block_number::difference_type stride = 0;


  auto found = table.check_hit({ip, cl_addr, stride});

  auto best_entry = [](const auto& lhs, const auto& rhs) {
      //if the rhs is the one we don't want, return true
    if (rhs.has_value() && !lhs.has_value()) {
      return true;
    }
    if (lhs.has_value() && !rhs.has_value()) {
      return false;
    }
    if(!rhs.has_value() && !lhs.has_value()) {
      return true;
    }
    return (rhs.value().degree > lhs.value().degree);
  };
// if we found a matching entry
if (found.has_value()) {
  stride = champsim::offset(found->last_cl_addr, cl_addr);

  // Initialize prefetch state unless we somehow saw the same address twice in
  // a row or if this is the first time we've seen this stride
  if (stride != 0 && stride == found->last_stride) {
    auto al = std::min_element(active_lookahead.begin(),active_lookahead.end(),best_entry);

    *al = {champsim::address{cl_addr}, stride, aggression};
  }
}

  // update tracking set
  table.fill({ip, cl_addr, stride});

  if(useful_prefetch)
    useful_counter++;
  
  if(!cache_hit)
    miss_counter++;
  else
    hit_counter++;

  return metadata_in;
}

void tcp_stride::prefetcher_cycle_operate()
{
  // If a lookahead is active
  //fmt::print("trying lookahead...\n");
  for(auto it = active_lookahead.begin(); it != active_lookahead.end(); it++) {
    if (it->has_value()) {
      auto [old_pf_address, stride, degree] = it->value();
      //fmt::print("entry: {} {} {}\n",old_pf_address,stride,degree);
      if(degree > 0) {
        champsim::address pf_address{champsim::block_number{old_pf_address} + stride};

        // If the next step would exceed the degree or run off the page, stop
        if (intern_->virtual_prefetch || champsim::page_number{pf_address} == champsim::page_number{old_pf_address}) {
          // check the MSHR occupancy to decide if we're going to prefetch to this level or not
          const bool mshr_under_light_load = intern_->get_mshr_occupancy_ratio() < 0.5;
          const bool success = prefetch_line(pf_address, mshr_under_light_load, 0);
          if (success)
            *it = {pf_address, stride, degree - 1};

          // If we fail, try again next cycle
          if(it->value().degree != 0)
            continue;

        }
      }
      it->reset();
    }
  }

  //aggression modulation
  cycles_so_far++;

  if(intern_->current_cycle() % heartbeat_size == 0)
    fmt::print("hit_rate: {} usefulness: {}\n",(hit_counter / (float)(miss_counter + hit_counter)),((useful_counter + useful_counter_lw + useful_counter_llw) /(float) (pf_fill_counter + pf_fill_counter_lw + pf_fill_counter_llw)));
  if(cycles_so_far >= cycles_per_window) {
    cycles_so_far = 0;
    if(got_back_off || ((useful_counter + useful_counter_lw + useful_counter_llw) /(float) (pf_fill_counter + pf_fill_counter_lw + pf_fill_counter_llw)) < 0.50) {
      aggression *= back_off_coeff;
    }
    else {
      aggression += back_on_coeff;
      if(aggression > aggression_max)
        aggression = aggression_max;
    }
    got_back_off = false;
    rolling_aggression += aggression;
    windows++;

    pf_fill_counter_lllw = pf_fill_counter_llw;
    pf_fill_counter_llw = pf_fill_counter_lw;
    pf_fill_counter_lw = pf_fill_counter;
    pf_fill_counter = 0;

    useful_counter_llw = useful_counter_lw;
    useful_counter_lw = useful_counter;
    useful_counter = 0;
    miss_counter = 0;
    hit_counter = 0;
  }
  if(intern_->current_cycle() % heartbeat_size == 0)
    fmt::print("current aggression: {} average aggression: {}\n",aggression,rolling_aggression/(float)windows);
}


uint32_t tcp_stride::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
{
  if(prefetch)
    pf_fill_counter++;
  return metadata_in;
}

void tcp_stride::prefetcher_initialize() {
  instances.push_back(this);
}

void tcp_stride::back_off(champsim::address addr, uint32_t cpu) {
  for(auto it : instances) {
    if(it->intern_->cpu == cpu)
      it->got_back_off = true;
  }
}