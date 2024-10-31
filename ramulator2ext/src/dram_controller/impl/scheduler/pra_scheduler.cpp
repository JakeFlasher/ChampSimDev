#include <vector>

#include "base/base.h"
#include "dram_controller/bh_controller.h"
#include "dram_controller/bh_scheduler.h"

namespace Ramulator {

class PRAScheduler : public IBHScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IBHScheduler, PRAScheduler, "PRAScheduler", "PRA Scheduler.")

  private:
    IDRAM* m_dram;

    int m_clk = -1;

    bool m_is_debug;


    uint64_t m_prefetch_scheduled;
    uint64_t m_ready_scheduled;
    uint64_t m_fcfs_scheduled;

  public:
    void init() override {
      m_prefetch_scheduled = 0;
      m_ready_scheduled = 0;
      m_fcfs_scheduled = 0;
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;
      register_stat(m_prefetch_scheduled).name("packets scheduled by prefetch");
      register_stat(m_ready_scheduled).name("packets scheduled by readiness");
      register_stat(m_fcfs_scheduled).name("packets scheduled by fcfs");

    }

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
      bool ready2 = m_dram->check_ready(req2->command, req2->addr_vec);

      //we also need to check if its a prefetch and not a row hit
      //if its a prefetch, not a row hit, and new enough that its not starving, don't serve
      bool pact1 = req1->is_prefetch && !m_dram->check_rowbuffer_hit(req1->command,req1->addr_vec);
      bool pact2 = req2->is_prefetch && !m_dram->check_rowbuffer_hit(req2->command,req2->addr_vec);

      //first ready, first served
      if ((ready1) ^ (ready2)) {
        m_ready_scheduled++;
        if (ready1) {
          return req1;
        } else {
          return req2;
        }
      }

      //now do prefetch prio for tie-breaker
      //this gives scheduling prio to prefetches that don't incur row activations
      if(pact1 ^ pact2) {
        m_prefetch_scheduled++;
        if (!pact1) {
          return req1;
        } else {
          return req2;
        }
      }

      m_fcfs_scheduled++;
      //finally do FCFS if they are still tied, i.e. both are ready, but both cause row activations and are prefetches
      if (req1->arrive <= req2->arrive) {
        return req1;
      } else {
        return req2;
      } 
    }

    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }

      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        candidate = compare(candidate, next);
      }
      return candidate;
    }

    virtual void tick() override {
      m_clk++;
    }
};

}       // namespace Ramulator
