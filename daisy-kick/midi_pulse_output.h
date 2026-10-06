#pragma once
#include <atomic>
#include <cstdint>

// Main-loop producer, audio-callback consumer. Bursts become distinct pulses
// with a guaranteed low gap rather than merging into one long gate.
class MidiPulseOutput
{
  public:
    static constexpr uint32_t kCapacity = 256;
    static_assert(ATOMIC_INT_LOCK_FREE == 2, "Pulse queue needs lock-free atomics");
    std::atomic<uint32_t> dropped{0};

    void Request()
    {
        uint32_t count=pending_.load(std::memory_order_relaxed);
        while(count<kCapacity)
        {
            if(pending_.compare_exchange_weak(count,count+1,std::memory_order_relaxed))
                return;
        }
        dropped.fetch_add(1,std::memory_order_relaxed);
    }

    // Called once at each real audio callback boundary, NOT inside the sample
    // loop (samples inside a callback are computed much faster than real time).
    bool Advance(uint32_t block_samples,uint32_t high_samples,uint32_t low_samples)
    {
        if(remaining_>block_samples)
        {
            remaining_-=block_samples;
            return high_;
        }
        remaining_=0;
        if(high_)
        {
            high_=false;
            remaining_=low_samples;
        }
        else if(pending_.load(std::memory_order_relaxed)>0)
        {
            pending_.fetch_sub(1,std::memory_order_relaxed);
            high_=true;
            remaining_=high_samples;
        }
        return high_;
    }

    // Caller must stop the audio callback before resetting.
    void Reset()
    {
        pending_.store(0,std::memory_order_relaxed);
        remaining_=0;
        high_=false;
    }
    uint32_t Pending() const { return pending_.load(std::memory_order_relaxed); }

  private:
    std::atomic<uint32_t> pending_{0};
    uint32_t remaining_=0;
    bool high_=false;
};
