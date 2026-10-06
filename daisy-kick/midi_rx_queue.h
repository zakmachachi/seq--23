#pragma once
#include <stdint.h>

// Single ISR producer / foreground consumer. Publish each slot only after
// writing it; preserve stream discontinuities at the exact resumption point.
struct MidiRxQueue
{
    static constexpr uint16_t GAP = 0x100;
    static constexpr unsigned SIZE = 1024;
    volatile uint16_t bytes[SIZE] = {};
    volatile unsigned write = 0, read = 0;
    volatile uint32_t overflow = 0;
    bool gap = false; // producer-owned
    static void Barrier() { __atomic_signal_fence(__ATOMIC_SEQ_CST); }
    void Discontinuity() { gap = true; }
    bool Push(uint8_t b)
    {
        unsigned next = (write + 1) & (SIZE - 1);
        if(next == read) { ++overflow; gap = true; return false; }
        bytes[write] = b | (gap ? GAP : 0);
        gap = false;
        Barrier(); write = next;
        return true;
    }
    bool Pop(uint16_t& b)
    {
        if(read == write) return false;
        Barrier(); b = bytes[read];
        Barrier(); read = (read + 1) & (SIZE - 1);
        return true;
    }
};
