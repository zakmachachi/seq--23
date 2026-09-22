/* Host stub for the libDaisy surface midi_oled_monitor.cpp touches. */
#pragma once
#include <stddef.h>
#include <stdint.h>

#define DMA_BUFFER_MEM_SECTION
#define DTCM_MEM_SECTION


namespace daisy
{

struct Pin
{
    int index = 0;
};

class System
{
  public:
    /* Driven by the harness so the firmware's millisecond timing is real. */
    static uint32_t now_ms;
    static uint32_t GetNow() { return now_ms; }
};

class I2CHandle
{
  public:
    enum class Result
    {
        OK,
        ERR
    };

    struct Config
    {
        enum class Peripheral
        {
            I2C_1
        };
        enum class Speed
        {
            I2C_1MHZ
        };
        enum class Mode
        {
            I2C_MASTER
        };

        Peripheral periph = Peripheral::I2C_1;
        Speed      speed  = Speed::I2C_1MHZ;
        Mode       mode   = Mode::I2C_MASTER;

        struct
        {
            Pin scl;
            Pin sda;
        } pin_config;
    };

    typedef void (*CallbackFunctionPtr)(void* context, Result result);

    Result Init(const Config&) { return Result::OK; }

    Result TransmitBlocking(uint16_t, uint8_t*, uint16_t, uint32_t)
    {
        return Result::OK;
    }

    /* No panel on the bench rig: report completion immediately. */
    Result TransmitDma(uint16_t,
                       uint8_t*,
                       uint16_t,
                       CallbackFunctionPtr cb,
                       void*               context)
    {
        if(cb)
            cb(context, Result::OK);
        return Result::OK;
    }
};

namespace AudioHandle
{
typedef const float* const* InputBuffer;
typedef float**             OutputBuffer;
typedef void (*AudioCallback)(InputBuffer, OutputBuffer, size_t);
} // namespace AudioHandle

/* Thrown by StartAudio so the harness can run the firmware's real init path
 * and then take control instead of entering its while(1). */
struct HostAudioStarted
{
};

class DaisySeed
{
  public:
    void Configure() {}
    void Init(bool = false) {}
    void SetAudioBlockSize(size_t n) { block_size = n; }
    void SetLed(bool) {}
    Pin  GetPin(int i) { return Pin{i}; }

    void StartAudio(AudioHandle::AudioCallback cb)
    {
        callback = cb;
        throw HostAudioStarted{};
    }

    size_t                      block_size = 8;
    AudioHandle::AudioCallback  callback   = nullptr;
};

} // namespace daisy
