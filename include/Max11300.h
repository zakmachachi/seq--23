#ifndef MAX11300_H
#define MAX11300_H

#include <Arduino.h>
#include <SPI.h>

// Minimal MAX11300 (PIXI) driver for analog CV output.
//
// Only the DAC path is implemented: ports are configured as voltage-output
// DACs and driven from a 12-bit code. The MAX11300 talks SPI mode 0, MSB
// first; every access is a 1-byte command (register address << 1, LSB = R/W)
// followed by a 16-bit big-endian data word.
//
// Register map and bit fields are taken from the MAX11300 PIXI Configuration
// Software User Guide. The handful of device_control bits we rely on are
// documented inline so they're easy to correct against the IC datasheet if a
// value turns out wrong on hardware.
class Max11300 {
  public:
    // Register addresses (see user guide register map).
    enum Reg : uint8_t {
      REG_DEVICE_ID      = 0x00, // r/o device identification
      REG_DEVICE_CONTROL = 0x10, // r/w global control
      REG_PORT_CFG_BASE  = 0x20, // port_cfg_00..19 = 0x20 + port
      REG_DAC_DATA_BASE  = 0x60, // dac_data_port_00..19 = 0x60 + port
    };

    // DAC output ranges (funcprm_range field, bits [10:8] of port_cfg).
    enum DacRange : uint8_t {
      RANGE_0_TO_10  = 1, // 0V .. +10V
      RANGE_NEG5_5   = 2, // -5V .. +5V
      RANGE_NEG10_0  = 3, // -10V .. 0V
    };

    explicit Max11300(uint8_t csPin, uint32_t spiHz = 8000000)
      : _cs(csPin), _settings(spiHz, MSBFIRST, SPI_MODE0) {}

    // Initialise SPI + the device. Returns true if the device ID reads back
    // with the expected MAX11300 part field. Caller must have the SPI pins
    // wired to the hardware SPI bus.
    bool begin();

    // Raw register access.
    void     writeReg(uint8_t addr, uint16_t value);
    uint16_t readReg(uint8_t addr);

    // Configure a PIXI port (0..19) as a voltage-output DAC with the given range.
    void configDac(uint8_t port, DacRange range = RANGE_0_TO_10);

    // Write a 12-bit code (0..4095) to a DAC port.
    void writeDacCode(uint8_t port, uint16_t code12);

    // Set a 0..10V DAC port's output in volts (clamped to 0..10).
    void setVoltage0to10(uint8_t port, float volts);

    // 1V/oct helper for a 0..10V port: note0 sits at 0V, +1 octave = +1V.
    // midiNote 0..120 spans 0..10V.
    void setNote1VOct0to10(uint8_t port, int midiNote);

    bool present() const { return _present; }

  private:
    uint8_t     _cs;
    SPISettings _settings;
    bool        _present = false;
};

#endif // MAX11300_H
