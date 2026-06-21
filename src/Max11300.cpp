#include "Max11300.h"

// device_control bits we use:
//   RESET   = 0x8000  soft reset (self-clearing)
//   DACREF  = 0x0040  DAC voltage reference: 1 = internal 2.5V ref
//   DACCTL  = 0x000C  DAC update mode [3:2]; 0b01 = immediate update on write
// Internal reference + immediate update means a dac_data write takes effect at
// once without an external reference pin or the DAC sequencer.
static const uint16_t DEVCTL_RESET   = 0x8000;
static const uint16_t DEVCTL_DACREF  = 0x0040;
static const uint16_t DEVCTL_DAC_IMMEDIATE = 0x0004; // DACCTL = 0b01
static const uint16_t DEVCTL_RUN = DEVCTL_DACREF | DEVCTL_DAC_IMMEDIATE;

void Max11300::writeReg(uint8_t addr, uint16_t value){
  SPI.beginTransaction(_settings);
  digitalWrite(_cs, LOW);
  SPI.transfer((uint8_t)(addr << 1));        // LSB = 0 -> write
  SPI.transfer((uint8_t)(value >> 8));
  SPI.transfer((uint8_t)(value & 0xFF));
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

uint16_t Max11300::readReg(uint8_t addr){
  SPI.beginTransaction(_settings);
  digitalWrite(_cs, LOW);
  SPI.transfer((uint8_t)((addr << 1) | 1));  // LSB = 1 -> read
  uint8_t hi = SPI.transfer(0x00);
  uint8_t lo = SPI.transfer(0x00);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  return ((uint16_t)hi << 8) | lo;
}

bool Max11300::begin(){
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  SPI.begin();

  // Soft reset for a known state, then let it settle.
  writeReg(REG_DEVICE_CONTROL, DEVCTL_RESET);
  delay(5);

  // Bring up internal DAC reference + immediate update mode.
  writeReg(REG_DEVICE_CONTROL, DEVCTL_RUN);
  delay(5); // reference settle

  // Device ID part field (bits [15:12]) is 0x4 on the MAX11300. Accept any
  // non-0x0000/0xFFFF read as "present" so a slightly different ID still works.
  uint16_t id = readReg(REG_DEVICE_ID);
  _present = (id != 0x0000 && id != 0xFFFF);
  return _present;
}

void Max11300::configDac(uint8_t port, DacRange range){
  if (port > 19) return;
  // port_cfg: funcid (DAC = 0x5) in bits [15:12], range in bits [10:8].
  uint16_t cfg = ((uint16_t)0x5 << 12) | ((uint16_t)(range & 0x7) << 8);
  writeReg((uint8_t)(REG_PORT_CFG_BASE + port), cfg);
  delay(1); // port reconfiguration settle
  writeReg((uint8_t)(REG_DAC_DATA_BASE + port), 0x0000); // start at the low rail
}

void Max11300::writeDacCode(uint8_t port, uint16_t code12){
  if (port > 19) return;
  if (code12 > 0x0FFF) code12 = 0x0FFF;
  writeReg((uint8_t)(REG_DAC_DATA_BASE + port), code12);
}

void Max11300::setVoltage0to10(uint8_t port, float volts){
  if (volts < 0.0f) volts = 0.0f;
  if (volts > 10.0f) volts = 10.0f;
  uint16_t code = (uint16_t)(volts / 10.0f * 4095.0f + 0.5f);
  writeDacCode(port, code);
}

void Max11300::setNote1VOct0to10(uint8_t port, int midiNote){
  if (midiNote < 0) midiNote = 0;
  setVoltage0to10(port, (float)midiNote / 12.0f); // 12 semitones per volt
}
