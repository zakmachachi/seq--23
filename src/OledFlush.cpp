#include "OledFlush.h"
#include <string.h>

namespace {
constexpr uint8_t WIDTH = 128, PAGES = 8;
constexpr uint8_t RAM_WIDTH = 132;       // SH1106 RAM is 132 columns wide
constexpr uint32_t I2C_HZ = 400000;      // same bus speed Adafruit uses during display()
constexpr uint8_t MAX_CHUNK = 128;       // Teensy 4 Wire buffer holds 136 bytes

struct Panel {
  Adafruit_SH1106G* display = nullptr;
  TwoWire* wire = nullptr;
  uint8_t address = 0x3C;
  uint8_t ramColumn = 2;                 // SH1106 column of buffer column 0
  bool valid = false;                    // shadow matches the panel's RAM
  uint8_t shadow[WIDTH * PAGES] = {};
};
Panel panels[2];

Panel* find(Adafruit_SH1106G& d){
  for (Panel& p : panels) if (p.display == &d) return &p;
  return nullptr;
}
} // namespace

void oledAttach(Adafruit_SH1106G& d, TwoWire& wire, uint8_t address, uint8_t ramColumn){
  if (ramColumn > RAM_WIDTH - WIDTH) ramColumn = RAM_WIDTH - WIDTH;
  for (Panel& p : panels){
    if (p.display && p.display != &d) continue;
    p.display = &d; p.wire = &wire; p.address = address; p.ramColumn = ramColumn; p.valid = false;
    return;
  }
}

namespace {
// Writes `count` bytes starting at RAM column `column` of `page`.
bool writeSpan(Panel& panel, uint8_t page, uint8_t column, const uint8_t* data, int count){
  TwoWire& wire = *panel.wire;
  wire.beginTransmission(panel.address);
  wire.write((uint8_t)0x00);               // command stream
  wire.write((uint8_t)(0xB0 + page));      // page address
  wire.write((uint8_t)(0x10 | (column >> 4)));
  wire.write((uint8_t)(column & 0x0F));
  if (wire.endTransmission() != 0) return false;
  for (int x = 0; x < count;){
    const int n = (count - x) < MAX_CHUNK ? (count - x) : MAX_CHUNK;
    wire.beginTransmission(panel.address);
    wire.write((uint8_t)0x40);             // data stream
    wire.write(data + x, (size_t)n);
    if (wire.endTransmission() != 0) return false;
    x += n;
  }
  return true;
}
} // namespace

void oledInvalidate(Adafruit_SH1106G& d){
  if (Panel* p = find(d)) p->valid = false;
}

void oledFlush(Adafruit_SH1106G& d){
  Panel* panel = find(d);
  if (!panel){ d.display(); return; }
  const uint8_t* buffer = d.getBuffer();
  if (!buffer) return;
  panel->wire->setClock(I2C_HZ);
  for (uint8_t page = 0; page < PAGES; ++page){
    const uint8_t* row = buffer + page * WIDTH;
    uint8_t* seen = panel->shadow + page * WIDTH;
    bool ok;
    if (!panel->valid){
      // Full page across all 132 RAM columns: the spare columns either side
      // of the picture are cleared so a nudged panel shows no stray pixels.
      uint8_t full[RAM_WIDTH] = {};
      memcpy(full + panel->ramColumn, row, WIDTH);
      ok = writeSpan(*panel, page, 0, full, RAM_WIDTH);
      if (ok) memcpy(seen, row, WIDTH);
    } else {
      int first = 0, last = WIDTH - 1;
      while (first < WIDTH && row[first] == seen[first]) ++first;
      if (first == WIDTH) continue;          // page unchanged
      while (row[last] == seen[last]) --last;
      ok = writeSpan(*panel, page, (uint8_t)(first + panel->ramColumn), row + first, last - first + 1);
      if (ok) memcpy(seen + first, row + first, (size_t)(last - first + 1));
    }
    if (!ok){                                // panel missing or bus error:
      panel->valid = false;                  // resend everything next time
      return;
    }
  }
  panel->valid = true;
}
