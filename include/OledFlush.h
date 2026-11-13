#pragma once
#include <Adafruit_SH110X.h>
#include <Wire.h>

// Partial OLED updates. Every screen redraw starts with clearDisplay(), which
// makes Adafruit's display() resend all 1 KB (~25 ms of blocking I2C at
// 400 kHz). oledFlush() keeps a copy of what each panel last received and
// sends only the changed column span of each changed page, so a knob value
// change costs a few hundred microseconds instead of a full frame.

// Register a panel after its begin(). Unregistered panels fall back to display().
// ramColumn is the SH1106 column (0..4) that receives buffer column 0; the
// Adafruit library always uses 2.
void oledAttach(Adafruit_SH1106G& d, TwoWire& wire, uint8_t address, uint8_t ramColumn = 2);
// Push the frame buffer to the panel, sending only what changed.
void oledFlush(Adafruit_SH1106G& d);
// Force the next flush to resend the whole frame.
void oledInvalidate(Adafruit_SH1106G& d);
