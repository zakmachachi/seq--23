Power test for Teensy 4.1

Steps:
1. Plug your PCB in via USB (or provide VIN/5V). Observe the Teensy power LED (near the USB connector) — it should light when powered.
2. If you have a multimeter, measure 3.3V between the Teensy's 3.3V pin and GND.

To upload the test firmware (PlatformIO):

```bash
cd power_test
platformio run -e teensy41 -t upload
```

Behavior after upload:
- The onboard LED (pin 13) will blink (250ms on, 750ms off).
- USB serial will print `Power test: Teensy alive` at 115200 baud once on reset. Use the PlatformIO Serial Monitor or any serial terminal:

```bash
platformio device monitor -e teensy41 -b 115200
```

If LED blinks and serial prints, Teensy is powered and running.
