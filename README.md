# seq-23: Sequencer Prototype (Work in Progress)
![Status: Prototype](https://img.shields.io/badge/Status-Prototype-orange?style=flat-square)
![Prototype Image](./images/prototype_v1.jpeg)
>[!WARNING]
>**Status:** WORK IN PROGRESS — ongoing prototype. Use at your own risk; expect frequent changes.

A **7-channel, 16-step** MIDI sequencer for Teensy 4.1 driving **two** SH1106 OLEDs, a button matrix, six dual-pot+button controls, a 30-LED WS2812 chain, and MIDI out via Serial5.

> [!CAUTION]
> The code was entirely _vibe coded_ since I have no clue how to write C and C++. All LLMs used were free versions.

---

## Quick start

```bash
cd Sequencer_prototybe
platformio run
platformio run --target upload
```

Connect MIDI OUT (Teensy pin 20 / Serial5 TX) through a standard MIDI driver circuit to your target's MIDI IN. Open the PlatformIO serial monitor at 115200 baud to see firmware logs.

Pin mappings live in [include/SeqConfig.h](include/SeqConfig.h).

---

## Hardware layout

| Component | Pins | Notes |
|---|---|---|
| Button matrix | rows 6–10, cols 0–5 | 5×6 = 30 keys, columns scan |
| 6× pot + button | A pins 38/40/14/22/24/26, B pins 39/41/15/23/25/27, buttons 36/35/34/33/30/31 | Two analog inputs per pot for direction-aware turning |
| Primary OLED (SH1106 128×64) | SDA 19, SCL 18 (Wire, I²C 400 kHz) | Per-menu detail view |
| Secondary OLED (SH1106 128×64) | SDA1 17, SCL1 16 (Wire1) | Dual-screen overview; auto-detected, optional |
| WS2812 LED chain | data pin 29, 30 LEDs in series | Step + channel + UI indicator LEDs |
| MIDI OUT | Serial5 TX = pin 20, RX = pin 21, 31250 baud | Drives any standard MIDI receiver |

Screen orientation is a build option, `SCREEN_ROTATION` (default **0°** for the public release). Boards with the OLEDs mounted upside down build at 180° via a local, gitignored `platformio_override.ini` containing `build_flags = -D SCREEN_ROTATION=2` — `platformio.ini` auto-loads any `*_override.ini` through `extra_configs`. The second screen is optional — if it isn't present at boot the firmware just runs the primary screen.

### Matrix index map

The matrix is row-major (`idx = row × 6 + col`). Verified assignments:

| Index | Button |
|---|---|
| 0–15 | Step buttons 1–16 |
| 16 | Page (Pages menu + transport/perf modifier) |
| 17 | Fill (performance) |
| 18 | Menu 1 — Notes (Generative) |
| 19 | Menu 2 — Step Visualizer |
| 20 | Menu 3 — Euclid |
| 21 | Menu 4 — Trigger Machines |
| 22 | Function (modifier) |
| 23–29 | Channel buttons **CH7…CH1** (idx 23 = CH7 … idx 29 = CH1) |

If a physical button doesn't behave as expected, send `t` over serial to enter switch test and confirm the index.

### LED chain map (30 LEDs, 0-based)

| Index | LED |
|---|---|
| 0–15 | Step buttons 1–16 |
| 16 | Page button |
| 17 | Fill button |
| 18–21 | Menu 1–4 buttons |
| 22 | Function button |
| 23–29 | Channel LEDs **CH7…CH1** (CH1 = idx 29 … CH7 = idx 23) |

---

## LED behaviour

- **Step LEDs (0–15):** show the selected channel's pattern on the current edit page. Active steps in the channel hue, dim hint when off, white playhead, fill-only steps a steady dim green.
- **Channel LEDs (23–29):** selected channel **green** (or **yellow** if it's muted); every other unmuted channel **red**; muted channels **off**. (Replaces the on-screen channel-number tabs.)
- **Menu / modifier LEDs (16–22):** the active menu's button lights up (the Page LED lights in the Pages menu); the Fill and Function LEDs light while held.
- **Paused hint:** when stopped, the **Function** and **Menu 4** LEDs slow-pulse white to show the Play combo.
- **Slide-all (Function + Fill):** the whole chain ripples as a dark-purple wave.
- **Accent-all (Function + Page):** the whole chain flashes red.

---

## Transport

| Action | How |
|---|---|
| Play / Stop | Hold **Function** + tap **Menu 4** |
| Tempo (BPM) ± 1 | Hold **Function** + rotate **Pot 1** anywhere |
| Save patch | Tap **Function** 5 times in a row (≤ 2 s between taps, no step held). `SAVED!` splash on success. |
| Clear current channel | Hold **Function + Page + Fill** together for **1 second** — `CLEAR CH n` splash |
| Clear EEPROM | Send `c` over serial |

Internal MIDI clock runs from the Teensy hardware timer at 24 PPQN. If external clock (`0xF8`) is detected on MIDI IN, the firmware switches to external sync automatically; if it stops for > 2 s, it falls back to internal.

---

## Channels

Seven independent tracks, each with its own pattern, pages, MIDI Out channel, default note, velocity, mute state, machine, scale, etc.

| Action | How |
|---|---|
| Select channel | Tap **CH1..CH7** button (its LED turns green) |
| Mute / Unmute channel | Hold **Function** + tap **CHn** — `MUTE` splash with strikethrough; LED turns yellow |
| Open channel focus (mute + MIDI Out) | Hold **CHn** — OLED switches to focus view |
| Change MIDI Out channel for held channel | Hold **CHn** + rotate **Pot 1** (1..16, persisted per channel) |
| Change root note for held channel | Hold **CHn** + rotate **Pot 2** (transposes the channel's notes) |

Default MIDI Out mapping at first boot: `CH1 → MIDI ch 1`, `CH2 → ch 2`, … `CH7 → ch 7`. Persisted to EEPROM after a save.

---

## Step editing

Step buttons toggle the active channel's pattern on the current edit page in every menu (including the Pages menu). While a step is **held**:

- **Tap Function** — cycle that step's Fill state (`NORMAL → FILL-ONLY`). Fill-only steps trigger only while the Fill button is held. Splash shows `STEP n FILL`.
- **Turn a pot (Notes page)** — write a **P-Lock** for that step (see below).

When a step is turned off it resets all per-step P-Locks (pitch, gate, velocity, slide, ratchet, fill) so re-enabling starts clean.

The **Fill** button held (alone) = global performance modifier: any step whose Fill state is `1` triggers; anti-fill (`2`) steps drop out. Fills are global (all channels).

### P-Locks (per-step parameter locks)

On the **Notes** page, hold a step and turn a pot to pin a value on just that step, independent of the channel/generative settings:

| Held step + | Locks |
|---|---|
| Pot 1 (rotate) | Note (pitch) for that step |
| Pot 4 (rotate) | Slide on/off for that step |
| Pot 5 (rotate) | Gate length for that step |
| Pot 6 (rotate) | Velocity for that step |

P-Locked steps keep their values until the channel is regenerated (Pot 2 button) — then they "re-join" the generated pattern. Random velocity / random gate (below) never override a P-Locked step.

---

## Menus

Five menus. Menu 1–4 are the matrix menu buttons; the Pages menu is opened with the Page button. On boot the unit starts on the **Notes** page.

### Menu 1 — Notes (Generative)
Per-channel generative note engine. The six pots are laid out in two rows of three to line up physically with the encoders:

| Pot | Rotate | Button press |
|---|---|---|
| Pot 1 | Root note (transposes existing pattern) | Toggle generative mode (scale 0 ↔ last used) |
| Pot 2 | Scale: Major, Minor, Penta, Locrian, Dim, Atonal | Re-randomize note sequence |
| Pot 3 | Octave spread 0–60 semitones | **Mutate** — nudge one currently-triggering note |
| Pot 4 | Slide probability 0–100 % (re-rolls slides) | — |
| Pot 5 | Gate length (11 divisions) | Toggle **random gate** (per note, full 1/32..1) |
| Pot 6 | Channel velocity 0–127 | Toggle **random velocity** (per note, ± 27) |

- **Function + Pot 3 (encoder 3) = contour bias** (−100…+100): clockwise biases new generations toward **ascending** motion, anti-clockwise toward **descending**; the harder you turn, the more consistently mono-directional. Shown as a growing up/down arrow on screen 2.
- **Mutate** (Pot 3 press) only changes a step that is actually being triggered (honours machine overlays, skips anti-fill steps).
- `R` next to GATE / VEL on screen indicates random gate / random velocity is on.

**Screen 2:** a piano-roll of the channel's notes — block height = pitch, width = note length (clipped to the next note). It follows the page that's **playing** on multi-page channels, and shows the contour-bias arrow on the right.

### Menu 2 — Step Visualizer
Read-only step grid showing the current channel's pattern, per-step state (Fill / Ratchet / Slide marks), and playhead. **Screen 2** shows a 7-row all-channel overview with playhead and ratchet pips.

### Menu 3 — Euclid
Euclidean rhythm generator, laid out like the Notes page (pattern grid + ON/OFF header over a pot-aligned 2×3 grid):

| Pot | Rotate | Button |
|---|---|---|
| Pot 1 | Pulses (number of triggers) | Toggle Euclid engine on/off |
| Pot 2 | Offset (rotates pattern) | — |
| Pot 3 | Scale mode | — |
| Pot 4 | Channel velocity | — |
| Pot 5 | Gate length | — |
| Pot 6 | Slide probability (re-rolls) | — |

**Screen 2:** a circular pulse view with slide chains drawn between sliding steps. When Euclid is off the ring simply reads as off (no redundant OFF label or empty bar).

### Menu 4 — Trigger Machines
Per-channel drum/trigger generators. All machines now share one **incremental accumulation** engine: each machine has a skeleton (always-on steps) and density adds one weighted-random extra trigger at a time. Lower-weighted steps come in less often, giving each machine its character; at maximum density every machine fills all 16 steps. Selecting a machine starts at density 0 (skeleton only). Dropping density back to 0 re-seeds a fresh set, so it varies slightly each pass.

| Pot | Action |
|---|---|
| Pot 1 (rotate) | Machine type: OFF, KICK, HIHAT, SNARE, ANTIKICK, PERC, EUCLID (resets density to 0) |
| Pot 2 (rotate) | Density 0 … machine pool size (incremental, one trigger per step) |
| Pot 3 (rotate) | Shift 0–15 (rotates the whole pattern) |
| Pot 4–6 (rotate) | KICK only: note spread, ratchet probability, "extras-as-fills" toggle |
| **Step button (1–16)** | Cycles the step's overlay: `AUTO → FORCE-ON → FORCE-OFF → AUTO` (persists across machine changes) |

Machines:
- **KICK** — 4-on-the-floor skeleton (steps 1, 5, 9, 13). Density first fills the 8th-note offbeats (~70 % weighted toward steps 3, 7, 11, 15) to split 4/4 into 8/8, with ~30 % variation on the other 1/16ths. KICK can also add note spread and ratchet fills on the extras.
- **HIHAT** — skeleton on the offbeats; fills downbeats then 1/16ths.
- **SNARE** — skeleton on the backbeats (5, 13); fills ghost notes around them.
- **ANTIKICK** — skeleton on the 1/8 offbeats; fills the other 1/16ths before the downbeats.
- **PERC** — skeleton on every 1/8; fills the in-between 1/16ths.
- **EUCLID** — re-uses the Menu 3 Euclid pattern, with the Menu 4 shift applied.

**Screen 2:** machine icon + name + a density fill bar (one segment per pool slot). OLED grid marks: `+` = force-on, line through cell = force-off, faint dot = machine wanted it on but you overrode it off. User P-Lock ratchets (Step Visualizer) merge with machine ratchets — the larger wins.

### Pages menu — Page button
Digitakt-style **1–4 pages per channel** plus per-channel pattern length and a rate multiplier.

| Control | Action |
|---|---|
| Tap **Page** | Open the Pages menu; tap again to cycle the page being edited |
| Pot 1 (rotate) | Pattern length in pages (1–4) |
| Pot 1 (button) | Toggle Global / Channel scope (Global broadcasts length/step changes to all channels) |
| Pot 2 (rotate) | Pattern step count (1–16) |
| Pot 3 (rotate) | Rate multiplier 0.25×/0.5×/1×/2×/4× (smoothly ramped) |
| Pot 3 (button) | Reset rate to 1× |
| Pot 6 (button) | Global reset — every channel back to 1 page / 16 steps |
| **Step button (1–16)** | Toggle that step (edit the pattern from the Pages menu) |
| **Function + step 1–4** | Jump straight to that page (auto-extends page count) |

Growing the page count duplicates page 1 into the new slots so new pages aren't empty.

---

## Modifier combos cheat-sheet

| Combo | Effect |
|---|---|
| **Function + Menu 4** | Play / Stop |
| **Function + Pot 1 turn** | BPM ± 1 with full-screen splash |
| **Function + Pot 3 turn** (Notes) | Melodic contour bias (ascending / descending) |
| **Function + CHn** | Mute / unmute channel n |
| **Function + Fill** (held) | Slide-all on the active channel (purple LED wave) |
| **Function + Page** (held) | Accent-all on the active channel (red LED flash) |
| **Function + Page + Fill** (held 1 s) | Clear the active channel |
| **Function × 5 (taps)** | Save patch to EEPROM |
| **Function tap with step held** | Toggle that step's Fill state |
| **CHn held** | OLED focus view for that channel (mute + MIDI Out ch) |
| **CHn held + Pot 1 turn** | Change that channel's MIDI Out channel (1–16) |
| **CHn held + Pot 2 turn** | Change that channel's root note |
| **Fill held (alone)** | Global performance fill — fill-only steps trigger, anti-fill steps drop out |

---

## Gate lengths

11 divisions, in ticks (24 PPQN — one 16th step = 6 ticks):

| Index | Name | Ticks |
|---|---|---|
| 0 | 1 (whole) | 96 |
| 1 | 3/4 | 72 |
| 2 | 1/2 | 48 |
| 3 | 3/8 | 36 |
| 4 | 1/4 | 24 |
| 5 | 3/16 | 18 |
| 6 | 1/8 | 12 |
| 7 | 3/32 | 9 |
| 8 | 1/16 (default) | 6 |
| 9 | 1/24 | 4 |
| 10 | 1/32 | 3 |

Slide steps automatically extend their gate past the step boundary so the next note gets portamento on synths that support it. Non-slide steps end one tick early to leave room for envelope release.

---

## Save / load

EEPROM layout is versioned (current = **v11**, magic `13572477`). On boot the firmware checks the magic and loads everything; a mismatch (e.g. after a version bump) starts blank with defaults.

Saved per channel: pattern + per-step pitches/velocities/slides/ratchets/lengths/fill states (all pages), mute, Euclid params, channel pitch/velocity, MIDI Out channel, generative slide probability, octave spread, trigger machine type + density + shift, machine overlay, kick live params, **number of pages**, and **pattern step count**.

> Live performance toggles — random velocity, random gate, and the contour bias — are intentionally **not** persisted.

**To persist your work:** tap **Function 5 times in a row**. Without that, changes are lost on reset. To wipe EEPROM: send `c` in the serial monitor.

---

## Serial commands (USB)

Send these at 115200 baud:

| Command | Action |
|---|---|
| `t` | Run 10 s switch test — prints matrix index when a button is pressed/released |
| `e` | Run 10 s pot-button test |
| `r` | Print raw pot readings continuously (encoder/quadrature debug) |
| `m` | 2 s MIDI pin monitor |
| `d` | Cycle step division (whole / half / quarter / 8th / 16th) |
| `p` | Play a test note C3 on selected channel |
| `c` | Clear saved EEPROM state |
| `g` | Dump full diagnostic state (channels, steps, params) |

You'll also see live MIDI logs in the monitor: `NOTE_ON …`, `NOTE_OFF …`, `TRANSPORT PLAY/STOP`, `CONTOUR CH…`, etc.

---

## Visual feedback (OLED splashes)

| Splash | Trigger | Duration |
|---|---|---|
| Spiral boot animation (mirrored on screen 2) + centred boot card | Power-on | ~1.8 s spiral, then card held 3 s |
| `PLAY` / `STOP` (inverted full-screen) | Function + Menu 4 | 900 ms |
| `BPM nnn` (full-screen) | Function + Pot 1 | 1.5 s (refreshes per tick) |
| `CH n  MIDI OUT ch n` | CHn held | While held |
| `MUTE` / `ON` (bordered) | Function + CHn | 600 ms |
| `STEP n FILL/NORM` (bordered) | Function tap with step held | 700 ms |
| `CLEAR CH n` (inverted) | Function + Page + Fill (1 s hold) | 700 ms |
| `SAVED!` (large text) | Function × 5 taps | 600 ms |

---

## Defaults at first boot

- BPM: 200
- Channel pitch: **A1** (MIDI note 33, matches Rytm MK2 default trig note)
- Channel velocity: **100** for every channel
- MIDI Out: `CH n → MIDI channel n`
- Gate length: 1/16
- Scale (when gen mode enabled): Major
- Trigger machine: OFF
- Pages: 1 page, 16 steps, 1× rate
- Boots into the Notes page

---

## Pin mapping (full reference)

See [include/SeqConfig.h](include/SeqConfig.h) for the authoritative list. Summary:

```
MATRIX_ROW_PINS[5]    = {6, 7, 8, 9, 10}   // inputs
MATRIX_COL_PINS[6]    = {5, 4, 3, 2, 1, 0} // outputs, scan
POT_A_PINS[6]         = {38, 40, 14, 22, 24, 26}
POT_B_PINS[6]         = {39, 41, 15, 23, 25, 27}
POT_BTN_PINS[6]       = {36, 35, 34, 33, 30, 31}
OLED1_SDA_PIN         = 19   // Wire  (primary)
OLED1_SCL_PIN         = 18
OLED2_SDA_PIN         = 17   // Wire1 (secondary, optional)
OLED2_SCL_PIN         = 16
LED_PIN               = 29   // WS2812 chain, 30 LEDs
MIDI_TX_PIN           = 20   // Serial5 TX
MIDI_RX_PIN           = 21   // Serial5 RX
```

---

## Troubleshooting

| Symptom | Check |
|---|---|
| Target doesn't trigger | Verify the channel's MIDI Out (`CHn held + Pot 1`) matches the receiver. Default trig note is **A1 (33)**. |
| A button does the wrong thing | Send `t`, press it, note the index, and compare against the matrix map / `include/SeqConfig.h`. |
| Patch doesn't survive reboot | You must tap Function × 5 to save. EEPROM version bumps reset to defaults. |
| Second screen blank | The secondary OLED is optional and auto-detected on Wire1 (pins 17/16). The unit runs fine without it. |
| External clock drifts | The firmware re-locks BPM from a rolling timestamp window; sustained drift means the external master is unstable. |

---

## Contributing

This is a private prototype. To contribute or report bugs, open an issue or contact the author.

## License

See project root for any license files. No formal release license is guaranteed for this prototype.

---

_This project is actively under development — expect breaking changes._
