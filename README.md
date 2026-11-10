# Motion Rig — Slider + Pan Controller

Firmware for a two-axis camera motion rig: a **linear slider (X)** with end-stop switches, and a **pan head (Y)**. You control everything with one rotary encoder and a 16×4 I²C LCD. You can jog the rig to positions, save them as keyframes with timings, and play the move back smoothly, once or on a loop.

- **MCU:** ESP32-C3 SuperMini
- **Framework:** Arduino on PlatformIO (`espressif32`)
- **Source:** all firmware is in [src/main.cpp](src/main.cpp)

---

## Contents

1. [Hardware](#hardware)
2. [Wiring / pin map](#wiring--pin-map)
3. [Building and flashing](#building-and-flashing)
4. [First power-up checklist](#first-power-up-checklist)
5. [Controls](#controls)
6. [Menu reference](#menu-reference)
7. [Typical workflow](#typical-workflow)
8. [How it works](#how-it-works)
9. [Configuration constants](#configuration-constants)
10. [Persistent storage](#persistent-storage)
11. [Troubleshooting](#troubleshooting)
12. [Known limitations](#known-limitations)

---

## Hardware

| Part | Notes |
|---|---|
| ESP32-C3 SuperMini | Programmed and powered over its native USB-C (USB-Serial-JTAG). |
| CNC shield + 2 stepper drivers | A4988/DRV8825-style STEP/DIR drivers. Microstepping set to **16×**. |
| Stepper X (slider) | 200 steps/rev, 20-tooth GT2 pulley → 40 mm per rev → **80 steps/mm**. |
| Stepper Y (pan) | 200 steps/rev, 16× microstep, **4:1 reduction** → 12 800 steps per 360°. |
| 2 × limit switches | MIN and MAX ends of the slider. 3-wire modules or bare switches to GND. |
| KY-040 rotary encoder | With push button. |
| 16×4 HD44780 LCD + I²C backpack | Auto-detected at `0x27` or `0x3F`. |

If your mechanics are different (pulley, gearing, microstepping), change the constants in [Configuration constants](#configuration-constants).

---

## Wiring / pin map

| Function | GPIO | Notes |
|---|---|---|
| X STEP | 0 | Slider driver |
| X DIR | 1 | |
| Y STEP | 3 | Pan driver |
| Y DIR | 4 | |
| Encoder CLK (A) | 7 | Internal pull-up, interrupt |
| Encoder DT (B) | 10 | Internal pull-up, interrupt |
| Encoder SW | 2 | Internal pull-up, active LOW |
| I²C SDA | 8 | LCD |
| I²C SCL | 9 | LCD |
| Limit MIN | 20 | Internal pull-up, active LOW by default |
| Limit MAX | 21 | Internal pull-up, active LOW by default |

### Wiring notes

- **Common ground.** The ESP32-C3, CNC shield logic, encoder, LCD and switches must all share GND.
- **Driver enable.** The firmware has no ENABLE pin. The drivers' EN line must be held at its *enabled* level (usually LOW) by the shield or by a jumper to GND, or the motors won't hold or move.
- **Strapping pins.** GPIO2, GPIO8 and GPIO9 are ESP32-C3 boot-strapping pins. GPIO9 is also the SuperMini's BOOT button.
  - Don't hold the encoder button down while the board powers up or resets.
  - Anything on GPIO8/GPIO9 must not pull them LOW at boot. Standard I²C pull-ups are fine.
- **GPIO20/21** are the C3's UART0 pins. They're free for the limit switches because serial output goes over USB instead (see `ARDUINO_USB_CDC_ON_BOOT` below).

---

## Building and flashing

### `platformio.ini`

```ini
[env:esp32-c3-devkitm-1]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
upload_port = COM5
monitor_port = COM5
lib_deps =
    waspinator/AccelStepper@^1.64
    marcoschwartz/LiquidCrystal_I2C@^1.1.4

build_flags =
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
```

**Why the two USB flags matter.** The SuperMini has no USB-UART chip. Its USB-C port goes straight to the C3's built-in USB-Serial-JTAG peripheral.

- `ARDUINO_USB_CDC_ON_BOOT=1` sends `Serial` to that USB port.
- `ARDUINO_USB_MODE=1` picks the hardware CDC (`HWCDC`) driver. The C3 has no USB-OTG peripheral, so without this flag the core never declares `Serial`. The build then fails with `'Serial' was not declared in this scope`.

**Port.** `upload_port`/`monitor_port` are fixed to `COM5`, the port Windows gave the board. If your board comes up as a different COM port, change both lines. Delete them to let PlatformIO auto-detect.

### Commands

```bash
pio run                 # build
pio run -t upload       # build + flash
pio device monitor      # serial monitor @115200
pio run -t clean        # wipe build artefacts
```

> The `.pio/build/supermini` and `.pio/libdeps/supermini` folders are left over from an older env name. They do no harm and you can delete them.

### Entering the bootloader manually

If the upload can't find or open the port, the running firmware may be blocking USB. Force the bootloader:

1. Hold **BOOT**.
2. Tap **RESET**.
3. Release **BOOT** after about a second.

The board should now enumerate as a USB serial device (Espressif VID `303A`). Retry the upload.

---

## First power-up checklist

Do these in order before you run a real move:

1. **LCD.** The splash screen should read `MOTION RIG / Slider + Pan / Home before use / KF stored: N`. If the screen is blank, see [Troubleshooting](#lcd-shows-nothing).
2. **Diagnostics** (`Main menu → Diagnostics`):
   - `MIN:0 MAX:0` with nothing pressed. Press each switch by hand: its value should go to `1`. If the values are inverted, set `LIMIT_ACTIVE_LEVEL` to `HIGH`.
   - Turn the encoder. `Enc` should change by **1 per click**. If it changes by 2 per click or skips, set `ENC_TRANSITIONS_PER_DETENT` to `2`. If it counts the wrong way, set `ENC_INVERT`.
3. **Directions.** In `Jog / Record`, turn clockwise in small steps. The slider should move **away from MIN**, and the pan should rotate the way you think of as positive. Fix either with `X_DIR_INVERT` / `Y_DIR_INVERT`.
4. **Home.** `Home / Zero → Home+measure`. The slider should go to MIN, back off, travel to MAX, back off, and return to 0. Its usable length is then saved.
5. **Pan zero.** Point the pan head where you want 0° and pick `Home / Zero → Zero pan here`.

---

## Controls

There is one encoder with a push button. What each input does depends on the screen:

| Input | Menus | Jog / Record | KF time edit | Homing / Play / Diagnostics |
|---|---|---|---|---|
| Turn | Move cursor | Move active axis by step size | ±0.5 s | — |
| Hold + turn | Move cursor | Change step size | ±5 s | — |
| Click | Select | Toggle axis (slider ⇄ pan) | Save and return | Abort / stop / exit |
| Double-click | — | **Save keyframe here** | — | — |
| Long press (≥ 0.7 s) | Back | Back to main menu | Save and back | Abort / stop / exit |

Double-click only works on the Jog screen. Everywhere else a click responds immediately, with no double-click delay.

### Screen indicators

- **Toast messages** (e.g. `KF 3 SAVED`, `HOME FIRST`, `LIMIT MIN`) replace the bottom line for 1.5 s.
- **Limit indicator.** The top-right character of every screen shows `<` while MIN is pressed, `>` while MAX is pressed, and `#` when both are pressed (a wiring fault).

---

## Menu reference

```
MAIN MENU
├── Jog / Record
├── Play sequence
├── Keyframes
│   ├── 1: <x>mm <y>°  →  Go to | Set time | Overwrite pos | Delete | Back
│   ├── 2: ...
│   └── Back
├── Home / Zero
│   ├── Home slider       (seek MIN, set X = 0)
│   ├── Home+measure      (seek MIN, then MAX, save travel length)
│   ├── Zero pan here     (current pan = 0°)
│   └── Back
├── Settings
│   ├── Loop: ON/OFF
│   ├── Ease: ON/OFF
│   ├── Speed: Slow/Med/Fast
│   ├── Clear keyframes   → confirm: No, keep / Yes, clear
│   └── Back
└── Diagnostics
```

### Jog / Record

```
JOG-SLD HOMED
>X    123.0 mm
 P     45.0 °
Step 2.0mm KF:3
```

- The `>` marks the active axis. Click to switch between slider (`SLD`) and pan (`PAN`).
- Step sizes: slider **0.5 / 2 / 10 / 50 mm**, pan **0.5 / 2 / 10 / 45°**.
- Turns add up: spinning fast queues several steps, and the axis accelerates to cover them.
- Double-click saves the current X/Y as a new keyframe. This requires the slider to be homed. Up to 20 keyframes.

### Keyframes

Each keyframe stores an X position, a Y position, and a **duration**: the number of seconds playback takes to travel *from the previous keyframe to this one*. Keyframe 1 is the start point and has no duration.

| Action | Effect |
|---|---|
| Go to | Moves the rig to that keyframe at the current Speed setting (requires homing). |
| Set time | Edits the duration (0.5–600 s, default 5 s). Not available on keyframe 1. |
| Overwrite pos | Replaces the stored position with where the rig is now. |
| Delete | Removes the keyframe. If you delete keyframe 1, the next keyframe becomes the start. |

Every change is saved to flash immediately.

### Settings

| Setting | Effect |
|---|---|
| **Loop** | Playback returns to keyframe 1 and repeats until you stop it. |
| **Ease** | Each segment eases in and out (smoothstep) instead of moving at constant speed. Peak speed is 1.5× the segment average. |
| **Speed** | Slow 25 % / Med 50 % / Fast 100 % of max speed. Applies to **jogging, Go to, and the move to the start of playback**. Timed playback ignores it, because the keyframe durations set the speed. |
| **Clear keyframes** | Deletes all keyframes, after a confirmation. |

### Diagnostics

```
MIN:0 MAX:0
CLK1 DT1 SW1
Enc 42
X1600 Y0
```

Shows the raw switch levels, the encoder pin states and running count, and both motor positions in steps. Any button press exits.

---

## Typical workflow

1. Power on, then **Home / Zero → Home slider**. Use **Home+measure** the first time, or after you change the rails.
2. **Zero pan here** to point the pan head at its reference.
3. **Jog / Record:** move to the start framing and double-click (KF 1).
4. Move to the next framing and double-click (KF 2). Repeat for as many as you need.
5. **Keyframes → KF n → Set time** to set how long each segment takes.
6. Optional: **Settings**, to turn Ease and/or Loop on.
7. **Play sequence.** The rig drives to KF 1, then runs the timed move. Press the button at any time to stop.

---

## How it works

### Main loop

`loop()` runs these steps and never blocks:

1. `motionRun()` checks the limit switches, then advances homing, playback, or normal point-to-point moves.
2. `pollInput()` debounces the button, works out click / double / long press, and reads the encoder.
3. `handleInput()` passes the input to the current screen.
4. `draw()` redraws the screen buffer every 50 ms (idle) or every 200 ms (while moving).
5. `lcdFlush()` sends only the characters that changed. It is limited to **2 I²C writes per loop while moving** and 16 while idle, so the LCD never delays step pulses.

### Encoder

Both encoder pins trigger an interrupt on every edge. A quadrature lookup table decodes the direction, which rejects contact bounce. Counts accumulate in an ISR-safe accumulator and become whole detents (`ENC_TRANSITIONS_PER_DETENT`). Turning while the button is held is reported separately (`rotHeld`) and stops the press from counting as a click.

### Homing (X only)

This is a non-blocking state machine running at `X_HOME_SPEED` (1000 steps/s ≈ 12.5 mm/s):

```
SEEK_MIN → BACKOFF_MIN (3 mm, until switch releases) → X = 0
   └─ measure only: SEEK_MAX → BACKOFF_MAX → save xMax = trip − 3 mm → RETURN to 0
```

- If both switches read as pressed at the start, it refuses to run (`LIMIT WIRING?`).
- It times out after 120 s (`HOME TIMEOUT`).
- Any button press aborts (`HOMING ABORT`).

### Soft and hard limits

- **Soft limits.** Once homed, every X target is clamped to ≥ 0. After a *measured* home, X is also clamped to ≤ `xMax`.
- **Hard limits** (`guardLimits()`). If a switch is pressed while X is moving towards it, X stops instantly and you see `LIMIT MIN`/`LIMIT MAX`. Playback stops too. You can still move *away* from a pressed switch.

### Playback

1. **Pre-check.** At least 2 keyframes and a homed slider are required. For each segment, the firmware works out the speed needed (×1.5 when Ease is on). If any segment needs more than 90 % of `X_VMAX`/`Y_VMAX`, playback refuses to start and shows `SEG n TOO FAST`. Give that segment more time.
2. **GOTO.** A normal accelerated move to keyframe 1.
3. **RUN.** Time-based tracking. For the current time *t*, the firmware computes where each axis *should* be (linear, or smoothstep with Ease). It then sets each motor's speed to

   `feed-forward velocity + PLAY_KP × (ideal position − actual position)`

   capped at the axis maximum. This keeps both axes in step with the clock and with each other over long moves. For the first second (`PLAY_RAMP_MS`), speed builds up gradually so the start doesn't jerk.
4. **End.** The rig stops with `DONE`. With Loop on, it goes back to keyframe 1 and repeats.

### LCD

At boot the firmware scans I²C for `0x27`, then `0x3F`. If it finds neither, it runs **headless**: everything still works and `No LCD found - running headless` is printed on serial. Row addresses use 16×4 offsets (`0x00, 0x40, 0x10, 0x50`).

---

## Configuration constants

All of these are at the top of [src/main.cpp](src/main.cpp):

| Constant | Default | Meaning |
|---|---|---|
| `LCD_COLS` / `LCD_ROWS` | 16 / 4 | Display size. |
| `LCD_ROW_OFFSET` | `{0x00,0x40,0x10,0x50}` | DDRAM row addresses for 16×4 panels. |
| `LIMIT_ACTIVE_LEVEL` | `LOW` | Pin level that means "switch pressed". |
| `X_DIR_INVERT` / `Y_DIR_INVERT` | `false` | Reverse motor direction. |
| `ENC_INVERT` | `false` | Reverse encoder direction. |
| `ENC_TRANSITIONS_PER_DETENT` | 4 | Encoder edges per click (try 2 if clicks skip). |
| `X_STEPS_PER_MM` | 80 | Slider steps per mm. |
| `Y_STEPS_PER_DEG` | 12800/360 ≈ 35.56 | Pan steps per degree. |
| `X_VMAX` / `Y_VMAX` | 3000 steps/s | Max speed (≈ 37.5 mm/s, ≈ 84 °/s). |
| `X_ACC` / `Y_ACC` | 6000 steps/s² | Acceleration for jog/goto moves. |
| `SPEED_PCT` | 25 % / 50 % / 100 % | Speed setting presets. |
| `X_HOME_SPEED` | 1000 steps/s | Homing speed. |
| `X_BACKOFF_STEPS` | 240 (3 mm) | Distance to back off a switch. |
| `X_HOME_TIMEOUT_MS` | 120 000 | Homing timeout. |
| `MAX_KF` | 20 | Keyframe slots. |
| `DEFAULT_KF_SECONDS` | 5.0 | Duration given to new keyframes. |
| `PLAY_KP` | 6.0 | Playback position-correction gain. |
| `PLAY_RAMP_MS` | 1000 | Soft-start time for playback. |
| `X_STEP_MM` / `Y_STEP_DEG` | see Jog section | Jog step-size presets. |
| `DEBOUNCE_MS` / `LONG_MS` / `DOUBLE_MS` | 25 / 700 / 280 | Button timing. |

---

## Persistent storage

Settings are stored in ESP32 NVS (flash) under the namespace `slider`:

| Key | Contents |
|---|---|
| `kfn` | Number of keyframes. |
| `kf` | Raw keyframe array (`x`, `y` in steps, `dur` in seconds). |
| `cfg` | Loop / Ease / Speed. |
| `xcal` / `xmax` | Whether travel has been measured, and its length in steps. |

The **homed state and current positions are not saved**. After every power-up, X and Y both read 0, and the slider must be homed again before you can record, use Go to, or play.

Keyframes are stored in **steps**, not mm/degrees. If you change `X_STEPS_PER_MM`, `Y_STEPS_PER_DEG` or the microstepping, re-record or clear existing keyframes.

---

## Troubleshooting

### Build: `'Serial' was not declared in this scope`
`-D ARDUINO_USB_MODE=1` is missing from `build_flags`. See [Building and flashing](#building-and-flashing).

### Upload: `Please specify upload_port`
PlatformIO couldn't find the board. Set `upload_port` in `platformio.ini`, or pass `--upload-port COMx`.

### Upload: `Could not open COM5, the port is busy or doesn't exist`
Windows can't see the board, or something else has the port open.

1. Close any serial monitor that has the port open.
2. Run `pio device list`, or check Device Manager → *Ports (COM & LPT)*. Look for a device with VID `303A`.
3. Enter the bootloader manually (BOOT + RESET) and retry.
4. Swap the USB-C cable. Charge-only cables are the most common cause. Plug straight into the PC, not a hub.
5. Unplug the shield and peripherals, then retry. A short, or a strapping pin (GPIO2/8/9) held LOW, can stop the board from booting.
6. If the board comes back on a different COM number, update `upload_port`/`monitor_port`.

### LCD shows nothing
- Check the serial monitor for `No LCD found - running headless`. If it's there, check SDA = GPIO8, SCL = GPIO9, power and GND.
- If the backlight is on but no text shows, turn the contrast trimmer on the I²C backpack.
- If text appears on the wrong lines, your panel uses different row offsets. Adjust `LCD_ROW_OFFSET`.

### Messages and what to do

| Message | Meaning / fix |
|---|---|
| `HOME FIRST` | Home the slider before recording, Go to, or Play. |
| `NEED 2+ KEYFRMS` | Playback needs at least two keyframes. |
| `SEG n TOO FAST` | Segment *n* needs more speed than the motors allow. Increase its time. |
| `LIMIT MIN` / `LIMIT MAX` | X was stopped by a switch. Jog away from it. |
| `LIMIT WIRING?` | Both switches read as pressed. Check wiring or `LIMIT_ACTIVE_LEVEL`. |
| `HOME TIMEOUT` | No switch was found within 120 s. Check the switch and the X direction. |
| `KEYFRAMES FULL` | All 20 slots are in use. |
| `START: NO TIME` | Keyframe 1 has no duration to edit. |

### Motor moves the wrong way / homing runs away from MIN
Flip `X_DIR_INVERT`. Homing always moves in the negative direction first, so it has to drive *towards* the MIN switch.

### Motors whine but don't move, or skip steps
Lower `X_VMAX`/`Y_VMAX` or `X_ACC`/`Y_ACC`. Check the driver current setting and the motor supply voltage.

---

## Known limitations

- **No pan homing.** Y has no switch and no limits; its zero is set by hand and lost on reboot.
- **Homing isn't saved.** You must re-home after every power cycle.
- **Speed setting doesn't scale playback.** Keyframe durations set the speed of a timed move.
- **Keyframe format is raw binary.** If you change the `Keyframe` struct layout, old saved keyframes will load as garbage. Clear them after such a change.
- **Single-threaded stepping.** Step pulses come from `loop()`, so very high step rates depend on the loop running fast. The LCD write budget exists to protect this.
- **Hardware not yet verified on the bench.** The firmware builds, but it hasn't yet been flashed and checked on the rig. Work through the [First power-up checklist](#first-power-up-checklist) before you trust any of the constants above.
