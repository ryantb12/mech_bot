# mech_bot

3-board mechatronics robot — 2× LilyGO T-Display S3 (ESP32-S3) + Cytron MOTION 2350 Pro (RP2350).

---

## Quick Start — PlatformIO

Open this folder in VS Code with the PlatformIO extension, then flash each board independently:

```bash
# Competition firmware
pio run -e master  -t upload    # Master T-Display S3
pio run -e slave   -t upload    # Slave  T-Display S3
pio run -e motion  -t upload    # MOTION 2350 Pro

# Debug / phone-control panels
pio run -e master_panel -t upload
pio run -e slave_panel  -t upload
pio run -e motion_panel -t upload
```

Monitor serial output:
```bash
pio device monitor -e master
pio device monitor -e slave
```

---

## Hardware

| Board | Role | Env |
|---|---|---|
| LilyGO T-Display S3 #1 | Master — sensors, IMU, state machine, WiFi panel | `master` / `master_panel` |
| LilyGO T-Display S3 #2 | Slave — wheel motors, linear actuators | `slave` / `slave_panel` |
| Cytron MOTION 2350 Pro | Motion — servos, music, RGB LED | `motion` / `motion_panel` |

---

## Communications

### Master ↔ Slave (ESP-NOW, wireless)
No wire needed. Uses ESP-NOW peer-to-peer on the ESP32-S3 radio.

| Command byte | Action |
|---|---|
| `0x00` | State advance |
| `0x02` | Drive forward |
| `0x03` | Drive backward |
| `0x04` | Turn left |
| `0x05` | Turn right |
| `0x06` | Stop all |
| `0x07` | Actuator up |
| `0x08` | Actuator down |
| `0x09` | Actuator stop |

### Master → MOTION (GPIO pulse, GPIO 1 → GP26)
- Long pulse ≥ 60 ms = state advance
- Short burst of N pulses = servo command (2=extend … 8=neutral)

---

## MAC Addresses

Stored in `Context/mac_addresses.h` and referenced in `platformio.ini`:

| Board | MAC |
|---|---|
| Master | `30:30:F9:59:31:78` |
| Slave  | `30:30:F9:59:CE:E4` |

To find a board's MAC, flash any sketch and read Serial at 115200 baud — it prints on boot.

---

## Project Structure

```
mech_bot/
├── platformio.ini                        ← PlatformIO environments (all boards)
├── Context/
│   ├── mac_addresses.h                   ← ESP-NOW MAC addresses
│   ├── MAC_Address.md                    ← Human-readable MAC reference
│   ├── wiring_and_components.md          ← Full wiring table
│   └── CLAUDE.md                         ← AI assistant context
└── Code/
    ├── LilyGo T-Display s3/
    │   ├── master_controller_v2/         ← Competition master (env:master)
    │   ├── slave_controller_v2/          ← Competition slave  (env:slave)
    │   ├── master_control_panel_v2/      ← Phone panel master (env:master_panel)
    │   ├── slave_control_panel/          ← Phone panel slave  (env:slave_panel)
    │   └── master_unified_panel/         ← Unified panel (Lewis)
    └── Motion 2350 Pro_code/
        ├── motion_controller_v2/         ← Competition motion (env:motion)
        └── motion_control_panel/         ← Phone panel motion (env:motion_panel)
```

---

## Phone Control Panel

Flash `master_panel` to master, then:
1. Connect phone WiFi to `Robot-Master` (password: `robot1234`)
2. Open browser → `192.168.4.1`

Flash `slave_panel` to slave for independent slave control:
1. Connect phone WiFi to `Robot-Slave` (password: `robot1234`)
2. Open browser → `192.168.4.1`
