# SARC — Smart Antenna Rotor Controller

**An Android + STM32 antenna rotor controller for amateur radio**  
**Project by YO6PIR** · Status: functional prototype / bench-tested

<p align="center">
<img width="1448" height="1086" alt="Controler rotor antenă inteligentă SARC" src="https://github.com/user-attachments/assets/9d6331b9-0846-4b7b-a21c-863a01810a7b" />


</p>

SARC provides a modern azimuth display and straightforward control of a directional antenna rotor. An **Android tablet** acts as the human–machine interface (HMI), while an **STM32 MASTER** measures the rotor position, drives the motor relays and manages safety, calibration and persistent state.

> **Design philosophy:** This is an antenna rotor, not a CNC machine. Reliable operation, clear feedback and sensible mechanical tolerances matter more than sub-degree positioning.

## Features

- **Azimuth control:** set a target on the tablet or with the MASTER's local rotary encoder and PUSH button.
- **Actual-position feedback:** a separate external A/B Hall encoder tracks physical rotor motion.
- **Manual operation:** hardware CW/CCW buttons drive the motor independently; MASTER observes the MANUAL signal and tracks the resulting position.
- **Operator-assisted HEAD/TAIL calibration:** accounts for mechanical end stops, backlash and small reversals. A calibration can be canceled and restarted.
- **Practical target stopping:** stops on target crossing without repeated CW/CCW hunting; small final deviations are acceptable.
- **Six local memories (M1–M6):** store and recall target headings on the Android tablet, including while offline. Recalling a heading does not move the rotor until confirmed.
- **USB CDC communication:** session handshake, synchronization, heartbeat, status messages and diagnostic log.
- **Flash persistence:** calibration and last confirmed position survive power loss when recovery conditions are valid.
- **BM — Bon de Mișcare (Movement Bond):** a persistent OPEN/CLOSED marker helps detect unfinished movement after a power interruption.
- **RF-conscious sensor isolation:** external Hall sensor lines can be disconnected when they are not needed.

## System architecture

```text
 Android tablet — SARC HMI
 (dial, target, memories, CAL, status, log)
               │
          USB OTG / CDC
               │
               ▼
 STM32F103CBT6 — MASTER
 ├── Local encoder + PUSH
 ├── Motor direction outputs ──► CW / CCW relays ──► 12 V geared motor
 ├── MANUAL sense ◄─────────── Hardware CW / CCW controls
 └── Sensor enable ──────────► External Hall sensor interface
                                │
                         Encoder A / B
                                │
                         Slotted rotor disc
```

**MASTER is the authority for actual antenna position.** The tablet may be disconnected or charged separately; if MASTER remains powered, it continues tracking manual movement and sends the updated position when the tablet reconnects.

## Hardware

| Subsystem | Implementation |
|---|---|
| Main controller | **STM32F103CBT6**, 128 KB Flash / 20 KB RAM |
| Tablet connection | USB-C connector, native USB CDC device; Android USB OTG host |
| Position feedback | Two external Hall sensors, A/B incremental signals, STM32 **TIM3** |
| Local input | Rotary encoder with PUSH, **TIM2** |
| Motor | 12 V DC motor with geared reduction |
| Motor switching | CW / CCW relays |
| Manual indication | **PB13**, active HIGH |
| External sensor enable | **PB12**, active HIGH |
| CW / CCW relay control | **PB10 / PB11** |
| External sensor interface | Relay isolation and **PC817** optocouplers (see schematic) |

The schematic includes the power circuitry, motor reversing relays, manual controls, Hall interface and encoder disc concept. The encoder disc drawing illustrates **32 filled and 32 open sectors**; the software uses the *measured* travel count from calibration rather than assuming a fixed count.

### Electrical schematic

<img width="3508" height="2480" alt="SARC_sch" src="https://github.com/user-attachments/assets/98ba3c1a-77d8-4579-bf36-f8c73214d6b9" />

**Hardware note:** Verify relay wiring, motor current, power dissipation, isolation, limit switches and RF protection for your particular rotor before connecting the real motor. The repository schematic is a project reference, not a certified commercial design.

## Operation

**NORMAL** — Select a desired azimuth with the tablet or MASTER encoder, then confirm with SYNC / PUSH. MASTER controls direction and stops when the target is reached or crossed; no repeated correction for a few degrees of residual mechanical error.

**MANUAL** — Physical CW/CCW controls take priority over automatic movement. MASTER tracks motion through the external encoder and updates the actual azimuth. A new automatic command must be selected after manual intervention.

**CALIBRATION** — The operator moves the antenna manually toward the **HEAD (10°)** and **TAIL (40°)** mechanical references along the permitted **330°** travel, then confirms each position. Small reverse corrections and mechanical rebound are allowed. MASTER calculates the measured counts-per-degree factor; the operator can cancel and restart calibration.

## Persistence and power recovery

SARC uses internal STM32 Flash to store calibration data and the last valid stable position. The **BM (Bon de Mișcare)** flag records whether a movement transaction has completed:

| BM state | Meaning | Recovery behavior |
|---|---|---|
| `OPEN` (`1`) | Motion or calibration not yet confirmed complete | Require recalibration after a restart |
| `CLOSED` (`0`) | Completed operation with stored stable position | Restore position if calibration and stored data are valid |
| Invalid / missing | Position cannot be trusted | Require calibration |

BM is opened before MASTER-controlled movement and closed only after successful completion and stabilization. Consecutive manual adjustments can share one open BM rather than writing Flash for every encoder transition. **No automatic movement is resumed after an interrupted operation.**

Recovery assumes the worm-geared rotor remains mechanically stationary while power is off. Movement that occurs when MASTER and its sensors have no power cannot be measured by an incremental encoder.

## Communication and diagnostics

The tablet and MASTER exchange human-readable messages over USB CDC, protected by **CRC-16/CCITT-FALSE**, with session identifiers and command correlation where applicable. The protocol supports IDENTIFY / SESSION, SYNC, READY, AZIMUTH, CAL, heartbeat and BM notifications.

The HMI provides a status line and diagnostic log. After reconnection, the displayed **ACTUAL** azimuth comes from MASTER—not from a stale tablet value.

## Repository layout

```text
SARC/
├── MASTER/            # STM32 firmware and build configuration
├── STM32_HMI/         # Android tablet application
├── docs/
│   ├── sarc_hmi.png        # Add a screenshot of the Android HMI
│   ├── sarc_schematic.png  # Add the electrical schematic
│   └── sarc_rotor.jpg      # Add a rotor / encoder photograph
└── README.md
```

**Development:** STM32 firmware is built with the STM32 Arduino core / Arduino CLI and uploaded through ST-Link. The Android HMI is built in Android Studio. Use the project-specific build settings in each folder.

## Project status

The communication, calibration, target control, manual tracking, BM persistence and reconnect/recovery behavior have been exercised on the development setup. **Final operation with the installed rotor, motor, relays and mechanical end stops must be verified on the real hardware.**

---

**73 de YO6PIR!** 📡  
*SARC — practical antenna positioning, with the MASTER in control.*
## Download SARC for Android

The SARC Android application is available as a precompiled APK.

**[Download the latest SARC release](../../releases/latest)**

- Designed for Android 6.0 and compatible devices.
- Requires USB OTG support.
- Requires a compatible STM32 SARC MASTER controller for rotor operation.
- No Google Play installation required.

Download the APK, transfer it to your Android device, and install it. Android may ask you to allow installation from this source.

**Note:** Download SARC only from the official GitHub repository.
