# Kalico TMC4671 AWD Synchronization

## Overview
The **Kalico TMC4671 AWD Synchronization** module is a high-performance, hardware-level extension for Klipper/Kalico, designed specifically for Ouroboros boards driving All-Wheel Drive (AWD) CoreXY and Cartesian 3D printers. 

In traditional AWD setups, the host software (Klipper) must calculate and send identical step/torque commands to two separate motor drivers across a USB/CAN bus. Due to micro-timing differences, bus latency, and independent driver clock drifts, the two motors mechanically fighting each other is inevitable. This results in resonance, loss of torque, and increased heat.

The **Kalico AWD Sync** module solves this by moving the synchronization off the host and down to the lowest possible level: the MCU hardware SPI bus.

## How It Works: Rigid-Coupling Synchronization
Instead of treating the AWD motors as two independent entities, this architecture establishes a strict **Leader-Follower (Master-Slave)** relationship directly on the MCU.

1. **Host Configuration:** The Python host module (`tmc4671_sync.py`) links the two drivers together during initialization and commands the Follower to permanently enter purely closed-loop Torque mode (`MODE_MOTION = 1`).
2. **C-Level Firmware Loop:** A dedicated hardware timer task (`tmc4671_sync_task`) runs in the background on the MCU at a high frequency (e.g., 2000Hz).
3. **SPI Interception:** 
   * The MCU reads the true, instantaneous `PID_TORQUE_FLUX_TARGET` (Register `0x64`) directly from the Leader driver's silicon.
   * It immediately writes this identical 32-bit value to the Follower driver's `PID_TORQUE_FLUX_TARGET` register.

By mirroring the exact target commands across the SPI bus in under a millisecond, the Follower acts as a direct current amplifier. Both motors are commanded to exert identical force simultaneously. While this eliminates the dominant source of mechanical fighting (two independent position loops targeting the same point), tiny residual mismatches from physical motor variance or FOC phase alignment quality will still exist and are typically absorbed by belt compliance. This bypasses all host-to-MCU communication latency and drastically reduces overall system resonance.

**⚠️ Critical FOC Requirement:** The sync module *only* mirrors the torque magnitude. It does not mirror the commutation angle. Therefore, the Follower must have its own independent encoder (or Hall sensors) wired up, and its `[tmc4671]` config block must be fully calibrated (`PHI_E_SELECTION`, `ABN_DECODER_PHI_E_PHI_M_OFFSET`, etc.) just like the Leader. If the Follower's FOC doesn't know its own rotor angle, commanding torque will produce wrong-direction current.

## Key Technical Features

### 1. SPI Split-Transfer Read Timing
The TMC4671 datasheet enforces a strict hardware requirement: when running the SPI bus at high speeds (up to 8MHz), any SPI read operation requires a mandatory 500ns pause *immediately after* the 1-byte address is transmitted. 

To satisfy this without stalling the rest of the MCU or slowing down generic SPI devices, Kalico implements a custom `spidev_transfer_tmc4671_read()` C function. It splits the transfer, injects a precise 500ns `timer_read_time()` delay after the address byte, and safely retrieves the remaining 4 bytes of telemetry.

### 2. State-Transition Safety Clamps
Because the sync loop runs continuously, there is a risk of mirroring glitched data if the Leader undergoes a state transition (e.g., during homing, configuration, or emergency stops). 

To prevent the Follower from violently reacting to stale limits during transitions, the firmware safely intercepts the forward path. It executes a sequential read of the Leader's `MODE_RAMP_MODE_MOTION` (0x63) register immediately *after* fetching the target. By reading the mode second, the firmware guarantees that the mode was valid at the exact moment the target was captured, completely closing any race condition window. If the Leader is **not** actively in a closed-loop motion mode (Torque, Velocity, or Position), the C-loop forces the Follower's target torque and flux to exactly `0`, safely cutting all power.

### 3. Hardware Torque Divergence Detection
If the Follower loses mechanical tracking (e.g., a phase wire disconnects, or the encoder slips), the Leader will dynamically ramp up its positional PID torque output to compensate, dragging the disabled Follower. To prevent this catastrophic failure, the C-loop constantly compares `PID_TORQUE_FLUX_ACTUAL` (0x69) between both drivers. If the instantaneous physical torque diverges by more than the threshold for a set duration, the MCU will instantly throw a hardware `AWD Torque Divergence Fault` shutdown.

**Config Parameters:**
* `divergence_threshold`: The allowable mismatch in raw ADC current units before flagging a fault (Default: `500`). *Note: This is set loosely by default. Even perfectly matched motors will exhibit transient divergence during heavy acceleration ramps due to micro-variations in rotor inertia and flux alignment. You may need to tune this to avoid false-positives while maintaining safety.*
* `divergence_time`: The duration the mismatch must persist continuously before triggering the MCU shutdown (Default: `0.05` seconds).

### 4. Asynchronous Telemetry & Host Controls
The synchronization loop exposes full diagnostic telemetry to the Kalico host without blocking the real-time C loop:
* **`DUMP_SYNC_<NAME>`**: Provides real-time metrics including cycle counts, overruns, the last forwarded target, and tracks both the recent `max_latency` and the `absolute_max_latency` (peak jitter) since boot.
* **`SYNC_STOP_<NAME>` / `SYNC_START_<NAME>`**: Dynamic G-Code commands allowing macros to pause the hardware synchronization cleanly during complex homing maneuvers or sensorless probing.

## Project Structure
* **`klippy/extras/tmc4671_sync.py`**: The Klipper Python module responsible for parsing the `printer.cfg`, validating OIDs, and scheduling the C-level sync task.
* **`src/tmc4671_sync.c`**: The MCU C-module containing the high-frequency hardware timer and SPI mirroring logic.
* **`src/spicmds.c` / `spicmds.h`**: Modified Klipper hardware SPI drivers containing the 500ns split-transfer implementation for TMC4671 reads.
* **`klippy/extras/tmc4671.py`**: The core TMC4671 host driver, updated with SPI accessors and simulator bypasses for development.
