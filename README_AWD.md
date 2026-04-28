# Kalico TMC4671 AWD Synchronization

## Overview
The **Kalico TMC4671 AWD Synchronization** module is a high-performance, hardware-level extension for Klipper/Kalico, designed specifically for Ouroboros boards driving All-Wheel Drive (AWD) CoreXY and Cartesian 3D printers.

In traditional AWD setups, the host software (Klipper) must calculate and send identical step/torque commands to two separate motor drivers across a USB/CAN bus. Due to micro-timing differences, bus latency, and independent driver clock drifts, mechanical fighting between the two motors is inevitable. This results in resonance, loss of torque, and increased heat.

The **Kalico AWD Sync** module solves this by moving the synchronization off the host and down to the lowest possible level: a dedicated MCU hardware timer interrupt (`TIM6_DAC_IRQHandler`) that drives the SPI bus directly. The current implementation has been validated under live step load to **10 kHz sustained operation** (3.5 M+ ISR fires, zero overruns) on STM32H723.

## How It Works: Rigid-Coupling Synchronization
Instead of treating the AWD motors as two independent entities, this architecture establishes a strict **Leader-Follower (Master-Slave)** relationship directly on the MCU.

1. **Host Configuration:** The Python host module (`tmc4671_sync.py`) links the two drivers together during initialization and commands the Follower to permanently enter purely closed-loop Torque mode (`MODE_MOTION = 1`).
2. **Hardware Timer ISR:** A dedicated STM32 hardware timer (`TIM6`) drives an interrupt service routine (`TIM6_DAC_IRQHandler` in `src/tmc4671_sync.c`) at the configured `sync_rate`.
3. **Two control loops share one ISR:**
   * **Torque mirror loop** runs *every* TIM6 fire. It reads `PID_TORQUE_FLUX_TARGET` (Register `0x64`) from the Leader and writes the same 32-bit value to the Follower's `PID_TORQUE_FLUX_TARGET`, plus reads `MODE_RAMP_MODE_MOTION` (`0x63`) for the safety clamp described below.
   * **Trajectory loop** runs every Nth TIM6 fire (decimation factor `N = sync_rate / trajectory_rate`). It pushes Klipper's commanded position into the Leader's `PID_POSITION_TARGET` (`0x68`), reads back `PID_POSITION_ACTUAL` (`0x6B`) to compute the instantaneous following error, and triggers an E-stop if that error exceeds the safety threshold.

By mirroring the exact target commands across the SPI bus on every cycle, the Follower acts as a direct current amplifier. Both motors are commanded to exert identical force simultaneously. While this eliminates the dominant source of mechanical fighting (two independent position loops chasing the same point), tiny residual mismatches from physical motor variance or FOC phase alignment quality will still exist and are typically absorbed by belt compliance.

**⚠️ Critical FOC Requirement:** The sync module *only* mirrors the torque magnitude. It does not mirror the commutation angle. Therefore, the Follower must have its own independent encoder (or Hall sensors) wired up, and its `[tmc4671]` config block must be fully calibrated (`PHI_E_SELECTION`, `ABN_DECODER_PHI_E_PHI_M_OFFSET`, etc.) just like the Leader. If the Follower's FOC doesn't know its own rotor angle, commanding torque will produce wrong-direction current.

## Configuration

Minimum config block:

```ini
[kalico_follower_stepper stepper_x1]
step_pin = PC12
dir_pin = PC11
enable_pin = !PD1
microsteps = 16
rotation_distance = 40
kinematics = cartesian
axis = x

[tmc4671_sync awd_x_axis]
leader = stepper_x
follower = stepper_x1
follower_stepper = stepper_x1
sync_rate = 5000           # torque mirror rate (Hz). Default 2000.
trajectory_rate = 2500     # position setpoint refresh rate (Hz). Defaults to sync_rate.
```

### `sync_rate` and `trajectory_rate`

These are the two knobs that control CPU load. The torque mirror runs at `sync_rate`; the trajectory loop runs at `trajectory_rate`. Klipper validates that `trajectory_rate` divides `sync_rate` exactly and passes the decimation factor (`sync_rate / trajectory_rate`) to the MCU.

**Per-fire ISR cost** at 8 MHz SPI clock:

| Fire type | SPI transfers | Duration |
|---|---|---|
| Torque-only fire | 3 | ~28 μs |
| Torque + trajectory fire | 5 | Re-measure after firmware changes |

**Recommended values** (based on industry CNC and servo-drive practice for matched-motor master/slave torque sharing):

| Profile | `sync_rate` | `trajectory_rate` | Avg CPU | Notes |
|---|---|---|---|---|
| Conservative / efficient | 5000 | 1000 | Re-test | Plenty of headroom for future MCU work. Position-loop rate matches typical industrial CNC. |
| **Balanced (recommended)** | **5000** | **2500** | **Re-test** | Recommended first production profile after the 2026-04-28 safety-clamp cleanup. |
| Aggressive | 10000 | 5000 | Re-test | Previously validated stable before the every-fire mode clamp cleanup. Re-measure before treating this as production headroom. |
| **Hard ceiling** | 10000 | 5000 | Re-test | Do not exceed until fresh STM32H723 measurements confirm headroom. |

Why these numbers? The TMC4671 already runs a complete cascaded current/velocity/position controller internally on the chip at 25–100 kHz (PWM rate). Our two "loops" are *outer* setpoint refreshes feeding those internal controllers. Industrial CNC position loops sit at 1–4 kHz; master/slave torque-sharing rates sit at 1–4 kHz; both are well above the mechanical resonance bandwidth of any reasonable gantry. There is no benefit to pushing the configured rates higher than the recommended profile, and significant cost in CPU headroom.

## Key Technical Features

### 1. SPI Split-Transfer Read Timing
The TMC4671 datasheet enforces a strict hardware requirement: when running the SPI bus at high speeds (up to 8 MHz), any SPI read operation requires a mandatory 500 ns pause *immediately after* the 1-byte address is transmitted.

To satisfy this without stalling the rest of the MCU or slowing down generic SPI devices, Kalico implements a custom `spidev_transfer_tmc4671_read()` C function. It splits the transfer, injects a precise 500 ns `timer_read_time()` delay after the address byte, and safely retrieves the remaining 4 bytes of telemetry.

*(Note: Klipper's default SPI speed for the TMC4671 is 1 MHz. At 1 MHz, the natural inter-byte gap fulfills the 500 ns requirement without this delay. To actually utilize the high-speed 8 MHz bus and minimize sync latency, you must explicitly set `spi_speed: 8000000` in both your Leader and Follower `[tmc4671]` config blocks.)*

### 2. Mode-Second Safety Clamp
Because the sync loop runs continuously, there is a risk of mirroring glitched data if the Leader undergoes a state transition (e.g., during homing, configuration, or emergency stops).

The firmware closes this race by reading the Leader's `MODE_RAMP_MODE_MOTION` (`0x63`) register sequentially *after* fetching the target torque. By reading the mode second, the firmware guarantees that the mode was valid at the exact moment the target was captured. If the Leader is **not** actively in a closed-loop motion mode (Torque, Velocity, or Position), the C-loop forces the Follower's target torque and flux to exactly `0`, safely cutting all power. Two consecutive valid-mode samples are required before the Follower is rearmed, eliminating single-sample chatter.

### 3. Following-Error E-Stop
The trajectory loop computes `error = abs(commanded_position - actual_position)` from the Leader's encoder on every fire. If the error exceeds **16 384 encoder ticks**, the MCU triggers a shutdown:

```
shutdown ... static_string_id=AWD Following Error E-Stop Triggered
```

This protects against catastrophic loss of tracking — e.g., a phase wire disconnect, encoder fault, motor stall, or a coupler slip that would otherwise let the Leader's PID ramp torque indefinitely.

For the user's typical config (`rotation_distance = 40`, `foc_abn_decoder_ppr = 4000`), 16 384 ticks corresponds to about 163 mm of unmatched commanded travel. This is loose enough not to false-trigger during normal acceleration ramps but tight enough to catch real mechanical faults within a single high-speed move.

*A future safety-related work item is true torque-divergence detection — comparing `PID_TORQUE_FLUX_ACTUAL` between the two drivers and flagging persistent mismatches. This is not currently implemented; the following-error guard above is the sole runtime safety check.*

### 4. SPI Bus Concurrency & Isolation
The hardware implementation relies on Klipper's per-transaction SPI locking. The 3 (or 5, including trajectory) register transfers per cycle are individually atomic, but the full ISR sequence is not bus-locked.

* **Hardware Requirement**: The TMC4671 Leader and Follower drivers **must** be on an isolated SPI bus with no bandwidth-heavy peripherals (e.g., SD cards, LCDs). The Ouroboros hardware natively guarantees this.
* **Host Polling**: Host SPI commands (e.g., `DUMP_TMC`) are wrapped in `tmc4671_sync_pause` / `resume` MCU commands so they execute atomically between TIM6 fires. This adds a few microseconds of latency per host transaction. Avoid aggressive host polling during high-speed prints to maintain strict sub-millisecond synchronization.

### 5. Telemetry & Diagnostics
The synchronization loop exposes full diagnostic telemetry to the Kalico host without blocking the real-time C loop. Run `DUMP_SYNC_<NAME>` (or the `AWD_DUMP_STATS` macro) at any time:

```
TMC4671 Sync 'awd_x_axis' Stats:
Cycles: 3512019
Overruns: 0
Last Leader Torque Target: 0
Rates: torque=10000 Hz trajectory=5000 Hz (decim=2)
ISR duration: last=32.5us max=54.0us / budget=100.0us
```

Field-by-field:

| Field | Meaning |
|---|---|
| `Cycles` | Count of TIM6 ISR fires since `SYNC_START`. 32-bit; wraps after ~119 hours at 10 kHz. |
| `Overruns` | Frames dropped because Klipper or host-side code was already using the SPI bus when TIM6 fired. |
| `Last Leader Torque Target` | The 32-bit value most recently mirrored to the Follower. |
| `Rates: torque=... trajectory=... (decim=N)` | Confirms what the MCU is actually running. |
| `ISR duration: last / max / budget` | Per-fire ISR work in microseconds vs. the per-fire time window. **This is the canonical CPU-load metric — if `max` approaches `budget`, you are running too aggressive a `sync_rate`.** |

`SYNC_STOP_<NAME>` and `SYNC_START_<NAME>` give you dynamic G-code control to pause and resume the hardware sync — useful for complex homing maneuvers, sensorless probing, or tuning sessions.

## Stress Testing

A simple oscillation macro suitable for stress-testing the AWD path:

Note: the active Kalico driver now runs the TMC4671 alignment/tuning path by default. If you intentionally test with no motors physically attached, set `allow_no_motor_current: True` in each bench-only `[tmc4671 ...]` section. Do not use that option as a normal printer default; it bypasses a wiring/current sanity check.

```ini
[gcode_macro AWD_STRESS]
description: Bidirectional X stress test for AWD sync. Stays inside ±DIST mm.
gcode:
    {% set dist  = params.DIST|default(20)|float %}
    {% set feed  = params.FEED|default(18000)|int %}
    {% set count = params.COUNT|default(500)|int %}
    {% if dist > 100 %}
        { action_raise_error("DIST=%s exceeds 100mm safety cap" % dist) }
    {% endif %}
    SET_KINEMATIC_POSITION X=0
    RESPOND TYPE=echo MSG="AWD_STRESS: {count} cycles, ±{dist}mm @ {feed}mm/min"
    {% for i in range(count|int) %}
        G1 X{dist} F{feed}
        G1 X0 F{feed}
    {% endfor %}
    RESPOND TYPE=echo MSG="AWD_STRESS: complete"
```

Without a motor connected, the encoder will not track commanded motion, so the trajectory loop's following-error guard will trigger if commanded distance exceeds ~163 mm (assuming default `rotation_distance` and `foc_abn_decoder_ppr`). The macro caps `DIST` at 100 mm and uses an oscillating pattern that resets to zero between cycles, keeping accumulated error well clear of the safety threshold.

Suggested invocations:

```
AWD_STRESS                                # defaults: DIST=20, FEED=18000, COUNT=500
AWD_STRESS COUNT=2000                     # longer run
AWD_STRESS DIST=50 FEED=24000             # bigger, faster swings
AWD_STRESS DIST=10 FEED=36000 COUNT=5000  # short fast oscillations
```

While `AWD_STRESS` is running, watch `klippy.log` for retransmits, scheduler complaints, or AWD E-stops, and run `AWD_DUMP_STATS` after the run for the post-stress numbers. Pass criteria: zero overruns, `max` ISR duration ≤ 60 μs, `mcu_awake` near zero, no `Rescheduled timer in the past`.

## Project Structure
* **`klippy/extras/tmc4671_sync.py`** — Klipper Python module. Parses the `[tmc4671_sync ...]` config, validates OIDs, computes the trajectory decimation, and dispatches the C-level sync configuration / start / stop / pause / resume commands.
* **`klippy/extras/kalico_follower_stepper.py`** — Optional follower stepper wrapper. Registers the follower MCU stepper under its normal name while allowing `tmc4671_sync.py` to unlink its trapq during active sync.
* **`src/tmc4671_sync.c`** — MCU C-module. Owns `TIM6_DAC_IRQHandler` (the hardware-timer ISR), the torque mirror, the decimated trajectory loop, the mode-second safety clamp, the following-error E-stop, and the per-fire `timer_read_time()` instrumentation used by `DUMP_SYNC`.
* **`src/spicmds.c`** / **`spicmds.h`** — Modified Klipper hardware SPI drivers containing the 500 ns split-transfer implementation for TMC4671 reads, plus the global `kalico_spi_active` lock that prevents host-vs-ISR bus collisions.
* **`klippy/extras/tmc4671.py`** — Core TMC4671 host driver and source of truth for the AWD build. Supplies SPI accessors used by `tmc4671_sync.py`, keeps `get_status()` zero-SPI, and exposes live current reads only for explicit tuning/alignment paths.
* **`AGENT.md`** — Architectural rules and gotchas. **Read this before modifying anything in this module.**
* **`issues.md`** — Historical record of resolved issues with full diagnostic context. Useful when a regression looks similar to something we've already seen.
