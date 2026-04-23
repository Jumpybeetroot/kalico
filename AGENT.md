# AI Agent Instructions (Kalico AWD Sync)

Hello, fellow AI coding assistant! If you are reading this, you are working on the **Kalico TMC4671 AWD Synchronization** module. This file contains critical architectural rules and context to prevent regressions in this codebase. Read this carefully before modifying any code.

## 1. Project Goal & Philosophy
The objective of this module is to achieve **Rigid-Coupling Synchronization** between two or more TMC4671 motor drivers on an Ouroboros board in an All-Wheel Drive (AWD) configuration. 
* **The Problem:** Software-level syncing (Klipper host sending the same target to both drivers) introduces latency and mechanical fighting between the motors due to micro-timing differences.
* **The Solution:** We intercept the SPI bus at the MCU (C-level) and constantly copy the true, live torque state of the "Leader" driver into the "Follower" driver.

## 2. Architecture
The AWD Sync system is split across the Python host and the C-level MCU firmware:

1. **`tmc4671_sync.py` (Klipper Host):**
   * Acts as the configurator.
   * It is responsible for identifying the Leader and Follower drivers, setting up the safety boundaries, and sending the `config_tmc4671_sync` initialization command to the MCU.
2. **`tmc4671_sync.c` (MCU Firmware):**
   * Contains the high-speed SPI synchronization task loop.
   * This is where the actual torque mirroring happens.

## 3. Mathematical & Hardware "Gotchas" (CRITICAL)

### A. The Torque Register Mirroring Rule
For matched motors and drivers, sync the target registers (`PID_TORQUE_FLUX_TARGET`).
* **The Rule:** The sync loop MUST read the `PID_TORQUE_FLUX_TARGET` value (Register `0x64`) from the Leader, and write it directly to the `PID_TORQUE_FLUX_TARGET` register (Register `0x64`) of the Follower.
* **Why:** Because the Leader and Follower motors and drivers are perfectly matched, copying the exact **Target** command bypasses any momentary noise or filtering artifacts that the Leader's own internal current PID loops might be experiencing. This results in a perfectly synchronized driving force without compounding signal noise.

### B. SPI Bus Delay
* **The Rule:** The SPI synchronization loop in the MCU contains a mandatory **500ns delay**. 
* **Datasheet Specification:** SPI write access can be performed up to 8 MHz. SPI read access can be performed up to 8 MHz ONLY if a pause of at least 500 ns is inserted *after* the transfer of the address byte. Without this 500 ns pause after the address byte, SPI read access is limited to a maximum of 2 MHz.
* DO NOT remove or optimize away this delay if operating the SPI bus at high frequencies (>2MHz).

### C. Testing on the Linux Simulator
If you are testing the firmware logic using the `linux` MCU simulator:
* The simulator will crash with "Timer too close" or assertion errors if you don't bypass hardware-level initialization constraints.
* The Linux environment does not have real TMC4671 chips attached, so any code expecting valid hardware identification (e.g., waiting for the `0x4671` ID register) must be explicitly mocked or bypassed during simulator testing to allow the host handshake to complete.

### D. TMC4671 Pipelined SPI Sub-Registers & MCU Preemption
The TMC4671 uses a multiplexed sub-register architecture (e.g., writing an address to `0x01` and reading data from `0x00`).
* **The Problem:** Klipper's Python driver (`tmc4671.py`) performs these multi-step sub-register accesses as two distinct, independent SPI commands (`spi_send` followed by `spi_transfer`). Because Klipper cooperatively schedules MCU tasks, the high-speed C-loop (`tmc4671_sync_task`) can and will interleave its own single-frame direct register reads (like `0x64`) right in the middle of Klipper's two-step sub-register dance. Any interleaved SPI access resets the TMC4671's internal SPI read-fetch state machine, causing Klipper to read back garbage or mismatched target payloads instead of the requested sub-register data.
* **The Solution:** Host SPI access is strictly serialized with the C-loop via the `tmc4671_sync_pause` and `tmc4671_sync_resume` MCU commands. On `klippy:mcu_identify`, `tmc4671_sync.py` monkey-patches `MCU_TMC_SPI.get_register`, `set_register`, and `set_register_once` on both the Leader and Follower to wrap each call in a pause/resume pair, keeping the whole two-step sub-register sequence inside a single pause window. Because Klipper's MCU tasks are cooperative, the `if (sync->paused) continue;` check at the top of `tmc4671_sync_task` is sufficient — any in-flight cycle completes atomically before the pause command is processed.
* **The Rule:** If you ever modify how `tmc4671.py` handles multi-step communication, or if you add new synchronization loops to the C-firmware, you **MUST** ensure that MCU tasks safely yield their SPI operations while the host driver completes stateful sub-register sequences. New host entry points that touch TMC registers directly (bypassing `MCU_TMC_SPI`) must be wrapped in the same pause/resume pair or routed through the monkey-patched methods.

### E. State-Transitions, Safety Clamps, & Telemetry
When dealing with a high-speed continuous firmware loop, state management is critical to prevent dangerous race conditions:
* **The "Mode Second" Rule:** To prevent mirroring stale or dangerous targets during Leader state transitions (e.g., motor disables, estops), the C-loop MUST read the `MODE_RAMP_MODE_MOTION` register (0x63) sequentially *after* fetching the target torque. This guarantees the mode was valid at the exact moment the target was captured. Any invalid mode must immediately force the Follower's target to `0`.
* **Telemetry Sterilization:** When `SYNC_START` is called, the C-loop must completely zero-out all accumulators, counters, and max latency trackers (especially `current_divergence_ticks`). Failing to clear legacy state can trigger false-positive hardware shutdowns instantly on the next print.
* **Divergence Thresholds:** The Python host calculates dynamic bounds (e.g., capping `divergence_time` mathematically based on `sync_rate`) to prevent `uint16_t` wrapping in the C-struct. If you add new physical bounds to the firmware, they must be strictly clamped in `tmc4671_sync.py` to prevent silent overflows.
