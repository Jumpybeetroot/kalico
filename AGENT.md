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
   * It is responsible for identifying the Leader and Follower drivers, setting up the safety boundaries, and sending the `tmc4671_awd_sync` initialization command to the MCU.
2. **`tmc4671.c` (MCU Firmware):**
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
