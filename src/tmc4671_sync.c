// TMC4671 Leader/Follower synchronization via SPI
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_MACH_STM32H7
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // try_shutdown
#include "spicmds.h" // spidev_transfer
#include "board/misc.h" // timer_read_time
#include "byteorder.h" // be32_to_cpu

#ifdef CONFIG_MACH_STM32H7
#include "stm32/internal.h"
#include "generic/armcm_boot.h" // armcm_enable_irq
#endif

void configure_awd_hardware_engine(void);
void start_awd_hardware_engine(void);
void stop_awd_hardware_engine(void);

struct tmc4671_sync_s {
    uint32_t rest_time;
    struct spidev_s *leader_spi;
    struct spidev_s *follower_spi;
    uint16_t valid_mode_ticks;
    uint32_t cycle_count;
    uint32_t overrun_count;
    uint32_t last_leader_value;
    uint32_t max_isr_cycles;   // peak ISR duration, timer ticks (1 tick = 1 CPU cycle on H7)
    uint32_t last_isr_cycles;  // most recent ISR duration, timer ticks
    uint8_t trajectory_decimation; // run trajectory loop every Nth fire (1 = every)
    uint8_t trajectory_counter;
    uint8_t paused;
};

// Global pointer for ISR access
static struct tmc4671_sync_s *sync_global = NULL;
extern volatile int32_t *awd_absolute_target_position_ptr;

void command_config_tmc4671_sync(uint32_t *args) {
    struct tmc4671_sync_s *sync = oid_alloc(args[0], command_config_tmc4671_sync, sizeof(*sync));
    sync->leader_spi = spidev_oid_lookup(args[1]);
    sync->follower_spi = spidev_oid_lookup(args[2]);
    sync->trajectory_decimation = args[3] < 1 ? 1 : args[3];
    sync->trajectory_counter = 0;
    sync->valid_mode_ticks = 0;
    sync->paused = 0;
    sync_global = sync;
}
DECL_COMMAND(command_config_tmc4671_sync,
             "config_tmc4671_sync oid=%c leader_spi_oid=%c follower_spi_oid=%c traj_decim=%c");

void command_tmc4671_sync_start(uint32_t *args) {
    struct tmc4671_sync_s *sync = oid_lookup(args[0], command_config_tmc4671_sync);

    // Reset counters and accumulators for a clean start
    sync->cycle_count = 0;
    sync->overrun_count = 0;
    sync->valid_mode_ticks = 0;
    sync->last_leader_value = 0;
    sync->max_isr_cycles = 0;
    sync->last_isr_cycles = 0;
    sync->trajectory_counter = 0;
    sync->paused = 0;

    // args[1] (clock) is unused; the hardware engine is driven by TIM6, not by
    // Klipper's software scheduler. Kept in the protocol for backward
    // compatibility with existing host code.
    sync->rest_time = args[2];
    configure_awd_hardware_engine();
    start_awd_hardware_engine();
}
DECL_COMMAND(command_tmc4671_sync_start,
             "tmc4671_sync_start oid=%c clock=%u rest_ticks=%u");

void command_tmc4671_sync_stop(uint32_t *args) {
    uint8_t oid = args[0];
    struct tmc4671_sync_s *sync = oid_lookup(oid, command_config_tmc4671_sync);
    // Stop the preemptive engine
    stop_awd_hardware_engine();

    // Disarm the follower: Write 0 to PID_TORQUE_FLUX_TARGET (0x64 | 0x80 = 0xE4)
    uint8_t zero_msg[5] = { 0xE4, 0x00, 0x00, 0x00, 0x00 };
    spidev_transfer(sync->follower_spi, 0, 5, zero_msg);
}
DECL_COMMAND(command_tmc4671_sync_stop, "tmc4671_sync_stop oid=%c");

void command_tmc4671_sync_pause(uint32_t *args) {
    uint8_t oid = args[0];
    struct tmc4671_sync_s *sync = oid_lookup(oid, command_config_tmc4671_sync);
    sync->paused = 1;
}
DECL_COMMAND(command_tmc4671_sync_pause, "tmc4671_sync_pause oid=%c");

void command_tmc4671_sync_resume(uint32_t *args) {
    uint8_t oid = args[0];
    struct tmc4671_sync_s *sync = oid_lookup(oid, command_config_tmc4671_sync);
    sync->paused = 0;
}
DECL_COMMAND(command_tmc4671_sync_resume, "tmc4671_sync_resume oid=%c");

extern volatile uint8_t kalico_spi_active;

void TIM6_DAC_IRQHandler(void) {
#ifdef CONFIG_MACH_STM32H7
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF; // Clear interrupt

        // Drop frame if Klipper or the Python host has paused the engine
        if (!sync_global || sync_global->paused)
            return;

        // If Klipper is actively transmitting on SPI2, we must drop the frame
        // to prevent bus collision. We track this as an overrun.
        if (kalico_spi_active) {
            sync_global->overrun_count++;
            return;
        }

        sync_global->cycle_count++;

        // DIAG: timestamp ISR start using Klipper's timer (400MHz on H7)
        uint32_t isr_start_t = timer_read_time();

        // 10kHz Torque Mirror Loop
        // 0x64 is PID_TORQUE_FLUX_TARGET
        uint8_t read_msg[5] = { 0x64, 0x00, 0x00, 0x00, 0x00 };
        spidev_transfer_tmc4671_read(sync_global->leader_spi, read_msg);

        uint32_t val;
        memcpy(&val, &read_msg[1], 4);
        sync_global->last_leader_value = be32_to_cpu(val);

        // Check leader mode after capturing the target. 0x63 is
        // MODE_RAMP_MODE_MOTION. Reading mode second ensures the captured
        // target is only mirrored if the leader was in a valid closed-loop
        // motion mode at that point in the SPI sequence.
        uint8_t mode_msg[5] = { 0x63, 0x00, 0x00, 0x00, 0x00 };
        spidev_transfer_tmc4671_read(sync_global->leader_spi, mode_msg);

        uint32_t mode_val;
        memcpy(&mode_val, &mode_msg[1], 4);
        uint8_t mode = be32_to_cpu(mode_val) & 0xFF;

        if (mode == 1 || mode == 2 || mode == 3) {
            if (sync_global->valid_mode_ticks < 255)
                sync_global->valid_mode_ticks++;
        } else {
            sync_global->valid_mode_ticks = 0;
        }

        if (sync_global->valid_mode_ticks < 2) {
            val = 0;
            uint32_t be_val = cpu_to_be32(val);
            memcpy(&read_msg[1], &be_val, 4);
            sync_global->last_leader_value = 0;
        }

        // Write to follower (0xE4)
        uint8_t write_msg[5] = { 0xE4, read_msg[1], read_msg[2], read_msg[3], read_msg[4] };
        spidev_transfer(sync_global->follower_spi, 0, 5, write_msg);

        // Trajectory loop runs every Nth fire (decimated from TIM6 rate).
        // Decimation = 1 → every fire (original behaviour).
        if (++sync_global->trajectory_counter >= sync_global->trajectory_decimation) {
            sync_global->trajectory_counter = 0;

            // Transfer absolute_target_position to Leader PID_POSITION_TARGET (0x68 -> 0xE8)
            uint32_t pos = awd_absolute_target_position_ptr ? *awd_absolute_target_position_ptr : 0;
            uint32_t be_pos = cpu_to_be32(pos);
            uint8_t pos_msg[5] = { 0xE8, 0, 0, 0, 0 };
            memcpy(&pos_msg[1], &be_pos, 4);

            spidev_transfer(sync_global->leader_spi, 0, 5, pos_msg);

            // Calculate Following Error
            uint8_t act_msg[5] = { 0x6B, 0, 0, 0, 0 }; // PID_POSITION_ACTUAL
            spidev_transfer_tmc4671_read(sync_global->leader_spi, act_msg);

            uint32_t act_val;
            memcpy(&act_val, &act_msg[1], 4);
            int32_t actual_pos = (int32_t)be32_to_cpu(act_val);

            int32_t error = (int32_t)pos - actual_pos;
            if (error < 0) error = -error;

            if (error > 16384) {
                try_shutdown("AWD Following Error E-Stop Triggered");
            }
        }

        // DIAG: record ISR duration in timer ticks (uint32 wrap is safe under subtract)
        uint32_t isr_dur = timer_read_time() - isr_start_t;
        sync_global->last_isr_cycles = isr_dur;
        if (isr_dur > sync_global->max_isr_cycles)
            sync_global->max_isr_cycles = isr_dur;
    }
#endif
}

void configure_awd_hardware_engine(void) {
#ifdef CONFIG_MACH_STM32H7
    // Make sure we have a valid reference
    if (!sync_global) return;

    // DIAG: ISR duration measurement uses timer_read_time() (Klipper's existing
    // 400MHz tick source). DWT/TRCENA path was tried first and regressed 5kHz.

    // Enable peripheral clocks for TIM6
    RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;

    // Calculate microseconds from rest_time (system clock ticks)
    uint32_t period_us = ((uint64_t)sync_global->rest_time * 1000000) / CONFIG_CLOCK_FREQ;
    if (period_us < 10) period_us = 10; // Safety floor
    if (period_us > 65535) period_us = 65535; // 16-bit max limit

    // Configure TIM6 dynamically based on host sync_rate
    TIM6->PSC = (get_pclock_frequency((uint32_t)TIM6) / 1000000) - 1; // 1MHz tick
    TIM6->ARR = period_us - 1;
    TIM6->DIER |= TIM_DIER_UIE; // Update interrupt enable
    // Priority 3 prevents starvation of Klipper's main stepper timer (Priority 2) and USB (Priority 1)
    armcm_enable_irq(TIM6_DAC_IRQHandler, TIM6_DAC_IRQn, 3);
#endif
}

void start_awd_hardware_engine(void) {
#ifdef CONFIG_MACH_STM32H7
    TIM6->CR1 |= TIM_CR1_CEN;
#endif
}

void stop_awd_hardware_engine(void) {
#ifdef CONFIG_MACH_STM32H7
    TIM6->CR1 &= ~TIM_CR1_CEN;
#endif
}


void command_tmc4671_sync_status(uint32_t *args) {
    uint8_t oid = args[0];
    struct tmc4671_sync_s *sync = oid_lookup(oid, command_config_tmc4671_sync);
    sendf("tmc4671_sync_status_response oid=%c cycle_count=%u overrun_count=%u last_leader_value=%u max_isr_cycles=%u last_isr_cycles=%u",
          oid, sync->cycle_count, sync->overrun_count, sync->last_leader_value, sync->max_isr_cycles, sync->last_isr_cycles);
    sync->max_isr_cycles = 0; // DIAG: reset on read so each dump shows peak since last
}
DECL_COMMAND(command_tmc4671_sync_status, "tmc4671_sync_status oid=%c");
