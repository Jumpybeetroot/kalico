// TMC4671 Leader/Follower synchronization via SPI
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK, sched_add_timer, etc.
#include "spicmds.h" // spidev_transfer
#include "board/misc.h" // timer_read_time
#include "byteorder.h" // be32_to_cpu

struct tmc4671_sync_s {
    struct timer timer;
    uint32_t rest_time;
    struct spidev_s *leader_spi;
    struct spidev_s *follower_spi;
    uint8_t flags;
    uint32_t scheduled_time;
    uint32_t cycle_count;
    uint32_t overrun_count;
    uint32_t max_latency;
    uint32_t absolute_max_latency;
    uint32_t last_leader_value;
    uint16_t divergence_threshold;
    uint16_t max_divergence_ticks;
    uint16_t current_divergence_ticks;
};

enum {
    TMC_SYNC_PENDING = 1,
};

static struct task_wake tmc4671_sync_wake;

static uint_fast8_t tmc4671_sync_event(struct timer *timer) {
    struct tmc4671_sync_s *sync = container_of(timer, struct tmc4671_sync_s, timer);
    sched_wake_task(&tmc4671_sync_wake);
    
    if (sync->flags & TMC_SYNC_PENDING) {
        sync->overrun_count++;
    } else {
        sync->scheduled_time = timer->waketime;
    }
    
    sync->flags |= TMC_SYNC_PENDING;
    sync->timer.waketime += sync->rest_time;
    return SF_RESCHEDULE;
}

void command_config_tmc4671_sync(uint32_t *args) {
    struct tmc4671_sync_s *sync = oid_alloc(args[0], command_config_tmc4671_sync, sizeof(*sync));
    sync->timer.func = tmc4671_sync_event;
    sync->leader_spi = spidev_oid_lookup(args[1]);
    sync->follower_spi = spidev_oid_lookup(args[2]);
    sync->divergence_threshold = args[3];
    sync->max_divergence_ticks = args[4];
    sync->current_divergence_ticks = 0;
}
DECL_COMMAND(command_config_tmc4671_sync,
             "config_tmc4671_sync oid=%c leader_spi_oid=%c follower_spi_oid=%c div_thresh=%hu div_ticks=%hu");

void command_tmc4671_sync_start(uint32_t *args) {
    struct tmc4671_sync_s *sync = oid_lookup(args[0], command_config_tmc4671_sync);
    sched_del_timer(&sync->timer);
    sync->timer.waketime = args[1];
    sync->rest_time = args[2];
    if (sync->rest_time)
        sched_add_timer(&sync->timer);
}
DECL_COMMAND(command_tmc4671_sync_start,
             "tmc4671_sync_start oid=%c clock=%u rest_ticks=%u");

void command_tmc4671_sync_stop(uint32_t *args) {
    uint8_t oid = args[0];
    struct tmc4671_sync_s *sync = oid_lookup(oid, command_config_tmc4671_sync);
    sched_del_timer(&sync->timer);
    sync->flags &= ~TMC_SYNC_PENDING;

    // Disarm the follower: Write 0 to PID_TORQUE_FLUX_TARGET (0x64 | 0x80 = 0xE4)
    uint8_t zero_msg[5] = { 0xE4, 0x00, 0x00, 0x00, 0x00 };
    spidev_transfer(sync->follower_spi, 0, 5, zero_msg);
}
DECL_COMMAND(command_tmc4671_sync_stop, "tmc4671_sync_stop oid=%c");

void tmc4671_sync_task(void) {
    if (!sched_check_wake(&tmc4671_sync_wake))
        return;
    uint8_t oid;
    struct tmc4671_sync_s *sync;
    foreach_oid(oid, sync, command_config_tmc4671_sync) {
        if (!(sync->flags & TMC_SYNC_PENDING))
            continue;
        irq_disable();
        uint32_t scheduled = sync->scheduled_time;
        sync->flags &= ~TMC_SYNC_PENDING;
        irq_enable();



        // Check leader mode. 0x63 is MODE_RAMP_MODE_MOTION
        uint8_t mode_msg[5] = { 0x63, 0x00, 0x00, 0x00, 0x00 };
        spidev_transfer_tmc4671_read(sync->leader_spi, mode_msg);
        
        uint32_t mode_val;
        memcpy(&mode_val, &mode_msg[1], 4);
        uint8_t mode = be32_to_cpu(mode_val) & 0xFF;

        // 0x64 is PID_TORQUE_FLUX_TARGET
        // Read from leader using split transfer (500ns pause after address)
        uint8_t read_msg[5] = { 0x64, 0x00, 0x00, 0x00, 0x00 };
        spidev_transfer_tmc4671_read(sync->leader_spi, read_msg);
        
        uint32_t val;
        memcpy(&val, &read_msg[1], 4);
        sync->last_leader_value = be32_to_cpu(val);
        
        // If leader is not in Torque (1), Velocity (2), or Position (3) mode,
        // it may be disabled or reconfiguring. Force the target to 0 for safety.
        if (mode != 1 && mode != 2 && mode != 3) {
            val = 0;
            uint32_t be_val = cpu_to_be32(val);
            memcpy(&read_msg[1], &be_val, 4);
            sync->last_leader_value = 0;
        }

        // Write to follower. Address MSB=1 to write, so 0x64 | 0x80 = 0xE4
        // The read_msg buffer now contains what we want to write in indices 1..4.
        uint8_t write_msg[5] = { 0xE4, read_msg[1], read_msg[2], read_msg[3], read_msg[4] };
        spidev_transfer(sync->follower_spi, 0, 5, write_msg);

        // Record true latency including SPI transactions
        uint32_t now = timer_read_time();
        uint32_t latency = now - scheduled;
        if (latency > sync->max_latency) {
            sync->max_latency = latency;
        }
        if (latency > sync->absolute_max_latency) {
            sync->absolute_max_latency = latency;
        }
        sync->cycle_count++;

        // Divergence detection: read PID_TORQUE_FLUX_ACTUAL (0x69)
        uint8_t act_leader_msg[5] = { 0x69, 0x00, 0x00, 0x00, 0x00 };
        spidev_transfer_tmc4671_read(sync->leader_spi, act_leader_msg);
        
        uint8_t act_follower_msg[5] = { 0x69, 0x00, 0x00, 0x00, 0x00 };
        spidev_transfer_tmc4671_read(sync->follower_spi, act_follower_msg);
        
        uint32_t l_act_val, f_act_val;
        memcpy(&l_act_val, &act_leader_msg[1], 4);
        memcpy(&f_act_val, &act_follower_msg[1], 4);
        
        // TORQUE_ACTUAL is upper 16 bits, FLUX_ACTUAL is lower 16 bits.
        int16_t l_torque = (int16_t)(be32_to_cpu(l_act_val) >> 16);
        int16_t f_torque = (int16_t)(be32_to_cpu(f_act_val) >> 16);
        
        int32_t diff = l_torque - f_torque;
        if (diff < 0) {
            diff = -diff;
        }
        
        if (diff > sync->divergence_threshold) {
            sync->current_divergence_ticks++;
            if (sync->current_divergence_ticks >= sync->max_divergence_ticks) {
                try_shutdown("AWD Torque Divergence Fault");
            }
        } else {
            sync->current_divergence_ticks = 0;
        }
    }
}
DECL_TASK(tmc4671_sync_task);

void command_tmc4671_sync_status(uint32_t *args) {
    uint8_t oid = args[0];
    struct tmc4671_sync_s *sync = oid_lookup(oid, command_config_tmc4671_sync);
    sendf("tmc4671_sync_status_response oid=%c cycle_count=%u overrun_count=%u max_latency=%u absolute_max_latency=%u last_leader_value=%u",
          oid, sync->cycle_count, sync->overrun_count, sync->max_latency, sync->absolute_max_latency, sync->last_leader_value);
    sync->max_latency = 0; // reset on read
}
DECL_COMMAND(command_tmc4671_sync_status, "tmc4671_sync_status oid=%c");
