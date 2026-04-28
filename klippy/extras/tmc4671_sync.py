# AWD Leader-Follower Sync for TMC4671 driver
import logging


def calc_trajectory_decimation(sync_rate, trajectory_rate):
    if trajectory_rate < 1:
        raise ValueError("trajectory_rate must be at least 1 Hz")
    if trajectory_rate > sync_rate:
        raise ValueError(
            f"trajectory_rate ({trajectory_rate}) must not exceed "
            f"sync_rate ({sync_rate})")
    if sync_rate % trajectory_rate:
        raise ValueError(
            f"trajectory_rate ({trajectory_rate}) must divide sync_rate "
            f"({sync_rate}) exactly")
    decim = sync_rate // trajectory_rate
    if decim > 255:
        raise ValueError(
            f"trajectory_rate too low; computed decimation {decim} exceeds "
            "8-bit limit of 255")
    return decim


class TMC4671Sync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[1]
        self.leader_name = config.get("leader")
        self.follower_name = config.get("follower")
        self.follower_stepper_name = config.get("follower_stepper", None)
        self.follower_stepper = None

        if self.leader_name == self.follower_name:
            raise config.error(f"[tmc4671_sync {self.name}]: leader and follower must be different drivers")
            
        self.sync_rate = config.getint("sync_rate", 2000, minval=100, maxval=10000)
        # Trajectory loop (target-position write + actual-position read + error
        # check) can run slower than the torque mirror. Default = sync_rate
        # (same as before). Lowering it frees ISR CPU time per cycle.
        self.trajectory_rate = config.getint(
            "trajectory_rate", self.sync_rate, minval=1, maxval=self.sync_rate)
        try:
            self.trajectory_decimation = calc_trajectory_decimation(
                self.sync_rate, self.trajectory_rate)
        except ValueError as e:
            raise config.error(f"[tmc4671_sync {self.name}]: {e}")

        self.leader = None
        self.follower = None
        self._is_sync_active = False
        
        self.printer.register_event_handler("klippy:mcu_identify", self.handle_mcu_identify)
        self.printer.register_event_handler("klippy:disconnect", self.handle_disconnect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            f"DUMP_SYNC_{self.name.upper()}", self.cmd_DUMP_TMC4671_SYNC,
            desc=f"Dump TMC4671 Leader/Follower sync stats for {self.name}"
        )
        gcode.register_command(
            f"SYNC_STOP_{self.name.upper()}", self.cmd_SYNC_STOP,
            desc=f"Stop TMC4671 Leader/Follower sync for {self.name}"
        )
        gcode.register_command(
            f"SYNC_START_{self.name.upper()}", self.cmd_SYNC_START,
            desc=f"Start TMC4671 Leader/Follower sync for {self.name}"
        )
        gcode.register_command(
            f"SPI_TEST_{self.name.upper()}", self.cmd_SPI_TEST,
            desc=f"SPI integrity test for TMC4671 drivers in {self.name}"
        )

        
    def handle_mcu_identify(self):
        # Look up the TMC4671 driver objects
        self.leader = self.printer.lookup_object(f"tmc4671 {self.leader_name}")
        self.follower = self.printer.lookup_object(f"tmc4671 {self.follower_name}")
        
        if not hasattr(self.leader, 'mcu_tmc') or not hasattr(self.follower, 'mcu_tmc'):
            raise self.printer.command_error(
                "TMC4671 Leader/Follower sync requires valid tmc4671 objects"
            )

        mcu = self.leader.mcu_tmc.tmc_spi.spi.get_mcu()
        if mcu is not self.follower.mcu_tmc.tmc_spi.spi.get_mcu():
            raise self.printer.command_error(
                "TMC4671 Leader/Follower sync requires both drivers to be on the same MCU"
            )
            
        leader_spi_oid = self.leader.get_spi_oid() if hasattr(self.leader, 'get_spi_oid') else None
        follower_spi_oid = self.follower.get_spi_oid() if hasattr(self.follower, 'get_spi_oid') else None

        if leader_spi_oid is None or follower_spi_oid is None:
             raise self.printer.command_error("Could not determine SPI OID for TMC4671 drivers")
             
        if leader_spi_oid == follower_spi_oid:
             raise self.printer.command_error(
                 "TMC4671 Leader/Follower sync requires distinct leader and follower drivers"
             )

        if self.follower_stepper_name is not None:
            self.follower_stepper = self.printer.lookup_object(
                f"kalico_follower_stepper {self.follower_stepper_name}")

        # Allocate our custom synced spi oid
        self.sync_oid = mcu.create_oid()
        mcu.add_config_cmd(
            f"config_tmc4671_sync oid={self.sync_oid} "
            f"leader_spi_oid={leader_spi_oid} follower_spi_oid={follower_spi_oid} "
            f"traj_decim={self.trajectory_decimation}"
        )
        
        self.cmd_sync_status = mcu.lookup_query_command(
            "tmc4671_sync_status oid=%c",
            "tmc4671_sync_status_response oid=%c cycle_count=%u overrun_count=%u last_leader_value=%u max_isr_cycles=%u last_isr_cycles=%u",
            oid=self.sync_oid
        )
        self.cmd_sync_stop = mcu.lookup_command("tmc4671_sync_stop oid=%c")
        self.cmd_sync_start = mcu.lookup_command("tmc4671_sync_start oid=%c clock=%u rest_ticks=%u")
        self.cmd_sync_pause = mcu.lookup_command("tmc4671_sync_pause oid=%c")
        self.cmd_sync_resume = mcu.lookup_command("tmc4671_sync_resume oid=%c")
        self.cmd_stepper_zero_awd_position = mcu.lookup_command("stepper_zero_awd_position oid=%c")

        # NOTE: Wrappers are NOT installed here. They are only activated when
        # SYNC_START is called so that the heavy TMC4671 initialization sequence
        # does not flood the MCU command queue with useless pause/resume pairs.
        self._wrap_installed = False

        logging.info(f"TMC4671 Sync configured for Leader {self.leader_name} "
                     f"and Follower {self.follower_name} at {self.sync_rate}Hz")

    def _wrap_tmc_methods(self):
        if self._wrap_installed:
            return
        # Wrap leader
        self.old_leader_get_reg = self.leader.mcu_tmc.get_register
        self.old_leader_set_reg = self.leader.mcu_tmc.set_register_once
        self.old_leader_set_reg_m = self.leader.mcu_tmc.set_register
        
        self.leader.mcu_tmc.get_register = self._leader_get_reg
        self.leader.mcu_tmc.set_register_once = self._leader_set_reg
        self.leader.mcu_tmc.set_register = self._leader_set_reg_m

        # Wrap follower
        self.old_follower_get_reg = self.follower.mcu_tmc.get_register
        self.old_follower_set_reg = self.follower.mcu_tmc.set_register_once
        self.old_follower_set_reg_m = self.follower.mcu_tmc.set_register
        
        self.follower.mcu_tmc.get_register = self._follower_get_reg
        self.follower.mcu_tmc.set_register_once = self._follower_set_reg
        self.follower.mcu_tmc.set_register = self._follower_set_reg_m
        self._wrap_installed = True

    def _unwrap_tmc_methods(self):
        if not self._wrap_installed:
            return
        self.leader.mcu_tmc.get_register = self.old_leader_get_reg
        self.leader.mcu_tmc.set_register_once = self.old_leader_set_reg
        self.leader.mcu_tmc.set_register = self.old_leader_set_reg_m
        self.follower.mcu_tmc.get_register = self.old_follower_get_reg
        self.follower.mcu_tmc.set_register_once = self.old_follower_set_reg
        self.follower.mcu_tmc.set_register = self.old_follower_set_reg_m
        self._wrap_installed = False

    def _leader_get_reg(self, reg_name):
        if not self._is_sync_active:
            return self.old_leader_get_reg(reg_name)
        self.cmd_sync_pause.send([self.sync_oid])
        try:
            return self.old_leader_get_reg(reg_name)
        finally:
            self.cmd_sync_resume.send([self.sync_oid])

    def _leader_set_reg(self, reg_name, val, print_time=None):
        if not self._is_sync_active:
            return self.old_leader_set_reg(reg_name, val, print_time)
        self.cmd_sync_pause.send([self.sync_oid])
        try:
            return self.old_leader_set_reg(reg_name, val, print_time)
        finally:
            self.cmd_sync_resume.send([self.sync_oid])
            
    def _leader_set_reg_m(self, reg_name, val, print_time=None):
        if not self._is_sync_active:
            return self.old_leader_set_reg_m(reg_name, val, print_time)
        self.cmd_sync_pause.send([self.sync_oid])
        try:
            return self.old_leader_set_reg_m(reg_name, val, print_time)
        finally:
            self.cmd_sync_resume.send([self.sync_oid])

    def _follower_get_reg(self, reg_name):
        if not self._is_sync_active:
            return self.old_follower_get_reg(reg_name)
        self.cmd_sync_pause.send([self.sync_oid])
        try:
            return self.old_follower_get_reg(reg_name)
        finally:
            self.cmd_sync_resume.send([self.sync_oid])

    def _follower_set_reg(self, reg_name, val, print_time=None):
        if not self._is_sync_active:
            return self.old_follower_set_reg(reg_name, val, print_time)
        self.cmd_sync_pause.send([self.sync_oid])
        try:
            return self.old_follower_set_reg(reg_name, val, print_time)
        finally:
            self.cmd_sync_resume.send([self.sync_oid])
            
    def _follower_set_reg_m(self, reg_name, val, print_time=None):
        if not self._is_sync_active:
            return self.old_follower_set_reg_m(reg_name, val, print_time)
        self.cmd_sync_pause.send([self.sync_oid])
        try:
            return self.old_follower_set_reg_m(reg_name, val, print_time)
        finally:
            self.cmd_sync_resume.send([self.sync_oid])

    def _arm_follower(self):
        if hasattr(self.follower, "mcu_tmc"):
            # Zero out the follower's target first to prevent sudden jerks,
            # then put it into torque mode without clobbering PID_TYPE/FF bits.
            self.follower.mcu_tmc.set_register("PID_TORQUE_FLUX_TARGET", 0)
            self.follower.mcu_tmc.write_field("MODE_MOTION", 1)

    def _disarm_follower(self):
        if hasattr(self, 'follower') and hasattr(self.follower, "mcu_tmc"):
            self.follower.mcu_tmc.set_register("PID_TORQUE_FLUX_TARGET", 0)

    def handle_disconnect(self):
        self._is_sync_active = False
        self._unwrap_tmc_methods()
        if hasattr(self, 'cmd_sync_stop'):
            try:
                self.cmd_sync_stop.send([self.sync_oid])
            except Exception:
                pass
        try:
            self._disarm_follower()
        except Exception:
            pass
        if self.follower_stepper is not None:
            try:
                self.follower_stepper.set_sync_engaged(False)
            except Exception:
                pass

    def cmd_DUMP_TMC4671_SYNC(self, gcmd):
        if not hasattr(self, 'cmd_sync_status'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
            
        params = self.cmd_sync_status.send([self.sync_oid])

        cycle_count = params['cycle_count']
        overrun_count = params['overrun_count']
        last_leader_value = params['last_leader_value']
        max_isr_cycles = params['max_isr_cycles']
        last_isr_cycles = params['last_isr_cycles']

        # DWT runs at CPU clock; on STM32H7 this matches CONFIG_CLOCK_FREQ.
        mcu = self.leader.mcu_tmc.tmc_spi.spi.get_mcu()
        cpu_hz = mcu.seconds_to_clock(1.0)
        max_isr_us = (max_isr_cycles * 1e6) / cpu_hz if cpu_hz else 0.0
        last_isr_us = (last_isr_cycles * 1e6) / cpu_hz if cpu_hz else 0.0
        period_us = 1e6 / self.sync_rate

        traj_rate = self.sync_rate // self.trajectory_decimation
        gcmd.respond_info(f"TMC4671 Sync '{self.name}' Stats:\n"
                           f"Cycles: {cycle_count}\n"
                           f"Overruns: {overrun_count}\n"
                           f"Last Leader Torque Target: {last_leader_value}\n"
                           f"Rates: torque={self.sync_rate} Hz "
                           f"trajectory={traj_rate} Hz (decim={self.trajectory_decimation})\n"
                           f"ISR duration: last={last_isr_us:.1f}us "
                           f"max={max_isr_us:.1f}us / budget={period_us:.1f}us")

    def cmd_SYNC_STOP(self, gcmd):
        if not hasattr(self, 'cmd_sync_stop'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
        try:
            self.cmd_sync_stop.send([self.sync_oid])
        finally:
            self._is_sync_active = False
            self._unwrap_tmc_methods()

            try:
                self._disarm_follower()
            except Exception:
                pass

            # Relink the follower stepper's trapq only after torque is zero and
            # the C-loop is stopped, so the first subsequent step pulse does not
            # fight a still-active torque mirror.
            if self.follower_stepper is not None:
                try:
                    self.follower_stepper.set_sync_engaged(False)
                except Exception:
                    pass

        gcmd.respond_info(f"TMC4671 Sync '{self.name}' stopped.")

    def cmd_SYNC_START(self, gcmd):
        if not hasattr(self, 'cmd_sync_start'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
        if self._is_sync_active:
            gcmd.respond_info(f"TMC4671 Sync '{self.name}' already started.")
            return

        follower_unlinked = False
        sync_started = False
        try:
            # Stop generating step pulses for the follower before flipping it into
            # torque mode. The TMC4671 ignores STEP/DIR in MODE_MOTION=1, but
            # leaving the trapq linked would (a) keep Klipper's software position
            # tracker drifting vs. the leader, and (b) burn MCU cycles on step
            # compression the driver will silently discard.
            if self.follower_stepper is not None:
                self.follower_stepper.set_sync_engaged(True)
                follower_unlinked = True
            self._arm_follower()
            mcu = self.leader.mcu_tmc.tmc_spi.spi.get_mcu()
            reactor = self.printer.get_reactor()
            clock = mcu.get_query_slot(
                mcu.estimated_print_time(reactor.monotonic()))
            rest_ticks = mcu.seconds_to_clock(1.0 / self.sync_rate)

            # Zero the C-level virtual step accumulator and aim the hardware
            # pointer at the Leader.
            stepper_enable = self.printer.lookup_object("stepper_enable")
            leader_stepper = stepper_enable.lookup_enable(
                self.leader_name).stepper
            self.cmd_stepper_zero_awd_position.send([leader_stepper.get_oid()])

            self.cmd_sync_start.send([self.sync_oid, clock, rest_ticks])
            sync_started = True
            # Install wrappers AFTER the sync task is running on the MCU.
            self._wrap_tmc_methods()
            self._is_sync_active = True
        except Exception:
            self._is_sync_active = False
            self._unwrap_tmc_methods()
            if sync_started:
                try:
                    self.cmd_sync_stop.send([self.sync_oid])
                except Exception:
                    pass
            try:
                self._disarm_follower()
            except Exception:
                pass
            if follower_unlinked and self.follower_stepper is not None:
                try:
                    self.follower_stepper.set_sync_engaged(False)
                except Exception:
                    pass
            raise
        gcmd.respond_info(f"TMC4671 Sync '{self.name}' started at {self.sync_rate}Hz.")

    def cmd_SPI_TEST(self, gcmd):
        count = gcmd.get_int("COUNT", 1000, minval=1, maxval=100000)
        expected = 0x34363731  # CHIPINFO_SI_TYPE constant

        leader_errors = 0
        follower_errors = 0
        leader_last_bad = None
        follower_last_bad = None

        gcmd.respond_info(f"SPI integrity test: reading CHIPINFO_SI_TYPE "
                          f"{count} times from each driver...")

        import time
        start = time.monotonic()
        for i in range(count):
            val = self.leader.mcu_tmc.get_register("CHIPINFO_SI_TYPE")
            if val != expected:
                leader_errors += 1
                leader_last_bad = val
            val = self.follower.mcu_tmc.get_register("CHIPINFO_SI_TYPE")
            if val != expected:
                follower_errors += 1
                follower_last_bad = val
        elapsed = time.monotonic() - start

        rate = (count * 2) / elapsed if elapsed > 0 else 0
        result = (f"SPI Test Complete ({count} reads per driver, "
                  f"{elapsed:.2f}s, {rate:.0f} reads/s):\n"
                  f"  Leader  '{self.leader_name}': "
                  f"{leader_errors} errors")
        if leader_last_bad is not None:
            result += f" (last bad: 0x{leader_last_bad:08x})"
        result += (f"\n  Follower '{self.follower_name}': "
                   f"{follower_errors} errors")
        if follower_last_bad is not None:
            result += f" (last bad: 0x{follower_last_bad:08x})"

        if leader_errors == 0 and follower_errors == 0:
            result += "\n  PASS - No SPI corruption detected"
        else:
            result += f"\n  FAIL - {leader_errors + follower_errors} total errors"

        gcmd.respond_info(result)

def load_config_prefix(config):
    return TMC4671Sync(config)
