# AWD Leader-Follower Sync for TMC4671 driver
import logging

class TMC4671Sync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[1]
        self.leader_name = config.get("leader")
        self.follower_name = config.get("follower")
        self.sync_rate = config.getint("sync_rate", 2000, minval=1)
        self.divergence_threshold = config.getint("divergence_threshold", 500, minval=0)
        self.divergence_time = config.getfloat("divergence_time", 0.05, minval=0.0)
        
        self.leader = None
        self.follower = None
        
        self.printer.register_event_handler("klippy:mcu_identify", self.handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.lookup_object('gcode').register_command(
            f"DUMP_SYNC_{self.name.upper()}", self.cmd_DUMP_TMC4671_SYNC,
            desc=f"Dump TMC4671 Leader/Follower sync stats for {self.name}"
        )
        self.printer.lookup_object('gcode').register_command(
            f"SYNC_STOP_{self.name.upper()}", self.cmd_SYNC_STOP,
            desc=f"Stop TMC4671 Leader/Follower sync for {self.name}"
        )
        self.printer.lookup_object('gcode').register_command(
            f"SYNC_START_{self.name.upper()}", self.cmd_SYNC_START,
            desc=f"Start TMC4671 Leader/Follower sync for {self.name}"
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
        
        # Allocate our custom synced spi oid
        self.sync_oid = mcu.create_oid()
        div_ticks = int(self.divergence_time * self.sync_rate)
        mcu.add_config_cmd(
            f"config_tmc4671_sync oid={self.sync_oid} "
            f"leader_spi_oid={leader_spi_oid} follower_spi_oid={follower_spi_oid} "
            f"div_thresh={self.divergence_threshold} div_ticks={div_ticks}"
        )
        
        self.cmd_sync_status = mcu.lookup_query_command(
            "tmc4671_sync_status oid=%c",
            "tmc4671_sync_status_response oid=%c cycle_count=%u overrun_count=%u max_latency=%u absolute_max_latency=%u last_leader_value=%u",
            oid=self.sync_oid
        )
        self.cmd_sync_stop = mcu.lookup_command("tmc4671_sync_stop oid=%c")
        self.cmd_sync_start = mcu.lookup_command("tmc4671_sync_start oid=%c clock=%u rest_ticks=%u")

        logging.info(f"TMC4671 Sync configured for Leader {self.leader_name} "
                     f"and Follower {self.follower_name} at {self.sync_rate}Hz")

    def handle_connect(self):
        if hasattr(self.follower, "mcu_tmc"):
            try:
                # 1. Zero out the follower's target first to prevent sudden jerks
                self.follower.mcu_tmc.set_register("PID_TORQUE_FLUX_TARGET", 0)
                # 2. Put follower into torque mode (MODE_MOTION = 1)
                self.follower.mcu_tmc.set_register("MODE_RAMP_MODE_MOTION", 1)
            except self.printer.command_error as e:
                logging.error(f"TMC4671 Sync: Failed to set follower '{self.follower_name}' to torque mode! Sync will fail.")
                raise e
                
        # 3. Now safely start the sync task
        mcu = self.leader.mcu_tmc.tmc_spi.spi.get_mcu()
        clock = mcu.get_query_slot(mcu.estimated_print_time(self.printer.get_reactor().monotonic()))
        rest_ticks = mcu.seconds_to_clock(1.0 / self.sync_rate)
        self.cmd_sync_start.send([self.sync_oid, clock, rest_ticks])

    def cmd_DUMP_TMC4671_SYNC(self, gcmd):
        if not hasattr(self, 'cmd_sync_status'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
            
        params = self.cmd_sync_status.send([self.sync_oid])
        
        cycle_count = params['cycle_count']
        overrun_count = params['overrun_count']
        max_latency = params['max_latency']
        absolute_max_latency = params['absolute_max_latency']
        last_leader_value = params['last_leader_value']
        
        gcmd.respond_info(f"TMC4671 Sync '{self.name}' Stats:\n"
                           f"Cycles: {cycle_count}\n"
                           f"Overruns: {overrun_count}\n"
                           f"Max Latency (since last dump): {max_latency}\n"
                           f"Absolute Max Latency (all time): {absolute_max_latency}\n"
                           f"Last Leader Torque Target: {last_leader_value}")

    def cmd_SYNC_STOP(self, gcmd):
        if not hasattr(self, 'cmd_sync_stop'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
        self.cmd_sync_stop.send([self.sync_oid])
        gcmd.respond_info(f"TMC4671 Sync '{self.name}' stopped.")

    def cmd_SYNC_START(self, gcmd):
        if not hasattr(self, 'cmd_sync_start'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
        mcu = self.leader.mcu_tmc.tmc_spi.spi.get_mcu()
        clock = mcu.get_query_slot(mcu.estimated_print_time(self.printer.get_reactor().monotonic()))
        rest_ticks = mcu.seconds_to_clock(1.0 / self.sync_rate)
        self.cmd_sync_start.send([self.sync_oid, clock, rest_ticks])
        gcmd.respond_info(f"TMC4671 Sync '{self.name}' started at {self.sync_rate}Hz.")

def load_config_prefix(config):
    return TMC4671Sync(config)
