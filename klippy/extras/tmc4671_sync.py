# AWD Leader-Follower Sync for TMC4671 driver
import logging

class TMC4671Sync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split(' ')[1]
        self.leader_name = config.get("leader")
        self.follower_name = config.get("follower")
        self.sync_rate = config.getint("sync_rate", 2000, minval=1)
        
        self.leader = None
        self.follower = None
        
        self.printer.register_event_handler("klippy:mcu_identify", self.handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.lookup_object('gcode').register_command(
            f"DUMP_SYNC_{self.name.upper()}", self.cmd_DUMP_TMC4671_SYNC,
            desc=f"Dump TMC4671 Leader/Follower sync stats for {self.name}"
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
            
        leader_spi_oid = getattr(self.leader.mcu_tmc.tmc_spi.spi, 'oid', None)
        follower_spi_oid = getattr(self.follower.mcu_tmc.tmc_spi.spi, 'oid', None)

        if leader_spi_oid is None or follower_spi_oid is None:
             logging.info(f"Leader SPI: {dir(self.leader.mcu_tmc.tmc_spi.spi)}")
             raise self.printer.command_error("Could not determine SPI OID for TMC4671 drivers")
        
        # Calculate rest ticks based on sync_rate
        clock = mcu.get_query_slot(mcu.estimated_print_time(self.printer.get_reactor().monotonic()))
        rest_ticks = mcu.seconds_to_clock(1.0 / self.sync_rate)
        
        # Allocate our custom synced spi oid
        self.sync_oid = mcu.create_oid()
        mcu.add_config_cmd(
            f"config_tmc4671_sync oid={self.sync_oid} "
            f"leader_spi_oid={leader_spi_oid} follower_spi_oid={follower_spi_oid}"
        )
        
        # Command them to start
        mcu.add_config_cmd(
            f"tmc4671_sync_start oid={self.sync_oid} clock={clock} rest_ticks={rest_ticks}",
            is_init=True
        )
        
        self.cmd_sync_status = mcu.lookup_query_command(
            "tmc4671_sync_status oid=%c",
            "tmc4671_sync_status_response oid=%c cycle_count=%u overrun_count=%u max_latency=%u last_leader_value=%u",
            oid=self.sync_oid
        )

        logging.info(f"TMC4671 Sync configured for Leader {self.leader_name} "
                     f"and Follower {self.follower_name} at {self.sync_rate}Hz")

    def handle_connect(self):
        # Override follower's mode to Torque mode (MODE_MOTION = 1)
        if hasattr(self.follower, "mcu_tmc"):
            try:
                self.follower.mcu_tmc.set_register("MODE_RAMP_MODE_MOTION", 1)
            except self.printer.command_error:
                pass

    def cmd_DUMP_TMC4671_SYNC(self, gcmd):
        if not hasattr(self, 'cmd_sync_status'):
            gcmd.respond_info("TMC4671 Sync not fully initialized yet.")
            return
            
        params = self.cmd_sync_status.send([self.sync_oid])
        
        cycle_count = params['cycle_count']
        overrun_count = params['overrun_count']
        max_latency = params['max_latency']
        last_leader_value = params['last_leader_value']
        
        gcmd.respond_info(f"TMC4671 Sync '{self.name}' Stats:\n"
                           f"Cycles: {cycle_count}\n"
                           f"Overruns: {overrun_count}\n"
                           f"Max Latency (ticks): {max_latency}\n"
                           f"Last Leader Torque Target: {last_leader_value}")

def load_config_prefix(config):
    return TMC4671Sync(config)
