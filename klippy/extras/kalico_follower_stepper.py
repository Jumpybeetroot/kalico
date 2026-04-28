import logging
from klippy import stepper

# Map (kinematics, axis) -> (itersolve alloc function, alloc argument bytes).
# For corexy, the A motor (leader stepper_a) gets '+', B motor gets '-'.
_CARTESIAN_AXES = {'x': b'x', 'y': b'y', 'z': b'z'}
_COREXY_AXES = {
    'a': b'+', 'x': b'+', '+': b'+',
    'b': b'-', 'y': b'-', '-': b'-',
}


class _StepperConfigAlias:
    def __init__(self, config, name):
        self._config = config
        self._name = name

    def get_name(self):
        return self._name

    def __getattr__(self, attr):
        return getattr(self._config, attr)


class KalicoFollowerStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        name_parts = config.get_name().split(' ')
        self.name = name_parts[1] if len(name_parts) > 1 else config.get_name()

        self.kinematics = config.getchoice(
            'kinematics', {'cartesian': 'cartesian', 'corexy': 'corexy'},
            'cartesian')
        axis = config.get('axis').lower()
        if self.kinematics == 'cartesian':
            if axis not in _CARTESIAN_AXES:
                raise config.error(
                    f"[kalico_follower_stepper {self.name}]: axis must be "
                    f"'x', 'y', or 'z' for cartesian kinematics")
            alloc_fn = "cartesian_stepper_alloc"
            alloc_arg = _CARTESIAN_AXES[axis]
        else:
            if axis not in _COREXY_AXES:
                raise config.error(
                    f"[kalico_follower_stepper {self.name}]: axis must be "
                    f"'a'/'x'/'+' (A motor) or 'b'/'y'/'-' (B motor) for "
                    f"corexy kinematics")
            alloc_fn = "corexy_stepper_alloc"
            alloc_arg = _COREXY_AXES[axis]

        stepper_config = _StepperConfigAlias(config, self.name)
        self.stepper = stepper.PrinterStepper(stepper_config)
        self.stepper.setup_itersolve(alloc_fn, alloc_arg)

        self._linked = False  # becomes True at klippy:connect

        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("KALICO_LINK_FOLLOWER", self.cmd_KALICO_LINK_FOLLOWER,
                               desc="Link/unlink kalico follower stepper trapq")

    def handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        self.stepper.set_trapq(toolhead.get_trapq())
        toolhead.register_step_generator(self.stepper.generate_steps)
        self._linked = True

    def set_sync_engaged(self, engaged):
        # Called by [tmc4671_sync] on SYNC_START (engaged=True) / SYNC_STOP (False).
        # engaged=True  -> unlink trapq so no step pulses are generated.
        # engaged=False -> resync software position to the toolhead and relink.
        if engaged == (not self._linked):
            return
        toolhead = self.printer.lookup_object('toolhead')
        # Flush before mutating trapq: otherwise the step generator could be
        # partway through a move against a trapq that's about to change.
        toolhead.flush_step_generation()
        if engaged:
            self.stepper.set_trapq(None)
            self._linked = False
        else:
            # While unlinked, toolhead moved but this stepper's software
            # position did not. Reseat to the current toolhead cartesian
            # position before relinking; the itersolve (cartesian or corexy)
            # converts (x,y,z) to motor position via its calc_position_cb.
            self.stepper.set_position(toolhead.get_position()[:3])
            self.stepper.set_trapq(toolhead.get_trapq())
            self._linked = True

    def cmd_KALICO_LINK_FOLLOWER(self, gcmd):
        follower_name = gcmd.get('FOLLOWER', None)
        if follower_name is not None and follower_name != self.name:
            return
        enable = gcmd.get_int('ENABLE', 1)
        self.set_sync_engaged(not enable)
        gcmd.respond_info(
            f"Kalico follower '{self.name}' "
            f"{'linked to' if enable else 'unlinked from'} trapq")

def load_config_prefix(config):
    return KalicoFollowerStepper(config)
