import odrive
from odrive.utils import *
from odrive.enums import *
from fibre import Event

class ODrive:

    connected_odrive = None
    axis0_enabled = False
    axis1_enabled = False
    control_mode = AXIS_STATE_IDLE

    def __init__(self, connected_odrive):
        self.connected_odrive = connected_odrive

    def start_watchdog(self, timeout):
        # enable watchdog timer for each motor
        connected_odrive.axis0.config.enable_watchdog = True
        connected_odrive.axis1.config.enable_watchdog = True

        # set watchdog timer timeout
        connected_odrive.axis0.config.watchdog_timeout = timeout
        connected_odrive.axis1.config.watchdog_timeout = timeout

    def stop_watchdog(connected_odrive):

        # disable watchdog timer for each motor
        connected_odrive.axis0.config.enable_watchdog = False
        connected_odrive.axis1.config.enable_watchdog = False
