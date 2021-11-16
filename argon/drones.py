import time
import dronekit
from pymavlink import mavutil


class IRIS(dronekit.Vehicle):
    ''' extend Vehicle class to customize functionality for IRIS+ w/ camera '''

    def lock_yaw(self, heading, direction='clock', absolute=True):
        ''' lock vehicle yaw at a specific vehicle heading '''
        msg = self.message_factory.command_long_encode(
            0, 0,                                   # system, component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,                                      # confirmation
            heading,                                # yaw in degrees
            0,                                      # speed as deg/sec
            1 if direction == 'clock' else -1,      # -1 is CCW, 1 is CW
            0 if absolute is True else 1,           # 1 is rel, 0 is abs
            0, 0, 0                                 # unused parameters
        )
        self.send_mavlink(msg)

    def unlock_yaw(self):
        ''' reset yaw to follow direction of vehicle travel '''
        msg = self.message_factory.command_long_encode(
            0, 0,                                   # target system, component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,     # command
            0,                                      # confirmation
            0, 0, 0, 0, 0, 0, 0                     # all params empty to reset
        )
        self.send_mavlink(msg)

    def trigger_camera(self):
        ''' trigger camera via usb cable via signal override on ch 7
            the handheld radio controller must be on for this to work
        '''
        # override on channel 7 to send signal to trigger camera
        self.channels.overrides[7] = 2000
        # send signal for 1 second
        time.sleep(1)
        # clear override
        self.channels.overrides = {}
