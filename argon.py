import cmd, re, time
import os, sys
import dronekit, dronekit_sitl
from pymavlink import mavutil
from LatLon import LatLon
from clint.textui import puts, colored


class console(object):
    ''' methods for console output using clint components '''

    @classmethod
    def clear(cls):
        ''' clear the console '''
        os.system('clear')

    @classmethod
    def blank(cls):
        ''' output a blank line to the console '''
        puts(newline=True)

    @classmethod
    def white(cls, text):
        ''' output white text to the console '''
        puts(text)

    @classmethod
    def red(cls, text):
        ''' output red text to the console '''
        puts(colored.red(text))


class arguments(object):
    ''' methods for parsing arguments from Cmd app console line args '''

    @classmethod
    def _parse_abstract(cls, line, regex, typ):
        ''' base functionality for argument parsing '''
        value = None
        try:
            match = re.search(regex, line)
            if match is not None:
                value = typ(match.group(1))
        except:
            pass
        return value

    @classmethod
    def altitude(cls, line):
        ''' parse --alt argument to an int '''
        return cls._parse_abstract(line, r'alt=(\d+)', int)

    @classmethod
    def distance(cls, line):
        ''' parse --dist argument to an int '''
        return cls._parse_abstract(line, r'dist=(\d+)', int)

    @classmethod
    def heading(cls, line):
        ''' parse --head argument to an int '''
        return cls._parse_abstract(line, r'head=(\d+)', int)

    @classmethod
    def position(cls, line):
        ''' parse --lat/--lng arguments to a tuple '''
        # parse lat/lng/alt as floats, int
        latitude = cls._parse_abstract(line, r'lat=(-?\d+.\d+)', float)
        longitude = cls._parse_abstract(line, r'lng=(-?\d+.\d+)', float)
        # only return lat/lng as a pair, otherwise both as None
        if latitude == None or longitude == None:
            latitude, longitude = None, None
        return latitude, longitude


class IRIS(dronekit.Vehicle):
    ''' extend the base dronekit vehicle to provide
        better yaw and camera trigger control methods
    '''

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


class App(cmd.Cmd):
    ''' console application to control drone '''

    prompt = '# '               # console prompt character prefix
    speed = 5.0                 # vehicle speed as a float
    range = 300                 # max movement distance

    def cmdloop(self):
        try:
            cmd.Cmd.cmdloop(self)
        except KeyboardInterrupt:
            console.blank()  # handle a ctrl-C by moving to new/blank console line
            self.cmdloop()

    def __init__(self, test=False):
        cmd.Cmd.__init__(self)
        console.clear()
        console.white("# argon : dronekit-based flight control console \n")
        console.white("connecting to drone")
        try:
            console.white("... waiting on connection")
            if test == True:
                self.vehicle = dronekit.connect('tcp:127.0.0.1:5760',
                        vehicle_class=IRIS,
                        status_printer=self._status_printer,
                        wait_ready=True,
                        heartbeat_timeout=30,   # 30 second timeout
                    )
            else:
                self.vehicle = dronekit.connect('/dev/cu.usbserial-DJ00DSDS',
                        baud=57600,
                        vehicle_class=IRIS,
                        status_printer=self._status_printer,
                        wait_ready=True,
                        heartbeat_timeout=30,   # 30 second timeout
                    )
            console.white("... connected \n")
        except KeyboardInterrupt:
            console.white("... canceling connection attempt \n")
            return True
        except Exception:
            console.white("... unable to connect \n")
            return True

    def default(self, args):
        console.red("unknown command, try 'help' for available commands \n")

    def emptyline(self):
        console.blank()
        pass

    def _status_printer(self, txt):
        return

    def _wait(self, delay=3):
        time.sleep(delay)

    # console functionality

    def do_clear(self, args):
        ''' clear the console window '''
        console.clear()

    def do_version(self, args):
        ''' print current vehicle firmware version '''
        self.vehicle.wait_ready('autopilot_version')
        console.white("vehicle firmware : {}".format(self.vehicle.version))
        console.blank()

    def do_exit(self, args):
        ''' close connection to vehicle and exit console environment '''
        console.white("closing connection \n")
        self.vehicle.close()
        return True

    def do_help(self, args):
        ''' print help text, all commands with options/details '''
        with open('help.txt', 'r') as help:
            console.white(help.read())

    # vehicle state

    def do_status(self, args):
        ''' get system status from the vehicle
              system (active or standby)
              mode (guided, loiter, alt_hold, rtl, land)
        '''
        console.white("system: {}".format(self.vehicle.system_status.state))
        console.white("mode: {}".format(self.vehicle.mode.name))
        console.blank()

    def do_telemetry(self, args):
        ''' print the telemetry data from vehicle to the console '''
        location = self.vehicle.location.global_relative_frame
        console.white("position: {}, {}".format(location.lat, location.lon))
        console.white("altitude: {}m".format(location.alt))
        console.white("heading: {}".format(360 if self.vehicle.heading == 0 else self.vehicle.heading))
        console.white("airspeed: {}".format(self.vehicle.airspeed))
        console.white("battery: {} / {:.3}".format(
                self.vehicle.battery.voltage,
                self.vehicle.parameters['FS_BATT_VOLTAGE']
            ))
        console.blank()

    def do_mode(self, arg):
        ''' set vehicle mode to 'guided', 'loiter' '''
        # options as a dict, with slugs and VehicleMode object
        modes = {
                'loiter': dronekit.VehicleMode('LOITER'),
                'guided': dronekit.VehicleMode('GUIDED'),
            }
        # check that the provided arguments is an option
        if arg in modes.keys():
            console.white("switching to {} mode".format(arg))
            self.vehicle.mode = modes[arg]
        else:
            console.white("must provide a mode, 'guided' or 'loiter'")
        console.blank()

    def _heading_is_not_valid(self, heading):
        if heading < 1 or heading > 360:
            console.red("must provide a heading between 1 and 360 \n")
            return True
        return False

    def _vehicle_is_not_active(self):
        ''' check if vehicle is not active
            print message and return true if not active
            otherwise return false
        '''
        if self.vehicle.system_status.state != 'ACTIVE':
            console.red("vehicle must be ACTIVE \n")
            return True
        return False

    def _vehicle_is_not_in_guided_mode(self):
        ''' check if the vehicle is not in guided mode
            print message and return true if not active
            otherwise return false
        '''
        if self.vehicle.mode.name != 'GUIDED':
            console.red("vehicle must be in GUIDED mode \n")
            return True
        return False

    # flight control

    def do_launch(self, args):
        ''' arm and launch drone, loiter at provided altitude parameter (meters) '''
        if self.vehicle.system_status.state == 'ACTIVE':
            console.red("vehicle is already ACTIVE \n")
            return
        if not self.vehicle.is_armable:
            if self.vehicle.battery.voltage < 11.0:
                console.red("vehicle cannot be ARMED with low battery \n")
            elif self.vehicle.gps_0.fix_type != 3:
                console.red("vehicle cannot be ARMED without GPS fix \n")
            else:
                console.red("vehicle cannot be ARMED \n")
            return
        # arm and launch the vehicle
        console.white('begin launch sequence')
        console.white('... preflight checks')
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        if not self.vehicle.armed:
            self.vehicle.armed = True
            console.white('... wait for vehicle to arm')
            # wait for the vehicle to arm (15 seconds max)
            for _ in range(0, 5):
                self._wait()
                if self.vehicle.armed:
                    break
            else:
                console.white('... vehicle cannot be armed \n')
                return
        # issue launch command
        if self.vehicle.armed:
            altitude = 7
            self.vehicle.simple_takeoff(altitude)
            console.white('... liftoff & approach target altitude')
            # wait for drone to reach launch altitude
            while True:
                self._wait()
                if self.vehicle.location.global_relative_frame.alt >= (altitude * .95):
                    break
            console.white('... launch successful, hovering at {}m'.format(
                    str(self.vehicle.location.global_relative_frame.alt)
                ))
            # issue dummy move command to allow yaw control
            self.vehicle.simple_goto(self.vehicle.location.global_relative_frame)
            # save position for use w/ the home command
            home = self.vehicle.location.global_relative_frame
            self.home = LatLon(home.lat, home.lon)
        else:
            console.white('... an error occured while arming the vehicle')
        console.blank()

    def do_land(self, args):
        ''' set the drone to descend and land at the current location '''
        if self._vehicle_is_not_active():
            return
        console.white('begin land sequence')
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        console.white('... landing command issued')
        console.white('... approaching ground')
        while True:
            if not self.vehicle.armed:
                break
            self._wait()
        # success output
        console.white('... vehicle shutdown \n')

    def do_home(self, args):
        ''' move to launch position at current altitude '''
        console.white('update position to home at current altitude')
        current = self.vehicle.location.global_relative_frame
        home = self.home
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(home.lat, home.lon, current.alt),
                groundspeed=self.speed
            )
        console.white('... position update command issued \n')

    def do_return(self, args):
        ''' set the drone to RTL mode to execute automatic return/landing '''
        if self._vehicle_is_not_active():
            return
        console.white('signal vehicle for return and land')
        self.vehicle.mode = dronekit.VehicleMode('RTL')
        console.white('... RTL command issued \n')

    def do_position(self, args):
        ''' move to the location provided as --lat/--lng with optional --alt '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        location = self.vehicle.location.global_relative_frame
        # parse & verify target latitude/longitude
        latitude, longitude = arguments.position(args)
        if latitude is None or longitude is None:
            console.red('invalid params, must provide --lat/--lng \n')
            return
        else:
            # verify latitude/longitude provided is within 300m of current
            current = LatLon(location.lat, location.lon)
            new = LatLon(latitude, longitude)
            if (current.distance(new) * 1000) > self.range:
                console.red('new position is outside control range of {}m \n'.format(self.range))
                return
        # parse & verify target altitude, use current if not provided
        altitude = arguments.altitude(args)
        if altitude is not None:
            if altitude < 4 or altitude > 120:
                console.red('must provide an altitude between 5m and 120m \n')
                return
        else:
            altitude = location.alt
        # issue move command
        console.white('update vehicle position')
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(latitude, longitude, altitude),
                groundspeed=self.speed
            )
        console.white('... position update command issued \n')

    def _calculate_point_by_offset(self, position, heading, distance):
        ''' find offset from position (LatLon) by heading/distance (ints)
            --heading=X is degrees 1-360, --distance=X is meters
            returns lat, lng as floats in a tuple
        '''
        distance_as_km = float(distance) / 1000
        lat, lng = position.offset(heading, distance_as_km).to_string('D')
        return (float(lat), float(lng))

    def do_move(self, args):
        ''' move to position via --head/--dist and/or --alt '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        location = self.vehicle.location.global_relative_frame
        # parse arguments from console
        heading = arguments.heading(args)
        distance = arguments.distance(args)
        altitude = arguments.altitude(args)
        # verify provided parameters
        if (heading != None and distance == None) \
                or (heading == None and distance != None):
            console.red('must provide --head/--dist together, or not at all \n')
            return
        elif (heading == None and distance == None and altitude == None):
            console.red('must provide --head/--dist, and/or --alt params \n')
            return
        else:
            # verify heading is between 1 and 360
            if heading:
                if self._heading_is_not_valid(heading):
                    return
            # verify distance is greater than 5m and less than 200m
            if distance:
                if distance < 5 or distance > self.range:
                    console.red('must provide a distance between 5m and {}m \n'.format(self.range))
                    return
            # verify altitude is between 4m and 120m, use current if not provided
            if altitude is not None:
                if altitude < 4 or altitude > 120:
                    console.red('must provide a altitude between 5m and 120m \n')
                    return
            else:
                altitude = location.alt
        # calculate latitude/longitude position from params or use existing
        console.white('update vehicle position')
        if heading and distance:
            console.white('... calculate new position from parameters')
            current = LatLon(location.lat, location.lon)
            latitude, longitude = self._calculate_point_by_offset(current, heading, distance)
        else:
            latitude, longitude = (location.lat, location.lon)
        # issue move command
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(latitude, longitude, altitude),
                groundspeed=self.speed
            )
        console.white('... position update command issued \n')

    def do_yaw(self, args):
        ''' lock yaw at --head=X or --unlock '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # parse arguments from console
        heading = arguments.heading(args)
        if heading is not None:
            if self._heading_is_not_valid(heading):
                return
            console.white("locking vehicle yaw")
            self.vehicle.lock_yaw(heading)
        elif '--unlock' in args:
            console.white("unlocking vehicle yaw")
            self.vehicle.unlock_yaw()
        else:
            console.red("must provide a --head=X or --unlock parameter")
        console.blank()

    def do_camera(self, args):
        ''' trigger camera to capture a photo '''
        # check vehicle status and mode
        console.white('... capturing image')
        self.vehicle.trigger_camera()
        self._wait()    # wait 3 seconds for clean shot
        console.blank()



if __name__ == '__main__':

    # check for the --test flag from the command line
    test = True if '--test' in ' '.join(sys.argv) else False

    # if running in test mode, start the simulator
    if test:
        sitl = dronekit_sitl.start_default(lat=41.9751961, lon=-87.6636616)

    # enter the console application
    app = App(test=test)
    app.cmdloop()

    # if running in test mode, end the simulator
    if test:
        sitl.stop()

