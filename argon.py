import cmd, os, sys, re, time
import dronekit
from pymavlink import mavutil
import LatLon as latlon
from clint.textui import puts, colored


class console(object):
    ''' methods for console output using clint components
        structured to provide interface similar to a module
    '''

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


class argsparse(object):
    ''' methods for parsing arguments from Cmd app console line args
        structured to provide interface similar to a module
    '''

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
    def speed(cls, line):
        ''' parse --speed argument to a float '''
        return cls._parse_abstract(line, r'speed=(\d+)', int)

    @classmethod
    def delay(cls, line):
        ''' parse --delay argument to an int '''
        return cls._parse_abstract(line, r'delay=(\d+)', int)

    @classmethod
    def altitude(cls, line):
        ''' parse --altitude argument to an int '''
        return cls._parse_abstract(line, r'altitude=(\d+)', int)

    @classmethod
    def distance(cls, line):
        ''' parse --distance argument to an int '''
        return cls._parse_abstract(line, r'distance=(\d+)', int)

    @classmethod
    def heading(cls, line):
        ''' parse --heading argument to an int
            verify between 0 and 359, print message + return False
        '''
        return cls._parse_abstract(line, r'heading=(\d+)', int)

    @classmethod
    def latitude(cls, line):
        ''' parse --latitude argument to a float '''
        return cls._parse_abstract(line, r'latitude=(-?\d+.\d+)', float)

    @classmethod
    def longitude(cls, line):
        ''' parse --longitude argument to a float '''
        return cls._parse_abstract(line, r'longitude=(-?\d+.\d+)', float)

    @classmethod
    def position(cls, line):
        ''' parse --lat/--lng/--alt arguments to a tuple '''
        # parse lat/lng/alt as floats, int
        latitude = cls._parse_abstract(line, r'lat=(-?\d+.\d+)', float)
        longitude = cls._parse_abstract(line, r'lng=(-?\d+.\d+)', float)
        altitude = cls._parse_abstract(line, r'alt=(\d+)', int)
        # only return lat/lng as a pair, otherwise both as None
        if latitude == None or longitude == None:
            latitude, longitude = None, None
        return latitude, longitude, altitude


class Vehicle(dronekit.Vehicle):
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
    ''' Cmd-frameworked console application for controlling drone
        allow fine position control through command line interface
        intended to augment control of drone via radio
    '''

    prompt = '# '               # console prompt character prefix
    range_limit = 500           # 500m range
    min_alt = 3                 # 3m-120m altitude envelope
    max_alt = 120
    launch_alt = 5              # 5m initial launch altitude
    base_speed = 3              # 3 m/s base speed
    heartbeat_timeout = 30      # 30 second timeout
    vehicle_class = Vehicle     # class to use for vehicle connection
    low_battery = 11.0          # vehicle cannot arm with battery below
    yaw_ready = False           # is the vehicle ready for yaw control

    def cmdloop(self):
        try:
            cmd.Cmd.cmdloop(self)
        except KeyboardInterrupt:
            console.blank()  # handle a ctrl-C by moving to new/blank console line
            self.cmdloop()

    def __init__(self, test=False):
        cmd.Cmd.__init__(self)
        console.clear()
        console.white("# argon : dronekit-based custom flight control console \n")
        console.white("connecting to drone")
        try:
            console.white("... waiting on connection")
            if test == True:
                self.vehicle = dronekit.connect('tcp:127.0.0.1:5760',
                        vehicle_class=self.vehicle_class,
                        status_printer=self._status_printer,
                        wait_ready=True,
                        heartbeat_timeout=self.heartbeat_timeout
                    )
            else:
                self.vehicle = dronekit.connect('/dev/cu.usbserial-DJ00DSDS',
                        baud=57600,
                        vehicle_class=self.vehicle_class,
                        status_printer=self._status_printer,
                        wait_ready=True,
                        heartbeat_timeout=self.heartbeat_timeout
                    )
            console.white("... connected \n")
        except KeyboardInterrupt:
            console.white("... canceling connection attempt \n")
            self._exit()
        except Exception:
            console.white("... unable to connect \n")
            self._exit()

    def default(self, args):
        console.red("unknown command, try 'help' for available commands \n")

    def _status_printer(self, txt):
        return

    def _exit(self):
        sys.exit(1)

    def _wait(self, delay=3):
        time.sleep(delay)

    # console functionality

    def do_clear(self, args):
        ''' clear the console window '''
        console.clear()

    def do_version(self, args):
        ''' print current version of argon and vehicle firmware version '''
        self.vehicle.wait_ready('autopilot_version')
        console.white("console console : 0.5.0")
        console.white("vehicle firmware : {}".format(self.vehicle.version))
        console.blank()

    def do_exit(self, args):
        ''' close connection to vehicle and exit console environment '''
        console.white("closing connection \n")
        self.vehicle.close()
        self._exit()

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
        ''' get telemetry data, once '''
        self._print_telemetry()
        console.blank()

    def do_monitor(self, args):
        ''' get telemetry data, continously every 4 seconds '''
        # begin monitoring loop
        try:
            while True:
                self._print_telemetry()
                self._wait(4)   # hard-coded 4 second delay
                console.blank()
        except KeyboardInterrupt:
            console.blank()
            # nb : these uses of new lines is for ctrl-c behavior
        console.blank()

    def _print_telemetry(self):
        ''' print the telemetry data from vehicle to the console '''
        location = self.vehicle.location.global_relative_frame
        console.white("position: {}, {}".format(location.lat, location.lon))
        console.white("altitude: {}m".format(location.alt))
        console.white("heading: {}".format(self.vehicle.heading))
        console.white("airspeed: {}".format(self.vehicle.airspeed))
        console.white("battery: {} / {:.3}".format(
                self.vehicle.battery.voltage,
                self.vehicle.parameters['FS_BATT_VOLTAGE']
            ))

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

    def do_yaw(self, args):
        ''' lock yaw at --heading=X or --unlock '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # the vehicle yaw cannot be managed after launch until it moves
        # this is a thing in the arducopter firmware
        # argon tracks/guards/reports this for UX
        if not self.yaw_ready:
            console.red("yaw not ready, must move vehicle first")
            return
        # parse arguments from console
        heading = argsparse.heading(args)
        if heading:
            if self._heading_is_not_valid(heading):
                return
            console.white("locking vehicle yaw")
            self.vehicle.lock_yaw(heading)
        else:
            if '--unlock' in args:
                console.white("unlocking vehicle yaw")
                self.vehicle.unlock_yaw()
            else:
                console.white("must provide a --heading=X or --unlock parameter")
        console.blank()

    def _heading_is_not_valid(self, heading):
        if heading < 0 or heading > 359:
            console.red("must provide a heading between 0 and 359 \n")
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
            if self.vehicle.battery.voltage < self.low_battery:
                console.red("vehicle cannot be ARMED with low battery \n")
            elif self.vehicle.gps_0.fix_type != 3:
                console.red("vehicle cannot be ARMED without GPS fix \n")
            else:
                console.red("vehicle cannot be ARMED \n")
            return
        # arm and launch the vehicle
        console.white('begining launching sequence')
        console.white('... preflight checks')
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        if not self.vehicle.armed:
            self.vehicle.armed = True
            console.white('... wait for vehicle to arm')
            maximum_wait = 7
            wait_count = 0
            while not self.vehicle.armed:
                self._wait()
                wait_count += 1
                # this is mostly to handle safety switch not engaged
                if wait_count == maximum_wait:
                    console.white('...aborting launch \n')
                    console.red('vehicle cannot be ARMED \n')
                    return
        if self.vehicle.armed:
            try:
                console.white('... liftoff & approach target altitude')
                self.vehicle.simple_takeoff(self.launch_alt)
                # verify drone reaching altitude before returning
                while True:
                    current_altitude = \
                        self.vehicle.location.global_relative_frame.alt
                    if current_altitude >= self.launch_alt * 0.9:
                        break
                    self._wait()
                # set base speed
                self.vehicle.groundspeed = self.base_speed
                # we are not yaw ready after launch
                self.yaw_ready = False
                # success output
                console.white('... launch successful, hovering at {}m'.format(
                        str(self.vehicle.location.global_relative_frame.alt)
                    ))
            except KeyboardInterrupt:
                # override launch w/ ctrl-c, triggers emergency landing
                console.blank()     # blank line after the ctrl-C (^C in console)
                console.red("... abort takeoff, attempt emergency landing")
                self.vehicle.mode = dronekit.VehicleMode("LAND")
                console.red("... attempting to land")
        else:
            console.red('... an error occured while arming the vehicle')
        console.blank()

    def do_land(self, args):
        ''' set the drone to descend and land at the current location '''
        if self._vehicle_is_not_active():
            return
        console.white('begin landing sequence')
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        console.white('... landing command issued')
        console.white('... approaching ground')
        while True:
            if not self.vehicle.armed:
                break
            self._wait(7)
        # success output
        console.white('... landing successful, vehicle shutdown \n')

    def do_return(self, args):
        ''' set the drone to RTL mode to execute automatic return/landing '''
        if self._vehicle_is_not_active():
            return
        console.white('signal vehicle for return and land')
        self.vehicle.mode = dronekit.VehicleMode('RTL')
        console.white('... RTL command issued \n')

    def do_position(self, args):
        ''' move to the location provided as --lat/--lng and await arrival
            provide option --speed=X (1-10) and --alt=X (w/in range)
        '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # parse arguments and verify lat/lng, handle alt as optional
        loc = self.vehicle.location.global_relative_frame
        lat, lng, alt = argsparse.position(args)
        if lat is None or lng is None:
            console.white('invalid params, must provide --lat/--lng \n')
            return
        else:
            # verify lat/lng provided, point is within 1000m of current
            current = latlon.LatLon(loc.lat, loc.lon)
            new = latlon.LatLon(lat, lng)
            if (current.distance(new) * 1000) > self.range_limit:
                console.white('new position is outside control range \n')
                return
        # parse speed argument, use default when not provided
        speed = argsparse.speed(args)
        if speed is None:
            speed = self.base_speed
        else:
            if speed < 1 or speed > 10:
                console.white('invalid speed, must be between 1 and 10 \n')
                return
        # verify altitude doesn't exceed min/max, if not provided use current
        if alt is not None:
            if alt > self.max_alt:
                console.white('altitude exceeds maximum of {}m \n'.format(self.max_alt))
                return
            if alt < self.min_alt:
                console.white('altitude is below minimum of {}m \n'.format(self.max_alt))
                return
        else:
            alt = loc.alt
        # issue move command
        console.white('update vehicle position')
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(lat, lng, alt),
                groundspeed=float(speed)
            )
        self.yaw_ready = True
        console.white('... position update command issued \n')

    def do_move(self, args):
        ''' move to position via --heading/--distance and/or --altitude
            provide optionally --speed=X (1-10)
        '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # parse arguments from console
        location = self.vehicle.location.global_relative_frame
        heading = argsparse.heading(args)
        distance = argsparse.distance(args)
        speed = argsparse.speed(args)
        alt = argsparse.altitude(args)
        # verify altitude, or use current if not provided
        if alt is not None:
            if alt > self.max_alt:
                console.white('altitude exceeds maximum of {}m \n'.format(self.max_alt))
                return
            if alt < self.min_alt:
                console.white('altitude is below minimum of {}m \n'.format(self.max_alt))
                return
        else:
            alt = location.alt
        # verify speed, use default when not provided
        if speed is None:
            speed = self.base_speed
        else:
            if speed < 1 or speed > 10:
                console.white('invalid speed, must be between 1 and 10 \n')
                return
        # must provide heading and distance, or neither, verify values
        if (heading != None and distance == None) \
                or (heading == None and distance != None):
            console.white('must provide --head/--dist together, or not at all \n')
            return
        elif (heading == None and distance == None and alt == None):
            console.white('must provide --head/--dist, and/or --alt params \n')
            return
        else:
            # verify heading is between 0 and 359
            if heading:
                if self._heading_is_not_valid(heading):
                    return
            # verify distance is less than 200 m
            if distance:
                if distance < 1 or distance > 200:
                    console.white('must provide a valid distance between 1 and 200 m \n')
                    return
        # calculate lat/lng position from params or use existing
        console.white('update vehicle position')
        if heading and distance:
            current = latlon.LatLon(location.lat, location.lon)
            lat, lng = self._find_position_by_offset(current, heading, distance)
            console.white('... calculate new position from parameters')
        else:
            lat, lng = (location.lat, location.lon)
            console.white('... calculate altitude at current position')
        # issue move command
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(lat, lng, alt),
                groundspeed=float(speed)
            )
        self.yaw_ready = True
        console.white('... position update command issued \n')

    def _find_position_by_offset(self, position, heading, distance):
        ''' find offset from position (LatLon) by heading/distance (ints)
            --heading=X is degrees 0-359, --distance=X is meters
            returns lat, lng as floats in a tuple
        '''
        distance_as_km = float(distance) / 1000
        lat, lng = position.offset(heading, distance_as_km).to_string('D')
        return (float(lat), float(lng))

    def do_camera(self, args):
        ''' trigger camera to capture a photo '''
        # check vehicle status and mode
        console.white('... capturing image')
        self.vehicle.trigger_camera()
        self._wait()    # wait 3 seconds for clean shot
        console.blank()



if __name__ == '__main__':
    test_flag = True if '--test' in ' '.join(sys.argv) else False
    App(test=test_flag).cmdloop()

