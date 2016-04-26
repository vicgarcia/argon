import os, sys, time, re, cmd
import LatLon as latlon
import dronekit
from pymavlink import mavutil


class argsparse(object):
    ''' methods for parsing arguments from Cmd app console line args
        structured to provide interface similar to a module
    '''

    @classmethod
    def _arg_parse_abstract(cls, line, regex, typ):
        ''' base functionality for argument parsing '''
        value = None
        try:
            match = re.search(regex, line)
            if match is not None:
                value = typ(match.group(1))
        except Exception as error:
            logging.error('error parsing argument : {}'.format(error.message))
        return value

    @classmethod
    def speed(cls, line):
        ''' parse --speed argument to a float '''
        return cls._arg_parse_abstract(line, r'speed=(\d+)', int)

    @classmethod
    def delay(cls, line):
        ''' parse --delay argument to an int '''
        return cls._arg_parse_abstract(line, r'delay=(\d+)', int)

    @classmethod
    def altitude(cls, line):
        ''' parse --altitude argument to an int '''
        return cls._arg_parse_abstract(line, r'altitude=(\d+)', int)

    @classmethod
    def distance(cls, line):
        ''' parse --distance argument to an int '''
        return cls._arg_parse_abstract(line, r'distance=(\d+)', int)

    @classmethod
    def heading(cls, line):
        ''' parse --heading argument to an int
            verify between 0 and 359, print message + return False
        '''
        return cls._arg_parse_abstract(line, r'heading=(\d+)', int)

    @classmethod
    def latitude(cls, line):
        ''' parse --latitude argument to a float '''
        return cls._arg_parse_abstract(line, r'latitude=(-?\d+.\d+)', float)

    @classmethod
    def longitude(cls, line):
        ''' parse --longitude argument to a float '''
        return cls._arg_parse_abstract(line, r'longitude=(-?\d+.\d+)', float)

    @classmethod
    def position(cls, line):
        ''' parse --lat/--lng/--alt arguments to a tuple '''
        # parse lat/lng/alt as floats, int
        latitude = cls._arg_parse_abstract(line, r'lat=(-?\d+.\d+)', float)
        longitude = cls._arg_parse_abstract(line, r'lng=(-?\d+.\d+)', float)
        altitude = cls._arg_parse_abstract(line, r'alt=(\d+)', int)
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
        # block forward execution to allow time photo to be taken
        time.sleep(3)


class App(cmd.Cmd):
    ''' Cmd-frameworked console application for controlling drone
        allow fine position control through command line interface
        intended to augment control of drone via radio
    '''

    prompt = '# '               # console prompt character prefix
    range_limit = 500           # 500m range
    min_alt = 3                 # 3m-100m altitude envelope
    max_alt = 100
    launch_alt = 20             # launch to this altitude
    base_speed = 4.0            # 4 m/s base speed
    heartbeat_timeout = 30      # 30 second timeout
    vehicle_class = Vehicle     # class to use for vehicle connection
    low_battery = 11.0          # vehicle cannot arm with battery below

    def cmdloop(self):
        try:
            cmd.Cmd.cmdloop(self)
        except KeyboardInterrupt:
            print   # handle a ctrl-C by moving to new/blank console line
            self.cmdloop()

    def __init__(self, test=False):
        cmd.Cmd.__init__(self)
        # clear incoming console, print intro banner
        os.system('clear')
        print '# argon : dronekit-based custom flight control console\n'
        # connect to drone, use connect param as function, exit if this fails
        print 'connecting to drone'
        try:
            print '... waiting on connection'
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
            print '... connected\n'
        except KeyboardInterrupt:
            print '... cancelling connection attempt'
            sys.exit(1)
        except Exception:
            print '... unable to connect'
            sys.exit(1)

    def default(self, args):
        print "unknown command, try 'help' for available commands"
        print

    def _status_printer(self, txt):
        return

    # console functionality

    def do_clear(self, args):
        ''' clear the console windows '''
        os.system('clear')

    def do_version(self, args):
        ''' print current version of argon and vehicle firmware version '''
        self.vehicle.wait_ready('autopilot_version')
        print 'argon console : 0.3.1'
        print 'vehicle firmware : {}'.format(self.vehicle.version)
        print

    def do_exit(self, args):
        ''' close connection to vehicle and exit console environment '''
        print 'closing connection\n'
        self.vehicle.close()
        sys.exit(1)

    def do_help(self, args):
        ''' print help text, all commands with options/details '''
        with open('help.txt', 'r') as help:
            print help.read()

    # vehicle state

    def do_status(self, args):
        ''' get system status from the vehicle
              system (active or standby)
              mode (guided, loiter, alt_hold, rtl, land)
        '''
        print 'system: {}'.format(self.vehicle.system_status.state)
        print 'mode: {}'.format(self.vehicle.mode.name)
        print

    def do_telemetry(self, args):
        ''' get telemetry data, once '''
        self._print_telemetry()
        print

    def do_monitor(self, args):
        ''' get telemetry data, continously, optional --delay '''
        # parse arguments
        delay = argsparse.delay(args)
        # enforce minimum of 3 second delay, default to 10 seconds
        if delay is not None:
            if delay < 3:
                delay = 3
        else:
            delay = 10
        # begin monitoring loop
        try:
            while True:
                self._print_telemetry()
                time.sleep(delay)
                print
        except KeyboardInterrupt:
            print
        print

    def _print_telemetry(self):
        ''' print the telemetry data from vehicle to the console '''
        location = self.vehicle.location.global_relative_frame
        print 'position: {}, {}'.format(location.lat, location.lon)
        print 'altitude: {}m'.format(location.alt)
        print 'heading: {}'.format(self.vehicle.heading)
        print 'airspeed: {}'.format(self.vehicle.airspeed)
        print 'battery: {} / {:.3}'.format(
                self.vehicle.battery.voltage,
                self.vehicle.parameters['FS_BATT_VOLTAGE']
            )

    def do_config(self, args):
        ''' get or set configuration parameters from the vehicle '''
        key, value = None, None
        # check for a key value argument pair
        args = args.split(' ')
        if len(args) == 2:
            key = args[0]
            value = args[1]
        if key and value:
            # update the parameter with provided key and value
            self.vehicle.parameters[key] = value
            print 'updated {} with {}'.format(key, value)
        else:
            # output all vehicle parameters with key and value
            for k, v in self.vehicle.parameters.iteritems():
                print '{} = {}'.format(k, v)
        print

    def do_mode(self, arg):
        ''' set vehicle mode to 'guided', 'loiter' '''
        # options as a dict, with slugs and VehicleMode object
        modes = {
                'loiter': dronekit.VehicleMode('LOITER'),
                'guided': dronekit.VehicleMode('GUIDED'),
            }
        # check that the provided arguments is an option
        if arg in modes.keys():
            print 'switching to {} mode'.format(arg)
            self.vehicle.mode = modes[arg]
        else:
            print "must provide a mode, 'guided' or 'loiter'"
        print

    def do_yaw(self, args):
        ''' lock yaw at --heading=X or --unlock '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # parse arguments from console
        heading = argsparse.heading(args)
        if heading:
            if self._heading_is_not_valid(heading):
                return
            print 'locking vehicle yaw'
            self.vehicle.lock_yaw(heading)
        else:
            if '--unlock' in args:
                print 'unlocking vehicle yaw'
                self.vehicle.unlock_yaw()
            else:
                print 'must provide a --heading=X or --unlock parameter'
        print

    def _heading_is_not_valid(self, heading):
        if heading < 0 or heading > 359:
            print 'must provide a valid heading between 0 and 359 \n'
            return True
        return False

    def _vehicle_is_not_active(self):
        ''' check if vehicle is not active
            print message and return true if not active
            otherwise return false
        '''
        if self.vehicle.system_status.state != 'ACTIVE':
            print 'vehicle must be ACTIVE\n'
            return True
        return False

    def _vehicle_is_not_in_guided_mode(self):
        ''' check if the vehicle is not in guided mode
            print message and return true if not active
            otherwise return false
        '''
        if self.vehicle.mode.name != 'GUIDED':
            print 'vehicle must be in GUIDED mode\n'
            return True
        return False

    # flight control

    def do_launch(self, args):
        ''' arm and launch drone, loiter at provided altitude parameter (meters) '''
        if self.vehicle.system_status.state == 'ACTIVE':
            print 'vehicle cannot already be ACTIVE\n'
            return
        if not self.vehicle.is_armable:
            if self.vehicle.battery.voltage < self.low_battery:
                print 'vehicle cannot be ARMED with low battery\n'
            elif self.vehicle.gps_0.fix_type != 3:
                print 'vehicle cannot be ARMED without GPS fix\n'
            else:
                print 'vehicle cannot be ARMED\n'
            return
        # arm and launch the vehicle
        print 'begining launching sequence'
        print '... preflight checks'
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        if not self.vehicle.armed:
            self.vehicle.armed = True
            print '... wait for vehicle to arm'
            while not self.vehicle.armed:
                time.sleep(3)       # wait 3 seconds between armed checks
        if self.vehicle.armed:
            try:
                print '... liftoff & approach target altitude'
                self.vehicle.simple_takeoff(self.launch_alt)
                # verify drone reaching altitude before returning
                while True:
                    current_altitude = \
                        self.vehicle.location.global_relative_frame.alt
                    if current_altitude >= self.launch_alt * 0.9:
                        break
                    time.sleep(3)   # wait 3 seconds between altitude checks
                # set base speed
                self.vehicle.groundspeed = self.base_speed
                # success output
                print '... launch successful, hovering at {}m'.format(
                        str(self.vehicle.location.global_relative_frame.alt)
                    )
            except KeyboardInterrupt:
                # override launch w/ ctrl-c, triggers emergency landing
                print '\n... abort takeoff, attempt emergency landing'
                self.vehicle.mode = dronekit.VehicleMode("LAND")
                print '... attempting to land'
        else:
            print '... an error occured while arming the vehicle'
        print

    def do_land(self, args):
        ''' set the drone to descend and land at the current location '''
        if self._vehicle_is_not_active():
            return
        print 'begin landing sequence'
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        print '... landing command issued'
        print '... approaching ground'
        while True:
            if not self.vehicle.armed:
                break
            time.sleep(7)
        # success output
        print '... landing successful, vehicle shutdown'
        print

    def do_return(self, args):
        ''' set the drone to RTL mode to execute automatic return/landing '''
        if self._vehicle_is_not_active():
            return
        print 'signal vehicle for return and land'
        self.vehicle.mode = dronekit.VehicleMode('RTL')
        print '... RTL command issued'
        print

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
            print 'invalid params, must provide --lat/--lng\n'
            return
        else:
            # verify lat/lng provided, point is within 1000m of current
            current = latlon.LatLon(loc.lat, loc.lon)
            new = latlon.LatLon(lat, lng)
            if (current.distance(new) * 1000) > self.range_limit:
                print 'new position is outside control range\n'
                return
        # parse speed argument, use default when not provided
        speed = argsparse.speed(args)
        if speed is None:
            speed = self.base_speed
        else:
            if speed < 1 or speed > 10:
                print 'invalid speed, must be between 1 and 10\n'
                return
        # verify altitude doesn't exceed min/max, if not provided use current
        if alt is not None:
            if alt > self.max_alt:
                print 'altitude exceeds maximum of {}m\n'.format(self.max_alt)
                return
            if alt < self.min_alt:
                print 'altitude is below minimum of {}m\n'.format(self.max_alt)
                return
        else:
            alt = loc.alt
        # issue move command
        print 'update vehicle position'
        print '... issue position update command'
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(lat, lng, alt),
                groundspeed=float(speed)
            )
        print

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
                print 'altitude exceeds maximum of {}m\n'.format(self.max_alt)
                return
            if alt < self.min_alt:
                print 'altitude is below minimum of {}m\n'.format(self.max_alt)
                return
        else:
            alt = location.alt
        # verify speed, use default when not provided
        if speed is None:
            speed = self.base_speed
        else:
            if speed < 1 or speed > 10:
                print 'invalid speed, must be between 1 and 10\n'
                return
        # must provide heading and distance, or neither, verify values
        if (heading != None and distance == None) \
                or (heading == None and distance != None):
            print 'must provide --head/--dist together, or not at all \n'
            return
        elif (heading == None and distance == None and alt == None):
            print 'must provide --head/--dist, and/or --alt params \n'
            return
        else:
            # verify heading is between 0 and 359
            if heading:
                if self._heading_is_not_valid(heading):
                    return
            # verify distance is less than 200 m
            if distance:
                if distance < 1 or distance > 200:
                    print 'must provide a valid distance between 1 and 200 m \n'
                    return
        # calculate lat/lng position from params or use existing
        print 'update vehicle position'
        if heading and distance:
            current = latlon.LatLon(location.lat, location.lon)
            lat, lng = self._find_position_by_offset(current, heading, distance)
            print '... calculate new position from parameters'
        else:
            lat, lng = (location.lat, location.lon)
            print '... calculate altitude at current position'
        # issue move command
        self.vehicle.simple_goto(
                dronekit.LocationGlobalRelative(lat, lng, alt),
                groundspeed=float(speed)
            )
        print '... position update command issued'
        print

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
        self.vehicle.trigger_camera()
        print

    # def do_circle(self, args):
    #     ''' fly a hex-shaped parth around a circle w/ ROI focus at a center '''
    #     pass

    # def do_grid(self, args):
    #     ''' demonstrate grid photo shoot at 40m, 4x4 photo grid '''
    #     pass



if __name__ == '__main__':
    ''' the console application can be run directly from this file
        ./env/bin/python argon.py

        to test with SITL, use two terminal windows

        run the drone simulator in terminal one
        ./env/bin/dronekit-sitl copter --home=41.9751961,-87.6636616,0,0

        run the console in test mode in terminal two
        ./env/bin/python argon.py --test
    '''

    test_flag = True if '--test' in ' '.join(sys.argv) else False
    App(test=test_flag).cmdloop()

