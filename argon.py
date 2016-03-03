import os, sys, time, re
from cmd import Cmd
from LatLon import LatLon
import dronekit


class argsparse(object):
    ''' methods for parsing arguments from Cmd app console line args
        structured to provide interface similar to a module
    '''

    @staticmethod
    def _parse_arg_with_regex_to_typ(line, regex, typ):
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
    def count(cls, line):
        ''' parse --count argument to an int '''
        regex = r'count=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def delay(cls, line):
        ''' parse --delay argument to an int '''
        regex = r'delay=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def altitude(cls, line):
        ''' parse --altitude argument to an int '''
        regex = r'altitude=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def distance(cls, line):
        ''' parse --distance argument to an int '''
        regex = r'distance=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def heading(cls, line):
        ''' parse --heading argument to an int
            verify between 0 and 359, print message + return False
        '''
        regex = r'heading=(\d+)'
        heading = cls._parse_arg_with_regex_to_typ(line, regex, int)
        # validate a heading (not None), print error if fails
        if heading != None and heading <= 0 or heading >= 359:
            print 'must provide a valid heading between 0 and 359 \n'
            return False
        return heading

    @classmethod
    def interval(cls, line):
        ''' parse --interval argument to an int '''
        regex = r'interval=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def maximum(cls, line):
        ''' parse --maximum argument to an int '''
        regex = r'maximum=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def radius(cls, line):
        ''' parse --radius argument to an int '''
        regex = r'radius=(\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, int)

    @classmethod
    def latitude(cls, line):
        ''' parse --latitude argument to a float '''
        regex = r'latitude=(-?\d+.\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, float)

    @classmethod
    def longitude(cls, line):
        ''' parse --longitude argument to a float '''
        regex = r'longitude=(-?\d+.\d+)'
        return cls._parse_arg_with_regex_to_typ(line, regex, float)

    @classmethod
    def position(cls, line):
        ''' parse --lat/--lng/--alt arguments to a tuple '''
        # parse lat/lng/alt as floats, int
        latitude = cls._parse_arg_with_regex_to_typ(line, r'lat=(-?\d+.\d+)', float)
        longitude = cls._parse_arg_with_regex_to_typ(line, r'lng=(-?\d+.\d+)', float)
        altitude = cls._parse_arg_with_regex_to_typ(line, r'alt=(\d+)', int)
        # only return lat/lng as a pair, otherwise both as None
        if latitude == None or longitude == None:
            latitude, longitude = None, None
        return latitude, longitude, altitude


class App(Cmd):
    ''' Cmd-frameworked console application for controlling drone
        allow fine position control through command line interface
        intended to augment control of drone via radio
    '''

    prompt = '# '           # console prompt character prefix
    range_limit = 500       # 500m range
    min_alt = 3             # 3m-100m altitude envelope
    max_alt = 100
    heartbeat_timeout = 30  # 30 second timeout

    def cmdloop(self):
        try:
            Cmd.cmdloop(self)
        except KeyboardInterrupt:
            print   # handle a ctrl-C by moving to new/blank console line
            self.cmdloop()

    def __init__(self, test=False):
        Cmd.__init__(self)
        # clear incoming console, print intro banner
        os.system('clear')
        print '# argon : dronekit-based custom flight control console\n'
        # connect to drone, use connect param as function, exit if this fails
        print 'connecting to drone'
        try:
            print '... waiting on connection'
            if test == True:
                self.vehicle = dronekit.connect('tcp:127.0.0.1:5760',
                        status_printer=self._status_printer,
                        wait_ready=False,
                        heartbeat_timeout=self.heartbeat_timeout
                    )
            else:
                self.vehicle = dronekit.connect('/dev/cu.usbserial-DJ00DSDS',
                        baud=57600,
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

    def _status_printer(self, txt):
        return

    # console functionality

    def do_clear(self, args):
        ''' clear the console windows '''
        os.system('clear')

    def do_version(self, args):
        ''' print current version of argon and vehicle firmware version '''
        #self.vehicle.wait_ready('autopilot_version')
        print 'vehicle firmware : {}'.format(self.vehicle.version)
        print 'argon console : 1.0'
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
              armed (true or false)
        '''
        print 'system: {}'.format(self.vehicle.system_status.state)
        print 'mode: {}'.format(self.vehicle.mode.name)
        print 'armed: {}'.format(self.vehicle.armed)
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
        print 'speed: {}'.format(self.vehicle.airspeed)
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
        if arg not in modes.keys():
            print "must provide a mode, 'guided' or 'loiter'\n"
            return
        else:
            # update the vehicle mode
            print 'switching to {} mode'.format(arg)
            self.vehicle.mode = modes[arg]
            print

    def do_speed(self, arg):
        ''' set the vehicle speed to value between 1 and 4 '''
        # verify arg is an int, between 1 and 4
        # assign speed to vehicle

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
        # verify altitude doesn't exceed min/max, if not provided default to 10 m
        altitude = argsparse.altitude(args)
        if altitude is not None:
            if altitude > self.max_alt:
                print 'altitude exceeds maximum of {}m\n'.format(self.max_alt)
                return
            if altitude < self.min_alt:
                print 'altitude is below minimum of {}m\n'.format(self.max_alt)
                return
        else:
            altitude = 10
        # arm and launch the vehicle
        print 'begining launching sequence'
        print '... preflight checks and arm vehicle'
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        if not self.vehicle.armed:
            self.vehicle.armed = True
            time.sleep(5)   # an initial 5 second pause
            while not self.vehicle.armed:
                print '... waiting on vehicle to arm'
                time.sleep(3)
        if self.vehicle.armed:
            print '... liftoff and approach target altitude'
            self.vehicle.simple_takeoff(altitude)
            # verify drone reaching altitude before returning
            time.sleep(2)   # an initial 3 second pause
            while True:
                print '... approaching target altitude'
                current_altitude = self.vehicle.location.global_relative_frame.alt
                if current_altitude >= altitude * 0.95:
                    break
                time.sleep(3)
            # success output
            print '... launch successful, loitering at {}m'.format(
                    str(self.vehicle.location.global_relative_frame.alt)
                )
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
        while True:
            print '... approaching ground'
            current_altitude = self.vehicle.location.global_relative_frame.alt
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
        ''' move to the location provided as --lat/--lng, --alt optional '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # parse arguments and verify lat/lng, handle alt as optional
        loc = self.vehicle.location.global_relative_frame
        lat, lng, alt = argsparse.position(args)
        if lat is None or lng is None:
            print 'invalid params, must provide --lat/--lng, --alt optional\n'
            return
        else:
            # verify lat/lng provided, point is within 1000m of current
            current = LatLon(loc.lat, loc.lon)
            new = LatLon(lat, lng)
            if (current.distance(new) * 1000) > self.range_limit:
                print 'new position is outside control range\n'
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
        self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat, lng, alt))
        print '... position update command issued'
        print

    def do_move(self, args):
        ''' move to position via --heading/--distance and/or --altitude '''
        # check vehicle status and mode
        if self._vehicle_is_not_active():
            return
        if self._vehicle_is_not_in_guided_mode():
            return
        # parse arguments from console
        location = self.vehicle.location.global_relative_frame
        heading = argsparse.heading(args)
        distance = argsparse.distance(args)
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
                if heading <= 0 or heading >= 359:
                    print 'must provide a valid heading between 0 and 359 \n'
                    return
            # verify distance is less than 200 m
            if distance:
                if distance <= 1 or distance >= 200:
                    print 'must provide a valid distance between 1 and 200 m \n'
                    return
        # calculate lat/lng position from params or use existing
        print 'update vehicle position'
        if heading and distance:
            current = LatLon(location.lat, location.lon)
            lat, lng = self._calculate_offset(current, heading, distance)
            print '... calculate new position from parameters'
        else:
            lat, lng = (location.lat, location.lon)
            print '... calculate altitude at current position'
        # issue move command
        self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat, lng, alt))
        print '... position update command issued'
        print

    def _calculate_offset(self, position, heading, distance):
        ''' calculate the offset from position (a LatLon) by heading/distance (ints)
            heading is degrees 0-359, heading is meters
            returns lat, lng as floats in a tuple
        '''
        distance_as_km = float(distance) / 1000
        lat, lng = position.offset(heading, distance_as_km).to_string('D')
        return (float(lat), float(lng))

    def do_photo(self, args):
        ''' rotate vehicle to --heading, default 0, and take photo '''
        # check speed, error if moving
        if self.vehicle.airspeed > .02:
            print 'vehicle is in motion, cannot position camera \n '
            return
        # parse heading, verify value, default to
        heading = argsparse.heading(args)
        if heading:
            if heading <= 0 or heading >= 359:
                print 'must provide a valid heading between 0 and 359 \n'
                return
        else:
            heading = 0
        '''
        rotate vehicle to heading, wait for this to finish
        trigger camera
        release yaw control
        '''


if __name__ == '__main__':
    ''' the console application can be run directly from this file
        ./env/bin/python console.py

        to test with SITL, use two terminal windows

        run the drone simulator in terminal one
        ./sitl.sh

        run the console in test mode in terminal two
        ./env/bin/python console.py --test
    '''

    # if --test flag is provided, use sitl to simulate vehicle & connect
    test_flag = True if '--test' in ' '.join(sys.argv) else False

    App(test=test_flag).cmdloop()
