import os, sys, time, re
from cmd import Cmd
from LatLon import LatLon
from dronekit import VehicleMode, LocationGlobalRelative


class App(Cmd):

    prompt = '# '

    def cmdloop(self):
        try:
            Cmd.cmdloop(self)
        except KeyboardInterrupt:
            # handle a ctrl-C by moving to new/blank console line
            print '\n'
            self.cmdloop()

    # application startup

    def __init__(self, connect):
        Cmd.__init__(self)
        # define flight params, in meters
        self.range_limit = 1000
        self.launch_alt = 15   # XXX paramaterize these
        self.min_alt = 5
        self.max_alt = 100
        # clear incoming console, print intro banner
        os.system('clear')
        print '# argon : dronekit-based custom flight control console \n'
        # connect to drone with connect param, exit if this fails
        print 'connecting to drone'
        try:
            print '... waiting on connection'
            self.vehicle = connect()
            print '... connected \n'
        except Exception, e:
            print '... unable to connect \n'
            sys.exit(1)
        except KeyboardInterrupt:
            print '... cancelling connection attempt \n'
            sys.exit(1)
        # help hint above initial prompt
        print "use 'help' to get information about available commands \n"

    # application functionality

    def do_clear(self, args):
        ''' clear the console windows '''
        os.system('clear')

    def do_version(self, args):
        ''' print current version of argon and vehicle firmware version '''
        print
        print 'argon version 1.0'
        self.vehicle.wait_ready('autopilot_version')
        print 'vehicle firmware version {}'.format(self.vehicle.version)
        print '\n'

    def do_exit(self, args):
        ''' close connection to vehicle and exit console environment '''
        print
        print 'closing connection'
        self.vehicle.close()
        print '\n'
        sys.exit(1)

    def do_help(self, args):
        print
        print 'available commands'
        print
        print 'help'
        print '  display this text'
        print
        print 'version'
        print '  print the argon version number'
        print
        print 'clear'
        print '  clear console history'
        print
        print 'exit'
        print '  exit the console app'
        print
        print 'status'
        print '  get status of connected drone'
        print
        print 'telemetry --count=# --delay=#'
        print '  print vehicle telemetry date, optionally repeat at intervals'
        print '  --count is the number of times to retrieve the telemetry data'
        print '  --delay is the ammount of time to wait between getting the data'
        print '  the execution of the telemetry command can be broken with CTRL-C'
        print
        print 'config <KEY> <VALUE>'
        print '  update or print all configuration values from vehicle'
        print '  providing a KEY and VALUE will update that parameter'
        print '  all current vehicle parameters are outputted when not'
        print
        print 'launch'
        print '  initiate the launch sequence for the vehicle'
        print '  requires that vehicle is not already active'
        print
        print 'land'
        print '  initiate the landing sequence for the vehicle'
        print '  requires that the vehicle is active'
        print '  this will land vehicle at current position, see return'
        print
        print 'position --lat=# --lng=# --alt=#'
        print '  signal vehicle to move to a lat/lng position'
        print '  optionally priovide an altitude to use at final position'
        print '  vehicle will maintain current altitude if none provided'
        print
        print 'move --head=# --dist=# --alt=#'
        print '  signal vehicle to move a distance, along a heading, to an altitude'
        print '  must provide head/dist together, optionally provide altitude'
        print '  providing only an altitude will change vehicle altitude'
        print '  final position is calculated and signalled to vehicle'
        print '  vehicle will maintain current altitude if none provided'
        print
        print 'return'
        print '  signal vehicle to return to home and land'
        print '  requires that the vehicle is active'
        print '  this command will not block while the vehicle responds'
        print
        print

    # vehicle overview

    def do_status(self, args):
        ''' get system status from the vehicle '''
        print
        print 'system: {}'.format(self.vehicle.system_status.state)
        print 'mode: {}'.format(self.vehicle.mode.name)
        print 'armed: {}'.format(self.vehicle.armed)
        print '\n'

    def do_telemetry(self, args):
        ''' get telemetry data from the vehicle, accepts --count/--delay '''
        count, delay = self._parse_countdelay_arg(args)
        for i in range(count):
            location = self.vehicle.location.global_relative_frame
            print
            print 'position: {}, {}'.format(location.lat, location.lon)
            print 'altitude: {}m'.format(location.alt)
            print 'heading: {}'.format(self.vehicle.heading)
            print 'speed: {}'.format(self.vehicle.airspeed)
            if (i + 1) != count:
                time.sleep(delay)
        print '\n'

    def do_config(self, args):
        ''' get or set configuration parameters from the vehicle '''
        print
        key, value = self._parse_config_args(args)
        if key and value:
            self.vehicle.parameters[key] = value
            print 'updated {} with {}'.format(key, value)
        else:
            for k, v in self.vehicle.parameters.iteritems():
                print '{} = {}'.format(k, v)
        print '\n'

    # flight control

    def do_launch(self, args):
        ''' arm and launch drone, loiter at provided altitude parameter (meters) '''
        print
        if self.vehicle.system_status.state == 'ACTIVE':
            print 'vehicle cannot already be ACTIVE'
            print '\n'
            return
        print 'begining launching sequence'
        print '... preflight checks and arm vehicle'
        if not self.vehicle.armed:
            # arm the vehicle and verify
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            time.sleep(3)   # an initial 3 second pause
            while not self.vehicle.armed:
                print '... waiting on vehicle to arm'
                time.sleep(3)
        if self.vehicle.armed:
            print '... liftoff and approach target altitude'
            self.vehicle.simple_takeoff(self.launch_alt)
            # verify drone reaching altitude before returning
            time.sleep(2)   # an initial 3 second pause
            while True:
                print '... approaching target altitude'
                current_altitude = self.vehicle.location.global_relative_frame.alt
                if current_altitude >= self.launch_alt * 0.95:
                    break
                time.sleep(3)
            # success output
            print '... launch successful, loitering at {}m'.format(
                    str(self.vehicle.location.global_relative_frame.alt)
                )
        else:
            print '... an error occured while arming the vehicle'
        print '\n'

    def do_land(self, args):
        ''' set the drone to descend and land at the current location '''
        print
        if self.vehicle.system_status.state != 'ACTIVE':
            print 'vehicle must be ACTIVE'
            print '\n'
            return
        print 'begin landing sequence'
        self.vehicle.mode = VehicleMode("LAND")
        print '... landing command issued'
        while True:
            print '... approaching ground'
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if not self.vehicle.armed:
                break
            time.sleep(7)
        # success output
        print '... landing successful, vehicle shutdown'
        print '\n'

    def do_return(self, args):
        ''' set the drone to RTL mode to execute automatic return/landing '''
        print
        if self.vehicle.system_status.state != 'ACTIVE':
            print 'vehicle must be ACTIVE'
            print '\n'
            return
        print 'signal vehicle for return and land'
        self.vehicle.mode = VehicleMode('RTL')
        print '... RTL command issued'
        print '\n'

    def do_position(self, args):
        ''' move to the location provided as --lat/--lng, --alt optional '''
        print
        # check vehicle mode and status
        if self.vehicle.system_status.state != 'ACTIVE' \
                and self.vehicle.mode.name != 'GUIDED':
            print 'vehicle must be ACTIVE and in GUIDED mode \n'
            return
        # parse arguments
        location = self.vehicle.location.global_relative_frame
        lat, lng = self._parse_latlng_arg(args)
        alt = self._parse_alt_arg(args)
        # verify lat/lng provided, point is within 1000m of current
        if lat is None or lng is None:
            print 'invalid params, must provide --lat/--lng, --alt optional \n'
            return
        else:
            current = LatLon(location.lat, location.lon)
            update = LatLon(lat, lng)
            if (current.distance(update) * 1000) > self.range_limit:
                print 'new position is outside control range \n'
                return
        # verify altitude doesn't exceed min/max, if not provided use current
        if alt is not None:
            if alt > self.max_alt:
                print 'altitude exceeds maximum of {}m \n'.format(self.max_alt)
                return
            if alt < self.min_alt:
                print 'altitude is below minimum of {}m \n'.format(self.max_alt)
                return
        else:
            alt = location.alt
        # issue move command
        print 'update vehicle position'
        self.vehicle.simple_goto(LocationGlobalRelative(lat, lng, alt))
        print '... position update command issued'
        print '\n'

    def do_move(self, args):
        ''' move to position via --head/--dist and/or --alt '''
        # check vehicle mode and status
        if self.vehicle.system_status.state != 'ACTIVE' \
                and self.vehicle.mode.name != 'GUIDED':
            print ' vehicle must be ACTIVE and in GUIDED mode'
            print '\n'
            return
        # parse arguments
        location = self.vehicle.location.global_relative_frame
        heading = self._parse_heading_arg(args)
        distance = self._parse_distance_arg(args)
        alt = self._parse_alt_arg(args)
        # verify altitude, or use current if not provided
        if alt is not None:
            if alt > self.max_alt:
                print ' altitude exceeds maximum of {}m\n'.format(self.max_alt)
                return
            if alt < self.min_alt:
                print ' altitude is below minimum of {}m\n'.format(self.max_alt)
                return
        else:
            alt = location.alt
        # must provide heading and distance, or neither, verify values
        if (heading != None and distance == None) \
                or (heading == None and distance != None):
            print ' must provide --head/--dist together, or not at all \n'
            return
        elif (heading == None and distance == None and alt == None):
            print ' must provide --head/--dist, and/or --alt params \n'
            return
        else:
            # verify heading is between 0 and 359
            if heading:
                if heading < 0 or heading > 369:
                    print ' must provide a valid heading between 0 and 359 \n'
                    return
            # verify distance is less than 200 m
            if distance:
                if distance < 1 or distance > 200:
                    print ' must provide a valid distance between 1 and 200 m \n'
                    return
        # calculate lat/lng position from params or use existing
        print ' update vehicle position'
        if heading and distance:
            current = LatLon(location.lat, location.lon)
            lat, lng = self._calculate_offset(current, heading, distance)
            print '  ... calculate new position from parameters'
        else:
            lat, lng = (location.lat, location.lon)
            print '  ... calculate new altitude'
        # issue move command
        self.vehicle.simple_goto(LocationGlobalRelative(lat, lng, alt))
        print '  ... position update command issued'
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
        ''' rotate vehicle to --head and take photo '''
        pass

    # grid planning

    def do_grid(self, args):
        ''' calculate grid markers based on --lat/--lng, --int/--max '''
        print
        directions = [('north', 0), ('east', 90), ('south', 180), ('west', 270)]
        # XXX sort this out, also position below
        #lat, lng = self._parse_latlng_arg(args)
        location = self.vehicle.location.global_relative_frame
        position = LatLon(location.lat, location.lon)
        # parse and verify, or use default
        #maximum = self._parse_maximum_arg(args)
        maximum = 120.0
        #interval = self._parse_interval_arg(args)
        interval = 30.0
        print 'center\n{}, {}\n'.format(location.lat, location.lon)
        for name, heading in directions:
            current = 0.0
            print name
            while current < maximum:
                current += interval  # starts at 0, add at top of iteration
                lt, lg = position.offset(heading, current/1000).to_string('D')
                print '{}m : {}, {}'.format(current, lt, lg)
            print
        print

    # argument parsing helpers

    def _parse_countdelay_arg(self, line):
        ''' helper method to parse count/delay arguments '''
        # parse count
        count = 1
        try:
            match = re.search(r'count=(\d+)', line)
            if match is not None:
                count = int(match.group(1))
        except:
            pass
        # parse delay
        delay = 0
        try:
            match = re.search(r'delay=(\d+)', line)
            if match is not None:
                delay = int(match.group(1))
        except:
            pass
        # enforce a min 3 second delay
        if count > 1 and delay < 3:
            delay = 3
        return ( count, delay )

    def _parse_latlng_arg(self, line):
        ''' helper method to parse lat/lng arguments '''
        # parse latitude
        latitude = None
        try:
            match = re.search(r'lat=(-?\d+.\d+)', line)
            if match is not None:
                latitude = float(match.group(1))
        except:
            pass
        # parse longitude
        longitude = None
        try:
            match = re.search(r'lng=(-?\d+.\d+)', line)
            if match is not None:
                longitude = float(match.group(1))
        except:
            pass
        # return as (lat, lng) tuple, float or None
        return ( latitude, longitude )

    def _parse_alt_arg(self, line):
        ''' helper method to parse alt argument '''
        altitude = None
        try:
            match = re.search(r'alt=(\d+)', line)
            if match is not None:
                altitude = float(match.group(1))
        except:
            pass
        return altitude

    def _parse_distance_arg(self, line):
        ''' helper method to parse alt argument '''
        distance = None
        try:
            match = re.search(r'dist=(\d+)', line)
            if match is not None:
                distance = int(match.group(1))
        except:
            pass
        return distance

    def _parse_heading_arg(self, line):
        ''' helper method to parse alt argument '''
        heading = None
        try:
            match = re.search(r'head=(\d+)', line)
            if match is not None:
                heading = int(match.group(1))
        except:
            pass
        return heading

    def _parse_interval_arg(self, line):
        ''' helper method to parse alt argument '''
        interval = None
        try:
            match = re.search(r'int=(\d+)', line)
            if match is not None:
                interval = int(match.group(1))
        except:
            pass
        return interval

    def _parse_maximum_arg(self, line):
        ''' helper method to parse alt argument '''
        maximum = None
        try:
            match = re.search(r'max=(\d+)', line)
            if match is not None:
                maximum = int(match.group(1))
        except:
            pass
        return maximum

    def _parse_config_args(self, line):
        key, value = None, None
        args = line.split(' ')
        if len(args) == 2:
            key = args[0]
            value = args[2]
        return key, value

