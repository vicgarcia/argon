import os, sys, time, re
from cmd import Cmd
from LatLon import LatLon
from dronekit import VehicleMode, LocationGlobalRelative
import argsparse


class App(Cmd):

    prompt = '# '
    range_limit = 1000
    launch_alt = 5
    min_alt = 3
    max_alt = 100

    def cmdloop(self):
        try:
            Cmd.cmdloop(self)
        except KeyboardInterrupt:
            # handle a ctrl-C by moving to new/blank console line
            print
            self.cmdloop()

    # application startup

    def __init__(self, connect):
        Cmd.__init__(self)
        # clear incoming console, print intro banner
        os.system('clear')
        print '# argon : dronekit-based custom flight control console\n'
        # connect to drone with connect param, exit if this fails
        print 'connecting to drone'
        try:
            print '... waiting on connection'
            self.vehicle = connect()
            print '... connected\n'
        except Exception, e:
            print '... unable to connect\n'
            sys.exit(1)
        except KeyboardInterrupt:
            print '... cancelling connection attempt\n'
            sys.exit(1)

    # application functionality

    def do_clear(self, args):
        ''' clear the console windows '''
        os.system('clear')

    def do_version(self, args):
        ''' print current version of argon and vehicle firmware version '''
        print 'argon version 1.0'
        self.vehicle.wait_ready('autopilot_version')
        print 'vehicle firmware version {}'.format(self.vehicle.version)
        print self.prompt
        print

    def do_exit(self, args):
        ''' close connection to vehicle and exit console environment '''
        print 'closing connection'
        self.vehicle.close()
        print
        sys.exit(1)

    def do_help(self, args):
        with open('help.txt', 'r') as help:
            print help.read()

    # vehicle state

    def do_status(self, args):
        ''' get system status from the vehicle '''
        print 'system: {}'.format(self.vehicle.system_status.state)
        print 'mode: {}'.format(self.vehicle.mode.name)
        print 'armed: {}'.format(self.vehicle.armed)
        print

    def do_telemetry(self, args):
        ''' get telemetry data from the vehicle, accepts --count/--delay '''
        # parse arguments
        count = argsparse.count(args)
        delay = argsparse.delay(args)
        # enforce minimum count of 1
        if count == None:
            count = 1
        # enforce minimum of 3 second delay
        if delay < 3:
            delay = 3
        for i in range(count):
            location = self.vehicle.location.global_relative_frame
            print 'position: {}, {}'.format(location.lat, location.lon)
            print 'altitude: {}m'.format(location.alt)
            print 'heading: {}'.format(self.vehicle.heading)
            print 'speed: {}'.format(self.vehicle.airspeed)
            print 'battery: {}'.format(self.vehicle.battery.level)
            print
            # don't pause for delay on the last iteration
            if (i + 1) < count:
                time.sleep(delay)

    def do_config(self, args):
        ''' get or set configuration parameters from the vehicle '''
        key, value = None, None
        # check for a key value argument pair
        args = line.split(' ')
        if len(args) == 2:
            key = args[0]
            value = args[2]
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
        ''' set vehicle mode to 'guided', 'loiter', 'standard' (alt hold) '''
        modes = {
                'standard':  VehicleMode('ALT_HOLD'),
                'loiter':    VehicleMode('LOITER'),
                'guided':    VehicleMode('GUIDED'),
            }
        if arg not in [ 'loiter', 'guided', 'standard' ]:
            print "must provide a mode, 'standard', 'guided', or 'loiter'\n"
            return
        else:
            print 'switching to {} mode'.format(arg)
            self.vehicle.mode = modes[arg]
            print

    # flight control

    def do_launch(self, args):
        ''' arm and launch drone, loiter at provided altitude parameter (meters) '''
        if self.vehicle.system_status.state == 'ACTIVE':
            print 'vehicle cannot already be ACTIVE\n'
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
        print

    def do_land(self, args):
        ''' set the drone to descend and land at the current location '''
        if self.vehicle.system_status.state != 'ACTIVE':
            print 'vehicle must be ACTIVE\n'
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
        print

    def do_return(self, args):
        ''' set the drone to RTL mode to execute automatic return/landing '''
        if self.vehicle.system_status.state != 'ACTIVE':
            print 'vehicle must be ACTIVE\n'
            return
        print 'signal vehicle for return and land'
        self.vehicle.mode = VehicleMode('RTL')
        print '... RTL command issued'
        print

    def do_position(self, args):
        ''' move to the location provided as --lat/--lng, --alt optional '''
        # pre-check vehicle mode and status
        if self.vehicle.system_status.state != 'ACTIVE' \
                and self.vehicle.mode.name != 'GUIDED':
            print 'vehicle must be ACTIVE and in GUIDED mode\n'
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
        self.vehicle.simple_goto(LocationGlobalRelative(lat, lng, alt))
        print '... position update command issued'
        print

    def do_move(self, args):
        ''' move to position via --heading/--distance and/or --altitude '''
        # check vehicle mode and status
        if self.vehicle.system_status.state != 'ACTIVE' \
                and self.vehicle.mode.name != 'GUIDED':
            print 'vehicle must be ACTIVE and in GUIDED mode\n'
            return
        # parse arguments
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
                if heading < 0 or heading > 369:
                    print 'must provide a valid heading between 0 and 359 \n'
                    return
            # verify distance is less than 200 m
            if distance:
                if distance < 1 or distance > 200:
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
            print '... calculate new altitude'
        # issue move command
        self.vehicle.simple_goto(LocationGlobalRelative(lat, lng, alt))
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
        ''' rotate vehicle to --head and take photo '''
        pass

