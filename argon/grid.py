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


