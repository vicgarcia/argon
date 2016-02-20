import argsparse
from LatLon import LatLon


def do_grid(args):
    ''' calculate grid markers based on --latitude/--longitude,
        when provided, use --interval/--maximum to determine points
    '''
    directions = [ ('north', 0), ('east', 90), ('south', 180), ('west', 270) ]

    # get lat/lng from args
    latitude = argsparse.latitude(args)
    longitude = argsparse.longitude(args)
    if latitude == None or longitude == None:
        print 'must provide valid --latitude and --longitude arguments'
        return

    # parse interval/maximum or work with 30/150 as defaults
    interval = argsparse.interval(args)
    if interval is None:
        interval = 30.0
    else:
        interval = float(interval)
    maximum = argsparse.maximum(args)
    if maximum is None:
        maximum = 150.0
    else:
        maximum = float(maximum)

    print
    position = LatLon(latitude, longitude)
    # iterate over directions, print points at interval to maximum
    for name, heading in directions:
        current = 0.0
        print name
        while current < maximum:
            current += interval  # starts at 0, add at top of iteration
            lt, lg = position.offset(heading, current/1000).to_string('D')
            print '{}m : {}, {}'.format(current, lt, lg)
        print

