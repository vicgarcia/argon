import re


def _parse(line, regex, typ):
    ''' base functionality for argument parsing via regex '''
    value = None
    try:
        match = re.search(regex, line)
        if match is not None:
            value = typ(match.group(1))
    except Exception:
        pass
    return value


def altitude(line):
    ''' parse --alt argument to an int '''
    return _parse(line, r'alt=(\d+)', int)


def distance(line):
    ''' parse --dist argument to an int '''
    return _parse(line, r'dist=(\d+)', int)


def heading(line):
    ''' parse --head argument to an int '''
    return _parse(line, r'head=(\d+)', int)


def position(line):
    ''' parse --lat/--lng arguments to a tuple '''
    # parse lat/lng/alt as floats, int
    latitude = _parse(line, r'lat=(-?\d+.\d+)', float)
    longitude = _parse(line, r'lng=(-?\d+.\d+)', float)
    # only return lat/lng as a pair, otherwise both as None
    if latitude == None or longitude == None:
        latitude, longitude = None, None
    return latitude, longitude
