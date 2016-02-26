import re
import logging


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

def count(line):
    ''' parse --count argument to an int '''
    regex = r'count=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def delay(line):
    ''' parse --delay argument to an int '''
    regex = r'delay=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def altitude(line):
    ''' parse --altitude argument to an int '''
    regex = r'altitude=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def distance(line):
    ''' parse --distance argument to an int '''
    regex = r'distance=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def heading(line):
    ''' parse --heading argument to an int '''
    regex = r'heading=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def interval(line):
    ''' parse --interval argument to an int '''
    regex = r'interval=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def maximum(line):
    ''' parse --maximum argument to an int '''
    regex = r'maximum=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def radius(line):
    ''' parse --radius argument to an int '''
    regex = r'radius=(\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, int)

def latitude(line):
    ''' parse --latitude argument to a float '''
    regex = r'latitude=(-?\d+.\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, float)

def longitude(line):
    ''' parse --longitude argument to a float '''
    regex = r'longitude=(-?\d+.\d+)'
    return _parse_arg_with_regex_to_typ(line, regex, float)

def position(line):
    ''' parse --lat/--lng/--alt arguments to a tuple '''
    # parse lat/lng/alt as floats, int
    latitude = _parse_arg_with_regex_to_typ(line, r'lat=(-?\d+.\d+)', float)
    longitude = _parse_arg_with_regex_to_typ(line, r'lng=(-?\d+.\d+)', float)
    altitude = _parse_arg_with_regex_to_typ(line, r'alt=(\d+)', int)
    # only return lat/lng as a pair, otherwise both as None
    if latitude == None or longitude == None:
        latitude, longitude = None, None
    return latitude, longitude, altitude

