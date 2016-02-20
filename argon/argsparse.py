import re


def count(line):
    ''' parse --count argument to an int or None '''
    count = None
    try:
        match = re.search(r'count=(\d+)', line)
        if match is not None:
            count = int(match.group(1))
    except:
        pass
    return count

def delay(line):
    ''' parse --delay argument to an int or None '''
    delay = None
    try:
        match = re.search(r'delay=(\d+)', line)
        if match is not None:
            delay = int(match.group(1))
    except:
        pass
    return delay

def altitude(line):
    ''' parse --altitude argument to an int or None '''
    altitude = None
    try:
        match = re.search(r'altitude=(\d+)', line)
        if match is not None:
            altitude = float(match.group(1))
    except:
        pass
    return altitude

def distance(line):
    ''' parse --distance argument to an int or None '''
    distance = None
    try:
        match = re.search(r'distance=(\d+)', line)
        if match is not None:
            distance = int(match.group(1))
    except:
        pass
    return distance

def heading(line):
    ''' parse --heading argument to an int or None '''
    heading = None
    try:
        match = re.search(r'heading=(\d+)', line)
        if match is not None:
            heading = int(match.group(1))
    except:
        pass
    return heading

def interval(line):
    ''' parse --interval argument to an int or None '''
    interval = None
    try:
        match = re.search(r'interval=(\d+)', line)
        if match is not None:
            interval = int(match.group(1))
    except:
        pass
    return interval

def maximum(line):
    ''' parse --maximum argument to an int or None '''
    maximum = None
    try:
        match = re.search(r'maximum=(\d+)', line)
        if match is not None:
            maximum = int(match.group(1))
    except:
        pass
    return maximum

def latitude(line):
    ''' parse --latitude argument to a float or None '''
    latitude = None
    try:
        match = re.search(r'latitude=(-?\d+.\d+)', line)
        if match is not None:
            latitude = float(match.group(1))
    except:
        pass
    return latitude

def longitude(line):
    ''' parse --longitude argument to a float or None '''
    longitude = None
    try:
        match = re.search(r'longitude=(-?\d+.\d+)', line)
        if match is not None:
            longitude = float(match.group(1))
    except:
        pass
    return longitude

def position(line):
    ''' parse --lat/--lng/--alt arguments to a tuple '''
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
    # parse altitude
    altitude = None
    try:
        match = re.search(r'alt=(\d+)', line)
        if match is not None:
            altitude = float(match.group(1))
    except:
        pass
    # only return lat/lng as a pair
    if latitude == None or longitude == None:
        latitude, longitude = None, None
    return latitude, longitude, altitude

