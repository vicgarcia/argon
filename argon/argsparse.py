import re

def countdelay(line):
    ''' parse --count/delay arguments '''
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

def location(line):
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
    # return a two item tuple, (latitude, longitude)
    if latitude and longitude:
        return latitude, longitude
    # return None, None when both are not present
    else:
        return None, None

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

