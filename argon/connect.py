import dronekit


def sitl():
    ''' connect to the sitl vehicle via tcp test interface '''
    return dronekit.connect('tcp:127.0.0.1:5760',
            status_printer=None, wait_ready=True, heartbeat_timeout=20)

def drone():
    ''' connect to the 3DR IRIS+ via usb radio dongle '''
    return dronekit.connect('/dev/cu.usbserial-DJ00DSDS', baud=57600,
            status_printer=None, wait_ready=True, heartbeat_timeout=20)

