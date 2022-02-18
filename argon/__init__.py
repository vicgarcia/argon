import os
import logging
import sys
import dronekit
import dronekit_sitl
from .app import Argon
from . import console
from . import drones


def run():

    # get the connection string from .env file
    connection_string = os.environ.get('VEHICLE_CONNECTION_STRING')

    # when test mode enabled, start the simulator and get the connection string to it
    test = True if '--test' in sys.argv else False
    if test:
        sitl = dronekit_sitl.start_default(lat=41.9751961, lon=-87.6636616)
        connection_string = sitl.connection_string()
        # silence noisy output, we generally are unconcerned
        autopilot_logger = logging.getLogger('autopilot')
        autopilot_logger.disabled = True
        dronekit_logger = logging.getLogger('dronekit')
        dronekit_logger.disabled = True

    # exit if there is no connection string
    if not connection_string:
        console.white("No VEHICLE_CONNECTION_STRING found.")
        sys.exit(1)

    # connect to the vehicle, error handling the connection
    try:
        console.white("Connecting to vehicle.")
        vehicle = dronekit.connect(connection_string,
            vehicle_class=drones.IRIS,
            status_printer=lambda txt: None,        # defined above w/ custom vehicle
            wait_ready=True,
            heartbeat_timeout=30,                   # 30 second timeout
            baud=57600,                             # works w/ usb radio
        )
    # catch exception for CTRL-D cancel during connection
    except KeyboardInterrupt:
        console.white("Canceling connection attempt!")
        vehicle = None
    # catch other exceptions that occur during connection
    except Exception:
        console.white("Unable to connect!")
        vehicle = None

    # enter the console application, only when connected to a vehicle
    if vehicle:
        app = Argon(vehicle)
        app.cmdloop()

        # after execution of console app exits, close vehicle connection
        vehicle.close()

    # if running in test mode, end the simulator
    if test:
        sitl.stop()
