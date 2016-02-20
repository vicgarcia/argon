import sys
from argon.app import App
from argon.connect import sitl, drone


# default is to connect to drone
connect = drone

# if --test flag is provided, use sitl
if len(sys.argv) == 2:
    if sys.argv[1] == '--test':
        connect = sitl

# launch the console app
App(connect).cmdloop()
