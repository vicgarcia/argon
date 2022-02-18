![pixhawk flight controller](https://github.com/vicgarcia/argon/raw/dev/.images/pixhawk.jpg)

Argon is a console application to control a [3D Robotics](https://3dr.com/) [IRIS+ quadrotor drone](https://3dr.com/support/articles/iris/).

I created this application to move the drone to specific latitude/longitude points in order to capture overhead imagery.  It uses [dronekit](https://github.com/dronekit/dronekit-python) and [cmd](https://docs.python.org/3/library/cmd.html).  The `--test` flag can be used when starting the app to also start and use the dronekit simulator.

```
# clone the repository
git clone git@github.com:vicgarcia/argon.git

# setup virtual environment + install dependencies w/ pipenv
cd argon
pyenv local 3.8.12
cp .env.example .env
mkdir .venv
pipenv install
```

Run Argon in test mode, which automatically starts/stop the SITL
```
pipenv run python run.py --test
```

Once started in test mode, the flight console can be used with a simulated drone.
```
# argon : flight control console

# status
system: STANDBY
mode: STABILIZE

# telemetry
position: 41.9751958, -87.663662
altitude: 0.01m
heading: 356
airspeed: 0.0
battery: 12.587 / 10.5

# launch
begin launch sequence
... preflight checks
... wait for vehicle to arm
... liftoff & approach target altitude
... launch successful, hovering at 7.91m

# move --dist=250 --head=90 --alt=50
update vehicle position
... calculate new position from parameters
... position update command issued

# status
system: ACTIVE
mode: GUIDED

# telemetry
position: 41.9751979, -87.6627578
altitude: 21.08m
heading: 90
airspeed: 3.819999933242798
battery: 12.193 / 10.5

# telemetry
position: 41.9751976, -87.6606524
altitude: 50.0m
heading: 90
airspeed: 0.029999999329447746
battery: 12.242 / 10.5

# return
signal vehicle for return and land
... RTL command issued

# status
system: ACTIVE
mode: RTL

# telemetry
position: 41.9751968, -87.6636602
altitude: 38.67m
heading: 90
airspeed: 0.019999999552965164
battery: 12.242 / 10.5

# telemetry
position: 41.9751968, -87.6636609
altitude: 26.44m
heading: 90
airspeed: 0.009999999776482582
battery: 12.242 / 10.5

# telemetry
position: 41.9751969, -87.6636608
altitude: 10.33m
heading: 89
airspeed: 0.0
battery: 12.242 / 10.5

# telemetry
position: 41.9751965, -87.6636608
altitude: 4.44m
heading: 89
airspeed: 0.009999999776482582
battery: 12.242 / 10.5

# telemetry
position: 41.9751985, -87.663657
altitude: 0.14m
heading: 90
airspeed: 0.0
battery: 12.587 / 10.5

# status
system: STANDBY
mode: RTL

# exit
exiting application
```

![drone photography](https://github.com/vicgarcia/argon/raw/dev/.images/kitescape.jpg)

This is an example of images captured using this application with my drone.  This image was created from three individual photos taken by moving the drone to each point and capturing the photo using this app.  The full size image is also included in this repository in the '.images' folder.
