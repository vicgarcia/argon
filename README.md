Argon is a console application to control a 3D Robotics IRIS+.

The goal of Argon is to get acquainted with dronekit as well as
navigate the IRIS+ to specified coordinates and capture images.

Argon uses dronekit, a python library provided by 3D Robotics.

Use the 'help' command in the application or refer to the help.txt
file in this repository for Argon's full command set.

The console application can be run directly ...

```
  ./env/bin/python argon.py
```

To test with SITL, use two terminal windows ...

Run the drone simulator in terminal one.

```
  ./env/bin/dronekit-sitl copter --home=41.9751961,-87.6636616,0,0
```

Run the console in test mode in terminal two.

```
  ./env/bin/python argon.py --test
```
