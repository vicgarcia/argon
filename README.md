Argon is a console application to control a 3D Robotics IRIS+.

It allows the IRIS+ to be positioned by lat/lng for image capture.

Refer to help.txt for Argon's full command set.


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
