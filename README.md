the console application can be run directly

```
  ./env/bin/python argon.py
```

to test with SITL, use two terminal windows

run the drone simulator in terminal one

```
  ./env/bin/dronekit-sitl copter --home=41.9751961,-87.6636616,0,0
```

run the console in test mode in terminal two

```
  ./env/bin/python argon.py --test
```
