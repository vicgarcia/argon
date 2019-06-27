![pixhawk flight controller](https://github.com/vicgarcia/argon/raw/dev/images/pixhawk.jpg)

Argon is a console application to control a 3D Robotics IRIS+.

![drone photography](https://github.com/vicgarcia/argon/raw/dev/images/kitescape.jpg)
https://github.com/vicgarcia/argon/raw/dev/images/fullsize.png

I created this application to move the drone to set points and with
precision movement to other points in order to capture imagery.



```

# clone the repository
git clone git@github.com:vicgarcia/argon.git

# setup virtual environment
cd argon
virtualenv --no-site-packages env

# install dependencies
./env/bin/pip install -r requirements.txt

# run argon in test mode, which uses the dronekit simulator
./env/bin/python argon.py --test

```
