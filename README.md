Argon is a console application to control a 3D Robotics IRIS+.

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
