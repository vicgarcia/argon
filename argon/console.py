import os
from clint import textui


HELP_TEXT = '''
available commands

help
  display this text

version
  print the vehicle firmware version number

clear
  clear console history

exit
  exit the console app

status
  get status of the vehicle

telemetry
  print vehicle telemetry data to the console

mode <mode>
  update the vehicle's mode to 'loiter' or 'guided'
  loiter mode will allow the vehicle to be controlled via radio
  guided mode will allow the vehicle to recieve direction from Argon

launch
  initiate the launch sequence for the vehicle
  requires that vehicle is not already active
  during the initial launch command, CTRL-D will cancel and land

position --lat=# --lng=# --alt=#
  signal vehicle to move to a lat/lng position
  optionally priovide an altitude to use at final position
  vehicle will maintain current altitude if none provided

move --head=# --dist=# --alt=#
  signal vehicle to move a distance, along a heading, to an altitude
  must provide head/dist together, optionally provide altitude
  providing only an altitude will change vehicle altitude
  final position is calculated and signalled to vehicle
  vehicle will maintain current altitude if none provided

yaw --head=# |or| --unlock
  lock the orientation of the vehicle by rotating yaw to heading
  use --unlock flag in place of heading to 'unlock' the yaw
  this command will lock the vehicle's yaw until unlocked
  while yaw is locked, vehicle will move and maintain yaw orientation

camera
  send the vehicle signal to trigger camera to capture photo
  command enforces a wait time of 5 seconds

home
  signal vehicle to return to home at current altitude
  requires that the vehicle is active
  this is not RTL, vehicle will return to home and hover

return
  signal vehicle to return to home and land (RTL mode)
  requires that the vehicle is active
  this command will not block while the vehicle responds

land
  initiate the landing sequence for the vehicle
  requires that the vehicle is active
  this will land vehicle at current position, see return
'''


def clear():
    ''' clear the console '''
    os.system('clear')


def blank():
    ''' output a blank line to the console '''
    textui.puts(newline=True)


def white(text):
    ''' output white text to the console '''
    textui.puts(text)


def red(text):
    ''' output red text to the console '''
    textui.puts(textui.colored.red(text))
