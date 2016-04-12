available commands

help
  display this text

version
  print the argon version number

clear
  clear console history

exit
  exit the console app

status
  get status of the vehicle

telemetry
  print vehicle telemetry data to the console

monitor --delay=#
  print vehicle telemetry data to the console in a continous loop
  optionally provide a delay in seconds, minimum of 3, default of 10
  --delay is the ammount of time to wait between printing the telemetry data
  the loop will continue until broken with CTRL-C

config <KEY> <VALUE>
  update or print all configuration values from vehicle
  providing a KEY and VALUE will update that parameter
  all current vehicle parameters are outputted when not

mode <mode>
  update the vehicle's mode to 'loiter' or 'guided'
  loiter mode will allow the vehicle to be controlled via radio
  guided mode will allow the vehicle to recieve direction from argon

launch
  initiate the launch sequence for the vehicle
  requires that vehicle is not already active
  during the initial launch command, CTRL-D will cancel and land

position --lat=# --lng=# --alt=#
  signal vehicle to move to a lat/lng position
  optionally priovide an altitude to use at final position
  vehicle will maintain current altitude if none provided

move --heading=# --distance=# --altitude=#
  signal vehicle to move a distance, along a heading, to an altitude
  must provide head/dist together, optionally provide altitude
  providing only an altitude will change vehicle altitude
  final position is calculated and signalled to vehicle
  vehicle will maintain current altitude if none provided

yaw --heading=# |or| --unlock
  lock the orientation of the vehicle by rotating yaw to heading
  use --unlock flag in place of heading to 'unlock' the yaw
  this command will lock the vehicle's yaw until unlocked
  while yaw is locked, vehicle will move and maintain yaw orientation

camera
  send the vehicle signal to trigger camera to capture photo
  command enforces a wait time of 5 seconds

return
  signal vehicle to return to home and land
  requires that the vehicle is active
  this command will not block while the vehicle responds

land
  initiate the landing sequence for the vehicle
  requires that the vehicle is active
  this will land vehicle at current position, see return