# -*- coding: utf-8 -*-
"""App settings.
"""
import os


BASEDIR = os.path.abspath(os.path.dirname(__file__))


""" Settings for UAV
  Address
    USB to pixhawk: /dev/tty.usbmodem1
    USB Radio Telemetry Ground Module: /dev/tty.SLAB_USBtoUART
    Simulator: udp:0.0.0.0:14550
    Start simulation
     dronekit-sitl plane-3.3.0 --home=<lat>,<lon>,<alt>,<yaw>
     mavproxy.py --master=tcp:0.0.0.0:5760 --out=udp:<IP of GC>:14550 --map
     Run GC on 'udp:0.0.0.0:14550'
  Baud
    Dronekit default is 115200
    USB: 115200 
    3DR Radio: 5700
  Timeouts
    in seconds
  Heartbeat timeout
    dronekit default is 30
  Refresh rate
    dronekit default is 4 for 4Hz
"""
UAV_ADDRESS                 = 'udp:0.0.0.0:14550'
UAV_BAUD                    = 57600
UAV_TIMEOUT_HEARTBEAT       = 30
UAV_TIMEOUT_CONNECTION      = 5
UAV_TIMEOUT_CONNECTION_TEST = 5
UAV_REFRESH_RATE            = 4

UAV_TIMEOUT_PREARM          = 10
UAV_TIMEOUT_ARM             = 10
UAV_TIMEOUT_GPS             = 10


""" Settings for local GPS
"""
GPS_PORT = '/dev/ttyUSB1'
GPS_BAUD = 4800


"""Settings for switchboard
"""
BOARD_PORT    = '/dev/ttyUSB0'
BOARD_BAUD    = 115200
BOARD_TIMEOUT = 2





# start SITL copter simulation
SITL = False

