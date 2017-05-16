# -*- coding: utf-8 -*-
"""App settings.
"""


""" Settings for UAV
  Address
    USB to pixhawk: /dev/tty.usbmodem1
    USB Radio Telemetry Ground Module: /dev/tty.SLAB_USBtoUART
    Simulator: tcp:127.0.0.1:5760
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




"""Settings for DB
"""
DATABASE = 'db.sql'




""" Settings for local GPS
"""
GPS_PORT = '/dev/tty.SLAB_USBtoUART'
GPS_BAUD = 4800




# start SITL copter simulation
SITL = False

