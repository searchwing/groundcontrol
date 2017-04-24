# -*- coding: utf-8 -*-
"""App settings.
"""

""" Settings for pixhawk
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
UAV_ADDRESS                 = '/dev/tty.usbmodem1'
UAV_BAUD                    = 57600
UAV_HEARTBEAT_TIMEOUT       = 30
UAV_CONNECTION_TIMEOUT      = 0
UAV_CONNECTION_TEST_TIMEOUT = 2
UAV_REFRESH_RATE            = 4



""" Settings for local GPS
"""
GPS_PORT = '/dev/tty.SLAB_USBtoUART'
GPS_BAUD = 4800
