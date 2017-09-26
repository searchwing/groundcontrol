# -*- coding: utf-8 -*-
"""App settings.
"""
import os


BASEDIR = os.path.abspath(os.path.dirname(__file__))


ALTITUDE = 30



CLEAR_COLOR = (255, 255, 255)
FONTNAME    = 'droidsansmono'
FONTSIZE    = 24




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




# In /etc/udev/rules.d/99-usbserial.rules
# add
# ACTION=="add",ENV{ID_BUS}=="usb",ENV{ID_SERIAL_SHORT}=="XXX",SYMLINK+="ttyBoard"
# ACTION=="add",ENV{ID_BUS}=="usb",ENV{ID_SERIAL_SHORT}=="YYY",SYMLINK+="ttyGPS"
# Replace XXX with result of
# sudo udevadm info --query=property --name=/dev/ttyUSB0 | grep ID_SERIAL_SHORT
# and YYY with result of
# sudo udevadm info --query=property --name=/dev/ttyUSB1 | grep ID_SERIAL_SHORT


""" Settings for local GPS
"""
GPS_PORT = '/dev/ttyGPS'
GPS_BAUD = 4800


"""Settings for switchboard
"""
BOARD_PORT    = '/dev/ttyBoard'
BOARD_BAUD    = 115200
BOARD_TIMEOUT = 2

