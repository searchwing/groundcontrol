#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""Test dronekit with pixracer.
Inspired by https://dev.px4.io/en/dronekit/example.html

Works with pixracer, mavesp8266 and dronekit installed from its GIT repo:
    git clone https://github.com/dronekit/dronekit-python.git
    cd dronekit-python
    pip install -e .

To bring the mavesp8266 wifi into client mode:
call http://192.168.4.1/setparameters?mode=1&ssidsta=<ssid>&pwdsta=<password>
To bring the mavesp8266 wifi back into AP mode:
call http://192.168.4.1/setparameters?mode=1&ssid=<ssid>&pwd=<password>
"""
import os

# Activate virtualenv in user homedir.
# You might not need this.
activate_this = '%s/venv/bin/activate_this.py' % os.path.expanduser('~')
execfile(activate_this, dict(__file__=activate_this))

import time, sys, argparse, math

from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil


BAUD              = 57600
TIMEOUT_HEARTBEAT = 30
REFRESH_RATE      = 4

MAV_MODE_AUTO = 4 # https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


connection_string = 'udp:0.0.0.0:14550'
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--connect', help='connection string')
args = parser.parse_args()
if args.connect:
    connection_string = args.connect




def px4_setMode(mavMode):
    global vehicle
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system, vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)


def clear_mission():
    global vehicle
    cmds = vehicle.commands
    print 'Clearing %s commands' % cmds.count
    if cmds.count:
        cmds.clear()
        cmds.upload()
    return cmds


def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of 'spherical' earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)




def fly():
    global vehicle

    cmds = vehicle.commands
    cmds.clear()

    home = vehicle.location.global_relative_frame
    alt, step = 5, 5 

    # Takeoff to alt meters
    wp = get_location_offset_meters(home, 0, 0, alt)
    cmd = Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1, 0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Move step meters north
    wp = get_location_offset_meters(wp, step, 0, 0)
    cmd = Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Move step meters east
    wp = get_location_offset_meters(wp, 0, step, 0)
    cmd = Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Move step meters south
    wp = get_location_offset_meters(wp, -step, 0, 0)
    cmd = Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Move step meters west
    wp = get_location_offset_meters(wp, 0, -step, 0)
    cmd = Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Land
    wp = get_location_offset_meters(home, 0, 0, 0)
    cmd = Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 1, 0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

    # Upload mission
    print 'Uploading commands'
    cmds.upload()
    print 'Uploaded %i commands' % len(vehicle.commands)

#    print 'Wait for armable'
#    while not vehicle.is_armable:
#        time.sleep(1)

    # Arm vehicle
    print 'Arming'
    vehicle.armed = True
#    while not vehicle.armed:
#        time.sleep(1)

    # Monitor mission execution
    print 'Starting mission'
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands):
        if vehicle.commands.next > nextwaypoint:
            display_seq = vehicle.commands.next + 1
            print 'Moving to waypoint %s' % display_seq
            nextwaypoint = vehicle.commands.next
        time.sleep(1)

    # Wait for landed
    print 'Waiting for land'
    while vehicle.commands.next > 0:
        time.sleep(1)
    print 'Landed'

    # Disarm vehicle
    print 'Disarming'
    vehicle.armed = False



vehicle = None
try:
    print 'Connecting %s' % connection_string
    vehicle = connect(
            connection_string,
            baud              = BAUD,
            rate              = REFRESH_RATE,
            heartbeat_timeout = TIMEOUT_HEARTBEAT,
            wait_ready         = ['system_status',])
    vehicle.armed = False

    # Go into STABILIZED mode. Do we need this?
    print 'Going into vehicle mode STABILIZED'
    vehicle.mode = VehicleMode('STABILIZED')
    while not vehicle.mode.name == 'STABILIZED':
        time.sleep(1)

    # Clear any eventual previous mission
    clear_mission()

    # We need an home position
    print 'Waiting for home position'
    home_position_set = False
    @vehicle.on_message('HOME_POSITION')
    def listener(self, name, home_position):
        global home_position_set
        home_position_set = True
    while not home_position_set:
        time.sleep(1)


    print
    print 'Status:              %s' % vehicle.system_status.state
    print 'Mode:                %s' % vehicle.mode.name
    print 'Firmware:            %s' % vehicle.version
    print 'Battery:             %s' % vehicle.battery
    print 'GPS Info:            %s' % vehicle.gps_0
    print 'Altitude:            %s' % vehicle.location.global_relative_frame.alt
    #print 'Local Location:      %s' % vehicle.location.local_frame
    print 'Global Location:     %s' % vehicle.location.global_frame
    print 'Global Location rel: %s' % vehicle.location.global_relative_frame
    #print 'EKF ok:              %s' % vehicle.ekf_ok
    print


    px4_setMode(MAV_MODE_AUTO)
    fly()

except KeyboardInterrupt:
    pass
#except Exception, e:
#    print e
finally:
    print 'Finishing'
    if vehicle:
        print 'Disarming vehicle'
        vehicle.armed = False
        clear_mission()
        print 'Closing vehicle'
        vehicle.close()
print 'Bye'

