#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""Test python dronekit: take off, fly a rectangle, land.

With pixracer, mavesp8266 on an esp01 and dronekit installed from its GIT repo
    pip install git+https://github.com/dronekit/dronekit-python
Might work with other mavlink stacks as well.

Get a pre build mavesp8266 binary
    http://www.grubba.com/mavesp8266/firmware-1.1.1.bin
Or build it yourself
    with platformIO - no external OS dependencies
    https://github.com/dogmaphobic/mavesp8266

To flash an esp8266 (with the mavesp8266 binary)
    esptool.py --baud 921600 --port <your_serial_port> write_flash 0x00000 <the_binary>

To bring the mavesp8266 wifi into client mode
    http://192.168.4.1/setparameters?mode=1&ssidsta=<ssid>&pwdsta=<password>

To bring the mavesp8266 wifi back into AP mode
    call http://192.168.4.1/setparameters?mode=0&ssid=<ssid>&pwd=<password>

If an esp8266 in client mode fails to connect an AP it falls back into the default
AP mode with ssid 'PixRacer' and password 'pixracer'.
"""
import os

# Activate python virtualenv in current user homedir.
#  (If you work in virtualenv, in ~/venv)
if __name__ == '__main__':
    activate_this = '%s/venv/bin/activate_this.py' % os.path.expanduser('~')
    execfile(activate_this, dict(__file__=activate_this))

import time, sys, argparse, math, socket, exceptions

import dronekit
from dronekit import Command, LocationGlobal, VehicleMode
from pymavlink import mavutil


BAUD              = 57600
TIMEOUT_HEARTBEAT = 30
REFRESH_RATE      = 4

# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
MAV_MODE_AUTO      =  4
MAV_MODE_STABILIZE = 16
MAV_MODE_MANUAL    = 64


def log(*args):
    """Logging for the poor.
    """
    print ':::',
    if args:
        for arg in args:
            print str(arg),
    print




def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def distance_to_current_waypoint():
    """Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    global vehicle

    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1]
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


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




def _px4_setMode(mavMode):
    global vehicle

    #log(vehicle._master.mode_mapping())

    vehicle._master.mav.command_long_send(
        vehicle._master.target_system, vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)




def vehicle_connect(connection_string):
    """Connect vehicle.
    """
    global vehicle

    try:
        # Connect, don't wait until its ready
        log('Connect %s' % connection_string)
        vehicle = dronekit.connect(
                connection_string,
                baud              = BAUD,
                rate              = REFRESH_RATE,
                heartbeat_timeout = TIMEOUT_HEARTBEAT,
                wait_ready        = False)
        log('Connected')

        # Reset as early as possible before it messes with
        # any residues
        _vehicle_reset()

        # Now we can wait until its ready
        log('Initialize')
        vehicle.wait_ready(True)
        log('Initialized')

    except socket.error:
        log('Connecting UAV error: no server')

    except exceptions.OSError:
        log('Connecting UAV error: no serial')

    except dronekit.APIException:
        log('Connecting UAV error: timeout')

    except Exception, e:
        log('Connecting UAV error: unkown', e)

    if not vehicle:
        log('Connecting failed')
        return False
    return True


def vehicle_close():
    """Close if there is a vehicle
    """
    global vehicle

    try:
        _vehicle_reset()
    except Exception, e:
        log('Error on reset', e)

    try:
        log('Closing')
        vehicle.close()
        log('Closed')
    except Exception, e:
        log('Error on closing', e)
    finally:
        vehicle = None


def _vehicle_reset():
    """Disarm,
    Go into STABILIZED mode,
    Clear any eventual mission.
    """
    global vehicle

    # Disarm, Waiting here for disarmed never returns
    log('Disarm')
    vehicle.armed = False
    log('Disarmed')

    # Go into stabilized mode
    _vehicle_mode('STABILIZED')

    # Clear mission
    log('Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()
    log('Mission cleared')


def _vehicle_mode(mode):
    """Switch vehicle mode.
    """
    log('Going into mode', mode)
    vehicle.mode = VehicleMode(mode)
    while not vehicle.mode.name == mode:
        log('Mode is', vehicle.mode.name)
        time.sleep(1)
    log('Mode is', vehicle.mode.name)


def vehicle_waitfor_position():
    """Wait for home position.
    """
    log('Wait for home position')
    count = 0
    while not vehicle.home_location:
        log('(%i)' % count, '    sats:', vehicle.gps_0.satellites_visible,
            'fix:', vehicle.gps_0.fix_type,
            'position:', vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
        count += 1
        time.sleep(1)
    home = vehicle.home_location
    log('Got a home position', vehicle.home_location)


def vehicle_mission():
    """Build and upload mission.
    """
    global vehicle

    cmds = vehicle.commands
    cmds.clear()

    home = vehicle.location.global_relative_frame
    alt, step = 25, 25 
    seq = 1

    # Takeoff to alt meters
    wp = get_location_offset_meters(home, 0, 0, alt)
    cmd = Command(0, 0, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1,
            0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    seq += 1

    # Move step meters north
    wp = get_location_offset_meters(wp, step, 0, 0)
    cmd = Command(0, 0, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    seq += 1

    # Move step meters east
    wp = get_location_offset_meters(wp, 0, step, 0)
    cmd = Command(0, 0, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    seq += 1

    # Move step meters south
    wp = get_location_offset_meters(wp, -step, 0, 0)
    cmd = Command(0, 0, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    seq += 1

    # Move step meters west
    wp = get_location_offset_meters(wp, 0, -step, 0)
    cmd = Command(0, 0, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            0, 0, 0, 0,
            wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    seq += 1

    # Land
    cmd = Command(0, 0, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 1,
            0, 0, 0, 0,
            home.lat, home.lon, home.alt+1)
    cmds.add(cmd)
    seq += 1

    # Upload mission
    log('Uploading %i commands' % len(cmds))
    cmds.upload()
    log('Uploaded %i commands' % len(vehicle.commands))
    cmds.next = 1


def vehicle_arm():
    """Arm vehicle and wait for armed
    """
    global vehicle

    # Set home position to current position. Is this working?
    vehicle.home_location = vehicle.location.global_frame

    log('Arming')
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    log('Armed')


def vehicle_fly(monitor):
    """Fly mission on vehicle.
    """
    global vehicle

    #log('Going into AUTO mode')
    #_px4_setMode(MAV_MODE_AUTO) # ?

    # Go into mission mode
    _vehicle_mode('MISSION')

    if monitor:
        # Monitor location
        def location_callback(self, attr_name, msg):
            log('Location Global', msg)
        vehicle.add_attribute_listener('global_frame',
                location_callback)

        # Monitor mission execution
        nextwaypoint = vehicle.commands.next
        while nextwaypoint < len(vehicle.commands):
            if vehicle.commands.next > nextwaypoint:
                display_seq = vehicle.commands.next + 1
                log('Moving to waypoint %s' % display_seq)
                nextwaypoint = vehicle.commands.next
            time.sleep(1)

        # Wait for landed
        log('Waiting for land')
        while vehicle.commands.next > 0:
            time.sleep(1)
        log('Landed')




vehicle = None

def main():
    global vehicle

    try:
        # look for connection arguments
        connection_string = 'udp:0.0.0.0:14550'
        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--connect', help='connection string')
        args = parser.parse_args()
        if args.connect:
            connection_string = args.connect

        # Connect, wait for ready
        if not vehicle_connect(connection_string):
            return

        #def wildcard_callback(self, attr_name, value):
        #    log(' CALLBACK(%s): %s' % (attr_name,value))
        #vehicle.add_attribute_listener('*', wildcard_callback)
        #time.sleep(1)
        #vehicle.remove_attribute_listener('*', wildcard_callback)

        # We need an home position
        vehicle_waitfor_position()

        log('Status:              %s' % vehicle.system_status.state)
        log('Mode:                %s' % vehicle.mode.name)
        log('Firmware:            %s' % vehicle.version)
        log('Battery:             %s' % vehicle.battery)
        log('GPS Info:            %s' % vehicle.gps_0)
        log('Global Location:     %s' % vehicle.location.global_frame)
        log('Global Location rel: %s' % vehicle.location.global_relative_frame)
        log('EKF ok:              %s' % vehicle.ekf_ok)

        # Lets see any message
    #    @vehicle.on_message('*')
    #    def listener(self, name, message):
    #        log('MESSAGE:', name

        # Build and upload a mission
        vehicle_mission()

        # Arm
        vehicle_arm()

        # For demo purposes
        time.sleep(2)

        # Actually fly the mission
        vehicle_fly(monitor = True)

    except KeyboardInterrupt:
        print
    finally:
        # Kill the vehicle
        if vehicle:
            vehicle_close()


if __name__ == '__main__':
    main()
    log('Bye')

