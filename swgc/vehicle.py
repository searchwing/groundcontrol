# -*- coding: utf-8 -*-
"""Yet another dronekit abstraction,
to work with Pixracer and PX4.
"""
import time, socket, exceptions

import dronekit
from dronekit import Command, VehicleMode
from pymavlink import mavutil

from . import geo, sync
from . settings import ALTITUDE as rtl_alt


BAUD              = 57600
TIMEOUT_HEARTBEAT = 30
REFRESH_RATE      = 4

# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
MAV_MODE_AUTO      =  4
MAV_MODE_STABILIZE = 16
MAV_MODE_MANUAL    = 64


# global singleton vehicle
vehicle = None

# and its home location
home = None




def log(*args):
    """Logging.
    """
    msg = ' '.join((str(arg) for arg in args)) if args else ''
    print 'UAV:', msg




def log_state():
    """Log selected vehicle states.
    """
    global vehicle, home

    if vehicle is None:
        log('Vehicle state: No vehicle, no state')

    try:
        log('Mode       :', vehicle.mode.name)
        log('Heartbeat  :', vehicle.last_heartbeat)
        log('Battery    :', vehicle.battery)
        log('GPS        :', vehicle.gps_0)
        log('Position   :', vehicle.location.global_relative_frame)
        log('Heading    :', vehicle.heading)
        log('Speed      :', vehicle.airspeed)
        log('Waypoint   :', vehicle.commands.next)
        log('Wpdistance :', get_distance_to_current_waypoint())

    except BaseException, e:
        log('Error logging state', e)




def get_distance_to_current_waypoint():
    """Get distance from home to current waypoint
    if there is a vehicle connected and has a current waypoint
    else None.
    """
    global vehicle, home

    if vehicle is None:
        return None

    nextwaypoint = vehicle.commands.next
    if not nextwaypoint:
        return None

    pos1 = vehicle.location.global_frame
    pos2 = vehicle.commands[nextwaypoint - 1]

    pos1 = geo.Position(pos1.lat, pos1.lon, pos1.alt)
    pos2 = geo.Position(pos2.x,   pos2.y,   pos2.z)

    return pos1.get_distance(pos2)




def px4_set_mode(mavMode):
    """Px4: set mode.
    """
    global vehicle, home

    log('Vehicle px4 set mode', mavMode)
    if mavMode is None:
        log('  Missing argument')
        return False
    if vehicle is None:
        log('No vehicle to set px4 mode')
        return False

    try:
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system, vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)

    except BaseException, e:
        log('Error px4 set mode', e)
        return False

    sync.notify()
    return True




# ------ Connection ----->

def connect(connection_string):
    """Connect vehicle.
    """
    global vehicle, home

    log('Vehicle connect')
    if connection_string is None:
        log('  Missing argument')
        return False
    if vehicle:
        log('Already connected a vehicle')
        return True

    try:
        # Connect
        log('Connect vehicle %s' % connection_string)
        vh = dronekit.connect(
            connection_string,
            baud              = BAUD,
            rate              = REFRESH_RATE,
            heartbeat_timeout = TIMEOUT_HEARTBEAT,
            wait_ready        = False)
        log('Connected vehicle')

        # Wait until its ready
        log('Initialize vehicle')
        vh.wait_ready(True)
        log('Initialized vehicle')

        vehicle = vh

    except socket.error, e:
        log('Error Connecting vehicle error: socket error', e)

    except exceptions.OSError, e:
        log('Error Connecting vehicle error: os error', e)

    except dronekit.APIException, e:
        log('Error Connecting vehicle error: timeout', e)

    except BaseException, e:
        log('Error Connecting vehicle error: unkown', e)

    if vehicle is None:
        log('Connecting vehicle failed')

    if vehicle is None:
        return False

    sync.notify()
    return True




def wait_for_connection():
    """Wait for connection to the vehicle.
    """
    global vehicle, home

    log('Wait for connection')

    while vehicle is None:
        sync.wait(1)




def is_connected():
    """Test if vehicle is connected.
    """
    global vehicle, home

    return not vehicle is None




def close():
    """Close vehicle.
    """
    global vehicle, home

    log('Vehicle close')
    if vehicle is None:
        log('No vehicle to close')
        return

    reset()

    home = None
    vh, vehicle = vehicle, None
    try:
        log('Close vehicle')
        vh.close()
        log('Closed vehicle')

    except BaseException, e:
        log('Vehicle Error on closing', e)

    sync.notify()

# <----- Connection ------




def wait_for_position():
    """Wait for home location.
    Return True on success.
    """
    global vehicle, home

    log('Vehicle waiting for position')
    if vehicle is None:
        log('No vehicle to wait for position')
        return False

    try:
        count = 0
        while not vehicle.home_location:#vehicle.gps_0.fix_type > 2:
            log('(%i) ' % count, 'sats:', vehicle.gps_0.satellites_visible,
                'fix:', vehicle.gps_0.fix_type,
                'position:',
                vehicle.location.global_frame.lat,
                vehicle.location.global_frame.lon,
                vehicle.location.global_frame.alt)
            count += 1
            time.sleep(1)

    except BaseException, e:
        log('Vehicle Error waiting for position', e)
        return False

    sync.notify()
    return True




def get_position():
    """Get current vehicle position if any.
    """
    global vehicle, home

    if vehicle is None:
        return None

    if vehicle.location is None:
        return None

    if vehicle.location.global_frame.lat      is None or \
            vehicle.location.global_frame.lon is None or \
            vehicle.location.global_frame.lat is None:
        return None

    return geo.Position.copy(
        vehicle.location.global_frame)



def get_heading():
    """Get vehicle heading if any.
    """
    global vehicle, home

    if vehicle is None:
        return None

    return vehicle.heading




def set_home_position():
    """Set current position as home position.
    """
    global vehicle, home

    position = get_position()
    if not position:
        log('No current position to set as home position')
        return False

    home = position
    return True




def get_home_position():
    """Get home position if set.
    """
    global vehicle, home

    return geo.Position.copy(home) if home else None




def reset():
    """Disarm,
    Go into STABILIZED mode,
    Clear any eventual mission.
    """
    global vehicle, home

    log('Vehicle reset')
    if vehicle is None:
        log('No vehicle to reset')
        return False

    try:
        # Disarm, Waiting here for disarmed never returns :(
        log('Disarm vehicle')
        vehicle.armed = False
        log('Disarmed vehicle')

        # Go into stabilized mode
        set_mode('STABILIZED')

        # Clear mission
        log('Clear vehicle mission')
        cmds = vehicle.commands
        cmds.clear()
        cmds.upload()
        log('Cleared vehicle mission')

    except BaseException, e:
        log('Vehicle Error on reset', e)
        return False

    sync.notify()
    return True




def arm():
    """Arm vehicle and wait for armed.
    """
    global vehicle, home

    log('Vehicle arm')
    if vehicle is None:
        log('No vehicle to arm')
        return False

    try:
        # Set home position to current position. Is this working?
        vehicle.home_location = vehicle.location.global_frame

        log('Arming vehicle')
        vehicle.armed = True
        while not vehicle.armed:
            time.sleep(1)
        log('Armed vehicle')

    except BaseException, e:
        log('Vehicle Error on arming', e)
        return False

    sync.notify()
    return True




def set_mode(mode):
    """Switch vehicle mode.
    """
    global vehicle, home

    log('Vehicle setting mode', mode)
    if mode is None:
        log('  Missing argument')
        return False
    if vehicle is None:
        log('No vehicle for setting mode')
        return False

    try:
        vehicle.mode = VehicleMode(mode)
        while not vehicle.mode.name == mode:
            time.sleep(1)
        log('Mode of vehicle now is', vehicle.mode.name)

    except BaseException, e:
        log('Vehicle Error on setting mode', e)
        return False

    sync.notify()
    return True




def get_mode():
    """Get vehicle mode.
    """
    global vehicle
    global vehicle, home
    if vehicle is None:
        return None

    return vehicle.mode.name




def is_flying():
    """Is the vehicle flying?
    """
    return get_mode() == 'MISSION'




def set_roi(pos):
    """Set 'region if interest'.
    Return True on success.
    """
    global vehicle, home

    log('Vehicle set ROI')
    if pos is None:
        log('  Missing argument')
        return False
    if not vehicle:
        log('No vehicle to setROI')
        return False

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,
        0, 0, 0, 0,
        pos.lat,
        pos.lon,
        pos.alt
    )
    vehicle.send_mavlink(msg)

    return True




def loiter(seconds, pos = None):
    """Loiter for passed seconds at passed position
    or if None at current position.
    Return True on success.
    """
    global vehicle, home

    log('Vehice loiter')
    if seconds is None:
        log('  Missing argument')
        return False
    if not vehicle:
        log('No vehicle to loiter')
        return False

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        0,
        seconds, 0, 0, 0,
        pos.lat,
        pos.lon,
        pos.alt
    )
    vehicle.send_mavlink(msg)

    return True




def set_mission(pos):
    """Build and upload mission.
    Return True on success.
    """
    global vehicle, home

    log('Vehicle set mission')
    if pos is None:
        log('  Missing argument')
        return False
    if vehicle is None:
        log('No vehicle to set mission')
        return False

    start = home
    if not start:
        log('No home position to start from')
        return False

    lat, lon, alt = pos.lat, pos.lon, pos.alt
    try:
        cmds = vehicle.commands
        cmds.clear()

        seq = 1

        # Takeoff to alt meters
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                      0, 1,
                      0, 0, 0, 0,
                      start.lat, start.lon, alt)
        cmds.add(cmd)
        seq += 1

        # Move to lat, lon, alt
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 1,
                      0, 0, 0, 0,
                      lat, lon, alt)
        cmds.add(cmd)
        seq += 1

        # Come back
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 1,
                      0, 0, 0, 0,
                      start.lat, start.lon, alt)
        cmds.add(cmd)
        seq += 1

        # Loiter
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                      0, 1,
                      20, 0, 0, 0,
                      start.lat, start.lon, alt)
        cmds.add(cmd)
        seq += 1

        # Land
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_LAND,
                      0, 1,
                      0, 0, 0, 0,
                      home.lat, home.lon, home.alt)
        cmds.add(cmd)
        seq += 1

        # Upload mission
        log('Uploading %i commands to vehicle' % len(cmds))
        cmds.upload()
        log('Uploaded %i commands to vehicle' % len(vehicle.commands))
        cmds.next = 1

    except BaseException, e:
        log('Vehicle Error on giving mission', e)
        return False

    return True




def return_to_land():
    """Command a flying vehicle to come home.
    Return True on success.
    """
    global vehicle, home

    log('Vehicle return to land')

    if vehicle is None:
        log('No vehicle to return to land')
        return False

    if home is None:
        log('No home to return to')
        return False

    try:
        cmds = vehicle.commands
        cmds.clear()

        seq = 1

        # Come home
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      0, 1,
                      0, 0, 0, 0,
                      home.lat, home.lon, rtl_alt)
        cmds.add(cmd)
        seq += 1

        # Land
        cmd = Command(0, 0, seq,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_LAND,
                      0, 1,
                      0, 0, 0, 0,
                      home.lat, home.lon, home.alt)
        cmds.add(cmd)
        seq += 1

        # Upload mission
        log('Uploading %i commands to vehicle' % len(cmds))
        cmds.upload()
        log('Uploaded %i commands to vehicle' % len(vehicle.commands))
        cmds.next = 1

    except BaseException, e:
        log('Vehicle Error on return to land', e)
        return False

    return True




def launch(monitor = False):
    """Fly mission on vehicle.
    """
    global vehicle, home

    log('Vehicle launch')
    if vehicle is None:
        log('No vehicle to launch')
        return False

    try:
        #log('Going into AUTO mode')
        #px4_set_mode(MAV_MODE_AUTO) # ?

        # Go into mission mode
        set_mode('MISSION')

        if monitor:
            # Monitor location
#            def location_callback(self, attr_name, msg):
#                log('Location Global', msg)
#            vehicle.add_attribute_listener('global_frame',
#                    location_callback)

            # Monitor mission execution
            nextwaypoint = vehicle.commands.next
            while nextwaypoint < len(vehicle.commands):
                if vehicle.commands.next > nextwaypoint:
                    display_seq = vehicle.commands.next + 1
                    log('Moving to waypoint %s' % display_seq)
                    nextwaypoint = vehicle.commands.next
                    log_state()
                sync.notify()
                time.sleep(1)

            # Wait for landed
            log('Waiting for land')
            while vehicle.commands.next > 0:
                sync.notify()
                log_state()
                time.sleep(1)
            log('Landed')

    except BaseException, e:
        log('Vehicle Error on launch (and monitor)', e)
        return False

    sync.notify()
    return True
