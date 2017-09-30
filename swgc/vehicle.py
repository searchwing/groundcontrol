# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""pixracer/px4/dronekit abstraction.
"""
import time, socket, exceptions, traceback

import dronekit
from dronekit import Command, VehicleMode
from pymavlink import mavutil

from . import geo, sync


BAUD              = 57600
TIMEOUT_HEARTBEAT = 30
REFRESH_RATE      = 4

# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
MAV_MODE_AUTO      =  4
MAV_MODE_STABILIZE = 16
MAV_MODE_MANUAL    = 64


# global singleton vehicle
vehicle = None




def log(*args):
    """Logging for the poor.
    """
    msg = ' '.join((str(arg) for arg in args)) if args else ''
    print 'UAV:', msg




def get_distance_to_current_waypoint():
    """Get distance from home to current waypoint
    if there is a vehicle connected and has a current waypoint
    else None.
    """
    global vehicle
    if not vehicle:
        return None

    nextwaypoint = vehicle.commands.next
    if not nextwaypoint:
        return None

    pos1 = vehicle.location.global_frame
    pos2 = vehicle.commands[nextwaypoint - 1]

    pos1 = geo.Position.copy(pos1)
    pos2 = geo.Position.copy(pos2)
    return pos1.distance(pos2)




def px4_set_mode(mavMode):
    """Px4: set mode.
    """
    global vehicle
    log('Vehicle px4 set mode', mavMode)
    if not vehicle:
        log('No vehicle to set px4 mode')

    else:
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system, vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)




def log_state():
    """Log selected vehicle states.
    """
    global vehicle
    if not vehicle:
        log('Vehicle state: No vehicle, no state')

    else:
        log('Mode       :', vehicle.mode.name)
        log('Heartbeat  :', vehicle.last_heartbeat)
        log('Battery    :', vehicle.battery)
        log('GPS        :', vehicle.gps_0)
        log('Position   :', vehicle.location.global_relative_frame)
        log('Heading    :', vehicle.heading)
        log('Speed      :', vehicle.airspeed)
        log('Waypoint   :', vehicle.commands.next)
        log('Wpdistance :', get_distance_to_current_waypoint())




def connect(connection_string):
    """Connect vehicle.
    """
    global vehicle
    log('Vehicle connect')
    if vehicle:
        log('Already connected a vehicle')

    else:
        try:
            # Connect, don't wait until its ready
            log('Connect vehicle %s' % connection_string)
            vh = dronekit.connect(
                connection_string,
                baud              = BAUD,
                rate              = REFRESH_RATE,
                heartbeat_timeout = TIMEOUT_HEARTBEAT,
                wait_ready        = False)
            log('Connected vehicle')

            # Reset as early as possible before it messes with
            # any residues
            reset()

            # Now we can wait until its ready
            log('Initialize vehicle')
            vh.wait_ready(True)
            log('Initialized vehicle')
            vehicle = vh

        except socket.error, e:
            log('Error Connecting vehicle error: no server', e)

        except exceptions.OSError, e:
            log('Error Connecting vehicle error: no serial', e)

        except dronekit.APIException, e:
            log('Error Connecting vehicle error: timeout', e)

        except BaseException, e:
            # We can't do anything about it
            log('Error Connecting vehicle error: unkown', e)

        if not vehicle:
            log('Connecting vehicle failed')

        sync.notify()




def is_connected():
    """Test if vehicle is connected.
    """
    global vehicle
    return not vehicle is None




def close():
    """Close vehicle.
    """
    global vehicle
    log('Vehicle close')
    if not vehicle:
        log('No vehicle to close')

    else:
        reset()

        try:
            log('Close vehicle')
            vehicle.close()
            log('Closed vehicle')

        except BaseException, e:
            # We can't do anything here
            log('Vehicle Error on closing', e)

        vehicle = None

    sync.notify()




def reset():
    """Disarm,
    Go into STABILIZED mode,
    Clear any eventual mission.
    """
    global vehicle
    log('Vehicle reset')
    if not vehicle:
        log('No vehicle to reset')

    else:
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

    sync.notify()




def arm():
    """Arm vehicle and wait for armed.
    """
    global vehicle
    log('Vehicle arm')
    if not vehicle:
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
        return True

    except BaseException, e:
        log('Vehicle Error on arming', e)

    finally:
        sync.notify()

    return False




def set_mode(mode):
    """Switch vehicle mode.
    """
    global vehicle
    log('Vehicle set mode', mode)
    if not vehicle:
        log('No vehicle for setting mode')

    else:
        try:
            vehicle.mode = VehicleMode(mode)
            while not vehicle.mode.name == mode:
                log('Mode of vehicle is', vehicle.mode.name)
                time.sleep(1)
            log('Mode of vehicle is', vehicle.mode.name)

        except BaseException, e:
            log('Vehicle Error on setting mode', e)

    sync.notify()




def get_mode():
    """Get vehicle mode.
    """
    global vehicle
    if not vehicle:
        log('No vehicle for getting mode')
        return None

    return vehicle.mode.name




def is_flying():
    """Is the vehicle flying?
    """
    return get_mode() == 'MISSION'




def wait_for_position():
    """Wait for home position.
    """
    global vehicle
    log('Vehicle waiting for home position')
    if not vehicle:
        log('No vehicle to wait for home position')

    else:
        try:
            count = 0
            while 1:
                log('(%i) ' % count, 'sats:', vehicle.gps_0.satellites_visible,
                    'fix:', vehicle.gps_0.fix_type,
                    'position:',
                    vehicle.location.global_frame.lat,
                    vehicle.location.global_frame.lon,
                    vehicle.location.global_frame.alt)
                if vehicle.home_location:
                    break
                count += 1
                time.sleep(1)
            log('Got a vehicle home position', vehicle.home_location)

        except BaseException, e:
            log('Vehicle Error waiting for home position', e)

    sync.notify()




def get_position():
    """Get vehicle position if any.
    """
    global vehicle
    if not vehicle:
        return None

    return vehicle.location.global_frame




def set_target(pos):
    """Build and upload mission.
    """
    global vehicle
    log('Vehicle give mission')
    if not vehicle:
        log('No vehicle to give a mission')

    else:
        lat, lon, alt = pos.lat, pos.lon, pos.alt

        try:
            cmds = vehicle.commands
            cmds.clear()

            home = vehicle.location.global_relative_frame
            alt = 25
            seq = 1

            # Takeoff to alt meters
            cmd = Command(0, 0, seq,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                          0, 1,
                          0, 0, 0, 0,
                          home.lat, home.lon, alt)
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

            # Move home
            cmd = Command(0, 0, seq,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          0, 1,
                          0, 0, 0, 0,
                          home.lat, home.lon, alt)
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
            traceback.print_exc()
            return False
        finally:
            sync.notify()
        return True





def launch(monitor = False):
    """Fly mission on vehicle.
    """
    global vehicle
    log('Vehicle launch')
    if not vehicle:
        log('No vehicle to launch')

    else:
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

        sync.notify()
