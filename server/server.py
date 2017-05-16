#!/usr/bin/env python
"""The http server
"""
import time, threading

from flask import render_template, jsonify
from dronekit import VehicleMode, LocationGlobal, Command
from pymavlink import mavutil

from . import app, gps, uav




def _run(run):
    """Run passed function in a thread.
    Internal only.
    """
    thread = threading.Thread(target = run)
    thread.daemon = True
    thread.start()




@app.route('/')
def index():
    """Index page.
    """
    ctx = {}
    return render_template('index.html', **ctx)


@app.route('/map')
def uav_map():
    """Map page.
    """
    ctx = {}
    return render_template('map.html', **ctx)


@app.route('/states')
def uav_states():
    ctx = {}
    return render_template('states.html', **ctx)








@app.route('/api/local/position')
def api_local_position():
    ctx = gps.get_position()
    return jsonify(ctx)




@app.route('/api/uav/states')
def api_uav_states():
    vehicle = uav.get()
    if not vehicle:
        return jsonify({})

    return jsonify({
        'version_major'             : vehicle.version.major,
        'version_patch'             : vehicle.version.patch,
        'version_release'           : vehicle.version.release,
        'system_status'             : vehicle.system_status.state,
        'is_armable'                : vehicle.is_armable,
        'armed'                     : vehicle.armed,
        'gps_fix'                   : vehicle.gps_0.fix_type,
        'gps_sats'                  : vehicle.gps_0.satellites_visible,
        'location_global_frame_alt' : vehicle.location.global_frame.alt,
        'location_global_frame_lon' : vehicle.location.global_frame.lon,
        'location_global_frame_lat' : vehicle.location.global_frame.lat,
        'rangefinder_distance'      : vehicle.rangefinder.distance,
        'rangefinder_voltage'       : vehicle.rangefinder.voltage,
        'attitude_pitch'            : vehicle.attitude.pitch,
        'attitude_roll'             : vehicle.attitude.roll,
        'attitude_yaw'              : vehicle.attitude.yaw,
        'velocity'                  : vehicle.velocity,
        'airspeed'                  : vehicle.airspeed,
        'groundspeed'               : vehicle.groundspeed,
        'heading'                   : vehicle.heading,
        'battery_voltage'           : vehicle.battery.voltage,
        'battery_current'           : vehicle.battery.current,
        'battery_level'             : vehicle.battery.current,
        'last_heartbeat'            : vehicle.last_heartbeat,
    })




@app.route('/api/uav/position')
def api_uav_position():
    vehicle = uav.get()
    if not vehicle:
        return jsonify({})

    return jsonify({
        'fix'  : vehicle.gps_0.fix_type,
        'sats' : vehicle.gps_0.satellites_visible,
        'alt'  : vehicle.location.global_frame.alt,
        'lon'  : vehicle.location.global_frame.lon,
        'lat'  : vehicle.location.global_frame.lat,
        'head' : vehicle.heading,
    })






@app.route('/api/uav/takeoff/<int:alt>')
def api_uav_takeoff(alt):
    """For copters only.
    Just climp to passed altitude.
    """
    def run():
        vehicle = uav.get()

        uav.prearm()
        vehicle.mode = VehicleMode('GUIDED')
        uav.arm()

        print 'UAV takeoff, to altitude', alt
        vehicle.simple_takeoff(alt)
    _run(run)

    return jsonify({'ok' : True,})




@app.route('/api/uav/goto/<float:lat>/<float:lon>/<int:alt>/<int:speed>')
def api_uav_goto(lat, lon, alt, speed):
    """Goto passed position and altitude.
    """
    def run():
        vehicle = uav.get()


        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        cmds.clear()

        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command(0, 0, 0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0,
                0, 0, 0, alt))
        cmds.add(Command(0,0,0,
                mavutil.mavlink.MAV_FRAME_GLOBAL,#_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, 0,
                lat, lon, alt))

        cmds.upload()


        uav.prearm()
        uav.arm()
        vehicle.mode = VehicleMode('AUTO')


#        print 'UAV takeoff', alt
#        vehicle.simple_takeoff(alt) # copters only
#        while True:
#            print 'UAV altitude:', vehicle.location.global_relative_frame.alt      
#            if vehicle.location.global_relative_frame.alt >= alt * 0.95:
#                print 'UAV reached target altitude'
#                break
#            time.sleep(1)


        vehicle.airspeed = speed
        vehicle.commands.next = 0


    _run(run)
    return jsonify({'ok' : True,})

