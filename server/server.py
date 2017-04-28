#!/usr/bin/env python
"""The http server component.
"""
import time

from dronekit import VehicleMode
from flask import render_template, jsonify

from . import app, gps, uav
from . import app




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






"""READY, BUSY, FAIL, OK"""
_api_uav_apiState = 'READY'

@app.route('/api/uav/apistate')
def api_uav_state():
    return _api_uav_apiState




@app.route('/api/local/gps')
def api_local_gps():
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
    })




@app.route('/api/uav/loiter/<int:altitude>')
def api_uav_loiter(altitude):
    if not uav.is_ready():
        return jsonify({'ok' : False,})

    uav.prearm()
    vehicle = uav.get()
    vehicle.mode = VehicleMode('GUIDED')
    uav.arm()
    vehicle.simple_takeoff(altitude)

    return jsonify({'ok' : True,})




if __name__ == "__main__":
    app.run()

