#!/usr/bin/env python
from flask import render_template, jsonify

from . import get_uav, get_uav_states
from . import app




@app.route('/')
def index():
    """Index page.
    """
    ctx = {}
    return render_template('index.html', **ctx)


@app.route('/uav/map')
def map():
    """Map page.
    """
    ctx = {}
    return render_template('map.html', **ctx)


@app.route('/uav/states')
def uav_states():
    ctx = {}
    return render_template('states.html', **ctx)




@app.route('/api/uav/states')
def api_uav_states():
    uav = get_uav()
    if not uav:
        return jsonify({})

    return jsonify({
        'version_major'             : uav.version.major,
        'version_patch'             : uav.version.patch,
        'version_release'           : uav.version.release,
        'system_status'             : uav.system_status.state,
        'is_armable'                : uav.is_armable,
        'armed'                     : uav.armed,
        'gps_fix'                   : uav.gps_0.fix_type,
        'gps_sats'                  : uav.gps_0.satellites_visible,
        'location_global_frame_alt' : uav.location.global_frame.alt,
        'location_global_frame_lon' : uav.location.global_frame.lon,
        'location_global_frame_lat' : uav.location.global_frame.lat,
        'rangefinder_distance'      : uav.rangefinder.distance,
        'rangefinder_voltage'       : uav.rangefinder.voltage,
        'attitude_pitch'            : uav.attitude.pitch,
        'attitude_roll'             : uav.attitude.roll,
        'attitude_yaw'              : uav.attitude.yaw,
        'velocity'                  : uav.velocity,
        'airspeed'                  : uav.airspeed,
        'groundspeed'               : uav.groundspeed,
        'heading'                   : uav.heading,
        'battery_voltage'           : uav.battery.voltage,
        'battery_current'           : uav.battery.current,
        'battery_level'             : uav.battery.current,
        'last_heartbeat'            : uav.last_heartbeat,
    })




@app.route('/api/uav/position')
def api_uav_position():
    uav = get_uav()
    if not uav:
        return jsonify({})

    return jsonify({
        'fix'  : uav.gps_0.fix_type,
        'sats' : uav.gps_0.satellites_visible,
        'alt'  : uav.location.global_frame.alt,
        'lon'  : uav.location.global_frame.lon,
        'lat'  : uav.location.global_frame.lat,
    })




if __name__ == "__main__":
    app.run()
