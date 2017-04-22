#!/usr/bin/env python
from flask import render_template, jsonify

from . import get_uav, get_uav_states
from . import get_gps_position
from . import app




@app.route('/')
def index():
    ctx = {}
    return render_template('index.html', **ctx)


@app.route('/map')
def map():
    ctx = {}
    return render_template('map.html', **ctx)


@app.route('/uavstates')
def uavstates():
    uav_states = get_uav_states()
    states = {key: uav_states.get(key) for key in [
        'groundspeed', 'airspeed', 'heading',
    ]}
    # jsonify sets json http content-type
    return jsonify(states)




if __name__ == "__main__":
    app.run()
