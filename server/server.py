#!/usr/bin/env python
from flask import render_template

from . import get_uav, get_uav_states
from . import get_gps_position
from . import app




@app.route('/')
def index():
    uav = get_uav()
    ctx = {
    }
    return render_template('index.html', **ctx)


@app.route('/map')
def map():
    uav = get_uav()
    ctx = {
    }
    return render_template('map.html', **ctx)




if __name__ == "__main__":
    app.run()
