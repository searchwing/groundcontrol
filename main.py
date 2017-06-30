#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Activate virtualenv
activate_this = '/home/pi/venv/bin/activate_this.py'
execfile(activate_this, dict(__file__=activate_this))

import time, threading

from lib import ui
from lib.gps import gps
from lib.uav import uav
from lib.switchboard import board


def show():

    gps.start()
    board.start()
    uav.start()
    ui.init()


    last = None
    while 1:
        text = '...'

        gpos = gps.get_position()
        upos = uav.get_position()
        bpos = board.get_position()

        dt = gps.dt or ''

        if upos:
            text = """
Current
Datetime  %s
Latitude  %3.5f
Longitude %3.5f
Altitude  %7im
""" % (dt, upos.lat, upos.lon, upos.alt)


        dist = ''
        if bpos:
            if gpos:
                dist = gpos.distance(bpos)

            text = """%s
Target
Latitude  %3.5f
Longitude %3.5f
Altitude  %7im
Distance  %7im
""" % (text, bpos.lat, bpos.lon, 200, dist)


        if not text == last:
            last = text
            ui.text(text)

        time.sleep(0.05)
        board.wait(1)


    

if __name__ == '__main__':
   show()

