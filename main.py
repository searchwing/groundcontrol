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


def init():
    ui.init()
    gps.start()
    board.start()
    uav.start()


def show():
    last = None
    arm = False
    launch = False
    abort = False

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

        if bpos and \
                board.arm and not arm and uav.get_states():
            bpos.alt = 200
            uav.set_target(bpos)
            uav.arm()
        if bpos:
            arm = board.arm

        if board.launch and not launch:
            uav.launch()
        launch = board.launch

        if board.abort and not abort:
            uav.land()
            uav.disarm()
        abort = board.abort

        time.sleep(0.01)
        board.wait(1)


    

if __name__ == '__main__':
   init()
   show()
   #t = threading.Thread(target = show)
   #t.daemon = True
   #t.start()
   #t.join()

