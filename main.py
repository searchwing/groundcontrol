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
    armed = False
    while 1:
        text = '...'

        gpos = gps.get_position()
        upos = uav.get_position()
        bpos = board.get_position()

        if upos:
            text = """Current
Latitude  %3.5f
Longitude %3.5f
Altitude  %7im
""" % (upos.lat, upos.lon, upos.alt)


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


        if board.arm and not armed and uav.get_states():
            print 'arm'
            uav.arm()

        time.sleep(0.01)
        board.wait(1)


    

if __name__ == '__main__':
   init()
   show()
   #t = threading.Thread(target = show)
   #t.daemon = True
   #t.start()
   #t.join()

