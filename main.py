#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Activate virtualenv
activate_this = '/home/pi/venv/bin/activate_this.py'
execfile(activate_this, dict(__file__=activate_this))

import time

from lib import ui
from lib.gps import gps
from lib.uav import uav
from lib.switchboard import board


def main():

    ui.init()

    gps.start()
#    board.start()

    uav.start()

    last = None
    while 1:
        print uav.get_states()
        bpos = gps.get_position()#board.get_position()
        if bpos:
            gpos = gps.get_position()
            dist = gpos.distance(bpos)

            text = """lat %3.5f
lon %3.5f
m   %2i""" % (bpos.lat, bpos.lon, dist)

            if not text == last:
                last = text
                ui.text(text)
        else:
            ui.text('?')
        board.wait(1)
        time.sleep(0.01)
    

if __name__ == '__main__':
    main()
