#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Activate virtualenv
activate_this = '/home/pi/venv/bin/activate_this.py'
execfile(activate_this, dict(__file__=activate_this))

import time
from lib.gps import gps
from lib.switchboard import board
from lib import ui


def main():

    gps.start()
    board.start()
    ui.init()

    last = None
    while 1:
        bpos = board.get_position()
        if bpos:
            gpos = gps.get_position()
            dist = gpos.distance(bpos)
            text = 'lat %3.5f lon %3.5f %4im' % (
                    bpos.lat, bpos.lon, dist)
            if not text == last:
                last = text
                ui.text(text)
        else:
            ui.text('?')
        board.wait(1)
        time.sleep(0.01)
    

if __name__ == '__main__':
    main()
