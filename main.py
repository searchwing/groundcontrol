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
    

if __name__ == '__main__':
    ui.start()
    gps.start()
    uav.start()
    board.start()

    try:
        while 1:
            time.sleep(1000)
    except KeyboardInterrupt:
        pass

