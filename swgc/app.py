# -*- coding: utf-8 -*-
"""Everything starts here.
Call run() to get swgc running.
"""
import sys, time

from . import uav, gps, ui
from . import switchboard as board
from . settings import *


def run():
    """Call to get the app running.
    """
    uav.start(UAV_ADDRESS)
    time.sleep(0.2)
    gps.start(GPS_PORT, GPS_BAUD)
    time.sleep(0.2)
    board.start(BOARD_PORT, BOARD_BAUD)
    time.sleep(0.2)
    ui.start()

    try:
        while 1:
            time.sleep(1000)
    except KeyboardInterrupt:
        print # Intentional exit
    except Exception, e:
        print e
    print 'Bye'
    sys.exit() # Really release all resources
