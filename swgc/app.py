# -*- coding: utf-8 -*-
"""Everything starts here.
Call run() to get swgc running.
"""
import sys, time, traceback

from . uav import uav
from . import gps, ui
from . import switchboard as board
from . settings import UAV_ADDRESS, GPS_PORT, GPS_BAUD, BOARD_PORT, BOARD_BAUD


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
        # Intentional exit
        print
    except Exception, e:
        # Something else
        print e
        traceback.print_exc()

    try:
        uav.close()
    except Exception, e:
        print e
        traceback.print_exc()
    else:
        while not uav.is_closed():
            time.sleep(0.2)

    print 'Bye'
    # Really release all resources
    sys.exit()
