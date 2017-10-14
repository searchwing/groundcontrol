# -*- coding: utf-8 -*-
"""Everything starts here.
Call run() to get swgc running.
"""
import sys, time

from . import uav, gps, ui
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

    print 'Bye'
    # Really release all resources
    sys.exit()
