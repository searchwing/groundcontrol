# -*- coding: utf-8 -*-
"""Everything starts here.
Call run() to get swgc running.
"""
import sys, time, traceback

from . ui import UI
from . uav import UAV
from . gps import GPS
from . switchboard import Board

from . settings import UAV_ADDRESS, GPS_PORT, GPS_BAUD, BOARD_PORT, BOARD_BAUD


def run():
    """Call to get the app running.
    """

    uav = UAV(address = UAV_ADDRESS)

    gps = GPS(
        name = 'GPS',
        port = GPS_PORT, baud = GPS_BAUD)

    board = Board(
        name = 'Board',
        port = BOARD_PORT, baud = BOARD_BAUD,
        gps = gps, uav = uav)

    ui = UI(gps = gps, uav = uav, board = board)


    uav.start()
    time.sleep(0.2)

    gps.start()
    time.sleep(0.2)

    board.start()
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
        uav.close() # Isn't blocking
    except Exception, e:
        print e
        traceback.print_exc()
    else:
        while not uav.is_closed():
            time.sleep(0.2)

    print 'Bye'
    # Really release all resources
    sys.exit()
