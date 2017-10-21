# -*- coding: utf-8 -*-
"""Everything starts here.
Call run() to get swgc running.
"""
import sys, time, traceback

from . import gps, uav, switchboard, ui


def run():
    """Call to get the app running.
    """

    uav = uav.UAV(UAV_ADDRESS)

    gps = gps.GPS(
            name = 'PGS',
            port = GPS_PORT, baud = GPS_BAUD)

    board = switchboard.Board(
            name = 'Board',
            port = BOARD_PORT, baud = BOARD_BAUD,
            gps = gps)

    ui = ui.UI(gps = gps, uav = uav)


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
