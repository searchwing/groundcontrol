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
    gps.start(GPS_PORT, GPS_BAUD)
    board.start(BOARD_PORT, BOARD_BAUD)
    ui.start()

    try:
        while 1:
            time.sleep(1000)
    except Exception, e:
        print e
        sys.exit() # really release all resources
