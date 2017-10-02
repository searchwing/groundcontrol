# -*- coding: utf-8 -*-
"""Everything starts here.
Call run() to get swgc running.
"""
from time import sleep

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

    while 1:
        sleep(1000)
