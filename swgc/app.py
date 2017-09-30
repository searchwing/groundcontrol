# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""Everything starts here.
Call run() to get swgc running.
"""
from time import sleep

from . ui import ui
from . gps import gps
from . uav import uav
from . switchboard import board


def run():
    """Call to get the app running.
    """
    gps.start()
    uav.start()
    board.start()
    ui.start()

    while 1:
        sleep(1000)
