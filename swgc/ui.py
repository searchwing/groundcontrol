# -*- coding: utf-8 -*-
"""SWGC UI.
On notify rewrite display.
"""
import threading, collections, traceback
import pygame

from . import sync
from . gps import gps
from . uav import uav
from . settings import *
from . switchboard import board
from . framebuffer import get as get_framebuffer



class UI(threading.Thread):
    """Thread controling the SWGC display.
    """

    def run(self):
        while 1:
            try:
                _run()
            except BaseException, e:
                print e
                traceback.print_exc()

ui = UI()




_screen, _font = None, None


def _run():
    """Internal.
    The UI thread.
    """
    global _screen, _font

    _screen = get_framebuffer()
    _font = pygame.font.SysFont(FONTNAME, FONTSIZE)
    _clear()

    while 1:
        texts = []
        texts.append('%s UTC\n' % gps.dt if gps.dt else '......\n')


        gpos = gps.get_position()
        upos = uav.get_position()
        bpos = board.get_position()


        if upos:
            text = \
 """UAV Current Position
 Latitude  %3.5f
 Longitude %3.5f
 Altitude  %7im
 """ % (upos.lat, upos.lon, upos.alt)

        else:
            text = \
"""UAV Current Position
 Latitude
 Longitude
 Altitude
"""
        texts.append(text)


        if bpos:
            if gpos:
                dist = gpos.distance(bpos)
                dist = '%7im' % dist
            else:
                dist = ''

            text = \
"""UAV Target Position
 Latitude  %3.5f
 Longitude %3.5f
 Distance  %s
 """ % (bpos.lat, bpos.lon, dist)

        else:
            text = \
"""UAV Target Position
 Latitude
 Longitude
 Distance
"""
        texts.append(text)


        text = board.get_message()
        if text:
            texts.append(text)


        _show(texts)

        # Wait max 1 second
        # For 25fps mandatory wait for 40ms
        pygame.time.delay(40)
        # Wait max 0.96 seconds for getting notified
        sync.wait(0.96)




def _clear():
    """Clear screen.
    """
    global _screen, _font

    _screen.fill(CLEAR_COLOR)
    pygame.display.update()




def _show(text):
    """Internal.
    Rewrite UI with passed text."""
    global _screen, _font

    if isinstance(text, collections.Iterable):
        text = '\n'.join([str(s) for s in text])

    text = text.split('\n')
    text = [_font.render(t, True, (0, 0, 0)) for t in text]
    _screen.fill((255, 255, 255))
    y = 20
    for t in text:
        _screen.blit(t, (20, y))
        y += 40
    pygame.display.flip()
