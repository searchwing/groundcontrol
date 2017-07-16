# -*- coding: utf-8 -*-
"""SWGC UI.
On notify rewrite display.
"""
import threading, collections
import pygame

from . gps import gps
from . uav import uav
from . switchboard import board


FONTNAME = 'droidsansmono'
FONTSIZE = 24
SCREENSIZE = (480, 800)




_condition = threading.Condition()
    

def wait(timeout = None):
    """Wait until UI gets notified.
    Called by UI thread.
    """
    global _condition

    _condition.acquire()
    _condition.wait(timeout = timeout)
    _condition.release()


def notify():
    """Notify UI to rewrite display.
    """
    global _condition

    _condition.acquire()
    _condition.notifyAll()
    _condition.release()




def _run():
    """Internal.
    The UI thread.
    """
    while 1:
        texts = []

        texts.append('%s UTC\n' % gps.dt if gps.dt else '......\n')


        gpos = gps.get_position()
        upos = uav.get_position()
        bpos = board.get_position()


        if upos:
            text = \
 """Current Position
 Latitude  %3.5f
 Longitude %3.5f
 Altitude  %7im
 """ % (upos.lat, upos.lon, upos.alt)
    
        else:
            text = \
"""Current Position
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
"""Target Position
 Latitude  %3.5f
 Longitude %3.5f
 Distance  %s
 """ % (bpos.lat, bpos.lon, dist)

        else:
            text = \
"""Target Position
 Latitude
 Longitude
 Distance
"""

        texts.append(text)


        text = board.get_message()
        if text:
            texts.append(text)


        _show(texts)
        wait(1) # Wait a second or get notified




_screen, _font = None, None


def start():
    """Start UI threat.
    """
    global _screen, _font

    pygame.init()
    _screen = pygame.display.set_mode(SCREENSIZE, pygame.FULLSCREEN)
    _font = pygame.font.SysFont(FONTNAME, FONTSIZE)

    thread = threading.Thread(target = _run)
    thread.daemon = True
    thread.start()


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

