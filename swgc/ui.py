# -*- coding: utf-8 -*-
"""The SWGC UI.
"""
import time, threading, collections, traceback
import pygame

from . import sync
from . settings import FONTNAME, FONTSIZE, CLEAR_COLOR
from . framebuffer import get as get_framebuffer


class UI(threading.Thread):
    """Thread controling the SWGC display.
    """
    def __init__(self, gps, uav, board):
        super(UI, self).__init__()
        self.gps, self.uav, self.board = gps, uav, board
        self.daemon = True

    def run(self):
        while 1:
            try:
                _run(self)
            except BaseException, e:
                print e
                traceback.print_exc()
                time.sleep(5)


_screen, _font = None, None


def _run(self):
    """Internal.
    The UI thread.
    """
    global _screen, _font

    _screen = get_framebuffer()
    _font = pygame.font.SysFont(FONTNAME, FONTSIZE)
    _clear()

    while 1:
        texts = []
        texts.append('%s UTC\n' % self.gps.dt if self.gps.dt else '......\n')


        gpos = self.gps.get_position()
        upos = self.uav.get_position()
        bpos = self.board.get_position()


        if upos:
            head = self.uav.get_heading()
            head = '%7i\xb0' % head if not head is None else ''

            text = \
 """UAV Current Position
 Latitude  %3.5f
 Longitude %3.5f
 Altitude  %7im
 Heading   %s
 """ % (upos.lat, upos.lon, upos.alt, head)

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
                dist, head = gpos.get_distance_and_heading(bpos)
                dist = int(dist)
                head = '%7i\xb0' % head if dist else ''
                dist = '%7im'    % dist
            else:
                dist, head = '', ''

            text = \
"""UAV Target Position
 Latitude  %3.5f
 Longitude %3.5f
 Distance  %s
 Heading   %s
 """ % (bpos.lat, bpos.lon, dist, head)

        else:
            text = \
"""UAV Target Position
 Latitude
 Longitude
 Distance
 Heading
"""
        texts.append(text)


        text = self.board.get_message()
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
