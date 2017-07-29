# -*- coding: utf-8 -*-
"""Get pygame framebuffer device.
"""
import os
import pygame


CLEAR_COLOR = (0,0,0)


screen = None

    
def get():
    global screen
    if screen:
        return screen

    disp_no = os.getenv("DISPLAY")
    if disp_no:
        print "Running under X display = {0}".format(disp_no)

    drivers = ('directfb', 'fbcon', 'svgalib')
    
    found = False
    for driver in drivers:
        if not os.getenv('SDL_VIDEODRIVER'):
            os.putenv('SDL_VIDEODRIVER', driver)
        try:
            pygame.display.init()
        except pygame.error:
            pass
        else:
            found = True
            break

    if not found:
        raise Exception('No suitable video driver found!')
    
    size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
    print "Framebuffer size: %d x %d" % (size[0], size[1])

    pygame.mouse.set_visible(False)

    screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
    pygame.font.init()

    screen.fill(CLEAR_COLOR)
    pygame.display.update()

    return screen


def test():
    import time
    screen = get()
    screen.fill((255,0,0))
    pygame.display.update()
    time.sleep(5)


if __name__ == '__main__':
    test()


