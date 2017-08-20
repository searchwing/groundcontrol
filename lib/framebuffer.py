# -*- coding: utf-8 -*-
"""Frame buffer stuff.
"""
import os
import pygame



    
def get():
    """Get pygame compatible framebuffer device.
    """

    disp_no = os.getenv("DISPLAY")
    if disp_no:
        print "Running under X display = {0}".format(disp_no)

    found = False
    for driver in ('directfb', 'fbcon', 'svgalib'):
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

    pygame.init()
    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
    return screen



def test():
    """Test framebuffer, fill it with red, wait for 5 seconds.
    """
    import time
    get().fill((255,0,0))
    pygame.display.update()
    time.sleep(5)
