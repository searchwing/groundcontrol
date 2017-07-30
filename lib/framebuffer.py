# -*- coding: utf-8 -*-
"""Get pygame framebuffer device.
"""
import os
import pygame


    
def get():
    #pygame.init()
    #pygame.display.init()
    #pygame.font.init()
    #return pygame.display.set_mode((480, 800), pygame.HWSURFACE | pygame.DOUBLEBUF)


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
    import time
    get().fill((255,0,0))
    pygame.display.update()
    time.sleep(5)

if __name__ == '__main__':
    test()

