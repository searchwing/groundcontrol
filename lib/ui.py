# -*- coding: utf-8 -*-
import pygame


screen, font = None, None


def init():
    global screen, font

    pygame.init()
    screen = pygame.display.set_mode((800, 480), pygame.FULLSCREEN)
    font = pygame.font.SysFont("comicsansms", 72)


def text(text):
    global screen, font

    text = text.split('\n')
    text = [font.render(t, True, (0, 0, 0)) for t in text]
    screen.fill((255, 255, 255))
    y = 20
    for t in text:
        screen.blit(t, (20, y))
        y += 40
    pygame.display.flip()

