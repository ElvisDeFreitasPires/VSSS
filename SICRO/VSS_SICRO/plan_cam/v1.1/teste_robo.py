import pygame
import numpy as np
from pygame.locals import *
from sys import exit
from robo import Robo

pygame.init()

SIZE = [920,840]

r1 = Robo(100,100,0)

screen = pygame.display.set_mode(SIZE)
pygame.display.set_caption('Jogo')
relogio = pygame.time.Clock()
while True:
    relogio.tick(60)
    screen.fill((0,0,0))
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            exit()
    
    pygame.draw.polygon(screen, "cyan", r1.RoboC[:2, :].T)
    pygame.draw.polygon(screen, "blue", r1.RoboE[:2, :].T)
    pygame.draw.polygon(screen, "blue", r1.RoboD[:2, :].T)

    pygame.display.update()

