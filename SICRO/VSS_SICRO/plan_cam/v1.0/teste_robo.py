import pygame
import numpy as np
from pygame.locals import *
from sys import exit
from robo import Robo
import time

pygame.init()
v1=30
v2=20
SIZE = [920,840]
x=0
y=0
r1 = Robo(0,0,0)
# r2 = Robo(500,300,0)
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
    
    #r1.perfumaria(screen,100,100,-0*np.pi)
    # print(r1.moveRobo(screen,v1,v2))
    #print(r2.moveRobo(screen,0,0))
    #print(r2.moveRobo(screen,30,0))
    #pygame.draw.circle(screen,(255,0,0),r1.moveRobo(screen,0,0),1)
    #pygame.draw.circle(screen,(255,0,0),r2.moveRobo(screen,0,0),1)
    #  (1/2 * (381/1))

    r1.cinematicaDireta(30,0)
    r1.draw(screen)
    # time.sleep(1)

    pygame.display.update()

