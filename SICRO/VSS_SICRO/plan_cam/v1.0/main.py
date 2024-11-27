import pygame
import numpy as np
from pygame.locals import *
from sys import exit
# from modelo import ModeloRobo
import controle_proporcional as cp

pygame.init()

SIZE = [1000,780]

p = np.array([200,200,0])
o = np.array([800,200,0])

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
    
    
    cp.calcVariacaoDeslocamentoP(p,o)


    pygame.display.update()

