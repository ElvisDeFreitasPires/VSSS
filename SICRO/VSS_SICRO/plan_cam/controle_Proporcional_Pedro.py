import numpy as np
import math
import pygame

def T2D(M1, deltax, deltay):
    T = np.array([[1,0,deltax],
                  [0,1,deltay],
                  [0,0,1]])
    return T@M1

def Rz2D(M1, th):
    Rz = np.array([[np.cos(th), -np.sin(th),0],
                   [np.sin(th), np.cos(th), 0],
                   [0,0,1]])
    return Rz@M1

def normalize(angle):
	return np.arctan2(np.sin(angle),np.cos(angle))

def ajustaangulo(angulo):
        # angulo = abs(angulo)
        # if angulo>math.pi:
        #     angulo = angulo-2*math.pi
        # return angulo
        # def ajustaangulo(angulo):
    # Adjusts angle to range [-pi, pi]
    while angulo > math.pi:
        angulo -= 2 * math.pi
    while angulo < -math.pi:
        angulo += 2 * math.pi
    return angulo
def hud(P,v,w):
    
    screen.blit(font.render(f"V: {v}", True, "white"), (20, 720-140))
    screen.blit(font.render(f"W: {w}", True, "white"), (20, 720-120))
    # screen.blit(font.render(f"wl: {np.rad2deg(wl)}", True, "white"), (20, 720 - 100))
    screen.blit(font.render(f"dx: {dx}", True, "white"), (20, 720 - 100))
    # screen.blit(font.render(f"wr: {np.rad2deg(wr)}", True, "white"), (20, 720 - 80))
    screen.blit(font.render(f"dy: {dy}", True, "white"), (20, 720 - 80))
    screen.blit(font.render(f"x: {P[0]}", True, "white"), (20, 720 - 60))
    screen.blit(font.render(f"y: {P[1]}", True, "white"), (20, 720 - 40))
    screen.blit(font.render(f"theta: {normalize(P[2])}", True, "white"), (20, 720-20))
    




pygame.init()
screen_dim = (1280,720)
screen = pygame.display.set_mode(screen_dim)
clock = pygame.time.Clock()
running = True
font = pygame.font.Font(None, 24)
dt = 0.1
escala = 150
# Corpo = np.array([[100, 227.5,227.5,100, -200,-227.5,-227.5,-200],
#                   [-190.5,-50,50,190.5, 190.5,163,-163,-190.5],
#                   [1000]*8]) / 1000
# RodaE = np.array([[97.5,97.5,-97.5,-97.5],
#                   [170.5, 210.5, 210.5,170.5],
#                   [1000]*4])/1000
# RodaD = np.array([[97.5,97.5,-97.5,-97.5],
#                   [-170.5, -210.5, -210.5,-170.5],
#                   [1000]*4])/1000
corpo = np.array([[100   , -190.5],      
                  [227.5 , -50   ],
                  [227.5 , 50    ],
                  [100   , 190.5 ],
                  [-200  , 190.5 ],
                  [-227.5, 163   ],
                  [-227.5, -163  ],
                  [-200  , -190.5]])*(escala/1000)
corpo = np.hstack((corpo, np.ones((corpo.shape[0], 1)))).T
# print(corpo)
rodaE = np.array([[ 97.5, 170.5],
                  [ 97.5, 210.5],
                  [-97.5, 210.5],
                  [-97.5, 170.5]])*(escala/1000)
rodaE = np.hstack((rodaE, np.ones((rodaE.shape[0], 1)))).T

rodaD = np.array([[ 97.5, -170.5],
                  [ 97.5, -210.5],
                  [-97.5, -210.5],
                  [-97.5, -170.5]])*(escala/1000)
rodaD = np.hstack((rodaD, np.ones((rodaD.shape[0], 1)))).T
p0 = [100,100]
G = np.array([800,500,np.deg2rad(0)])
P = np.array([p0[0], p0[1],np.deg2rad(0)])

# r = escala*(0.5 * (195/1000))
# l = escala*(0.5 * (381/1000))

r = escala*(0.5 * (17/1000))
l = escala*(0.5 * (37.5/1000))

# wl = np.deg2rad(10)
# wr = np.deg2rad(20)
# v = (r/2) * (wr + wl)
# w = (r/(2*l)) * (wl - wr) #pygame considera y+ para baixo, então a diferença das w das rodas foram invertidas
dx = G[0] - P[0]
dy = G[1] - P[1]
dth = G[2] - P[2]
rho = math.sqrt(dx**2 + dy**2)
# print(dy,dx,dth)
gamma = ajustaangulo(math.atan2(dy,dx))
alpha = ajustaangulo(gamma - P[2])
beta = ajustaangulo(G[2] - gamma)
krho = 2/11
kalpha = 345/823
kbeta = -2/11

sim_running = False

while running:

    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            sim_running = not sim_running
    if sim_running:
        # fill the screen with a color to wipe away anything from last frame
        screen.fill("black")
        # RENDER YOUR GAME HERE
        # print(th)
        dx = G[0] - P[0]
        dy = G[1] - P[1]
        dth = G[2] - P[2]
        rho = math.sqrt(dx**2 + dy**2)
        gamma = ajustaangulo(math.atan2(dy,dx))
        alpha = ajustaangulo(gamma - P[2])
        beta = ajustaangulo(G[2] - gamma)
        
        v = krho * rho
        if alpha > math.pi/2:
            v = -krho * rho
            alpha = ajustaangulo(gamma + math.pi)
            beta = ajustaangulo(beta + math.pi)

            
        w = (kalpha*alpha + kbeta * beta)
        # v = (r/2) * (wr + wl)
        # print(v)
        # w = (r/(2*l)) * (wl - wr)
        # th = normalize(P[2])
        th = P[2]
        # print(th)
        # deltaS = v*dt
        # deltath = w*dt

        # deltaP = np.array([deltaS*math.cos(th + deltath/2),
        #                deltaS*math.sin(th + deltath/2),
        #                deltath])
        
        dPdt = np.array([[v * math.cos(th)],
                    [v * math.sin(th)],
                    [w]])
        
            
        # P = P + deltaP
        P = P + dPdt.flatten() * dt
        P[2] = ajustaangulo(P[2])
        hud(P,v,w)
        # P = P + dP*dt
        # print(th)
        RoboC = T2D(Rz2D(corpo, th), P[0], P[1])
        RoboE = T2D(Rz2D(rodaE, th), P[0], P[1])
        RoboD = T2D(Rz2D(rodaD, th), P[0], P[1])
        # print(RoboC.shape)
        # print(RoboD.shape)
        # print(RoboE.shape)
        # flip() the display to put your work on screen
        # print(RoboD[:2,:])
        # pygame.draw.polygon(screen, "red",RoboC[:2,:].T)
        # pygame.draw.polygon(screen, "red",RoboE[:2,:].T)
        # pygame.draw.polygon(screen, "red",RoboD[:2,:].T)
        # RoboC_xy = RoboC[:2, :]
        # RoboE_xy = RoboE[:2, :]
        # RoboD_xy = RoboD[:2, :]
        # print(RoboD_xy)
        pygame.draw.circle(screen, "red", (p0[0], p0[1]),5)
        pygame.draw.circle(screen, "white", (G[0],G[1]),5)
        pygame.draw.polygon(screen, "cyan", RoboC[:2, :].T)
        pygame.draw.polygon(screen, "blue", RoboE[:2, :].T)
        pygame.draw.polygon(screen, "blue", RoboD[:2, :].T)
        pygame.draw.circle(screen, "black", (P[0], P[1]),5)
        pygame.display.flip()

        dt = clock.tick(60)/1000

    else:
        dt = clock.tick(60)/1000
