import numpy as np
import pygame
import math

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
    
class Robo:
    def __init__(self,x,y,ang):

        self.P = np.array([x, y,np.deg2rad(ang)])

        Robo.escala = 150
        
        Robo.r = Robo.escala*(0.5 * (195/1000))
        Robo.l = Robo.escala*(0.5 * (381/1000))

        Robo.corpo = np.array([[100   , -190.5],      
                  [227.5 , -50   ],
                  [227.5 , 50    ],
                  [100   , 190.5 ],
                  [-200  , 190.5 ],
                  [-227.5, 163   ],
                  [-227.5, -163  ],
                  [-200  , -190.5]])*(Robo.escala/1000)
        Robo.corpo = np.hstack((Robo.corpo, np.ones((Robo.corpo.shape[0], 1)))).T
        print(Robo.corpo)
        Robo.rodaE = np.array([[ 97.5, 170.5],
                          [ 97.5, 210.5],
                          [-97.5, 210.5],
                          [-97.5, 170.5]])*(Robo.escala/1000)
        Robo.rodaE = np.hstack((Robo.corpo, np.ones((Robo.corpo.shape[0], 1)))).T

        Robo.rodaD = np.array([[ 97.5, -170.5],
                          [ 97.5, -210.5],
                          [-97.5, -210.5],
                          [-97.5, -170.5]])*(Robo.escala/1000)
        Robo.rodaD = np.hstack((Robo.rodaD, np.ones((Robo.rodaD.shape[0], 1)))).T

        
        print("corpo: ",Robo.corpo)
        print("th: ",self.P[2])
        th = self.P[2]
        self.RoboE = T2D(Rz2D(Robo.rodaE, self.P[2]), self.P[0], self.P[1])
        self.RoboD = T2D(Rz2D(Robo.rodaD, self.P[2]), self.P[0], self.P[1])
        self.RoboC = T2D(Rz2D(Robo.corpo, self.P[2]), self.P[0], self.P[1])

    def cinematicaDireta(self, wr , wl):

        wl = np.deg2rad(wl)
        wr = np.deg2rad(wr)

        v = Robo.r/2 * (wr + wl)
        w = (Robo.r/(2*Robo.l) * (wr - wl))

        dPdt = np.array([[v * math.cos(self.P[2])],
                    [v * math.sin(self.P[2])],
                    [w]])
        
        # P = P + deltaP
        self.P = self.P + dPdt.flatten() *0.1
        self.P[2] = ajustaangulo(self.P[2])