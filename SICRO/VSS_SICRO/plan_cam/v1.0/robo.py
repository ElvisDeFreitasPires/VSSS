import numpy as np
import pygame
import math

# from teste_robo import screen


class Robo:
    def __init__(self,x,y,ang):
        #raio da roda
        self.r = (1/2 * (195/1000))*262 #x262 pois é em pixels
        #Centro do eixo
        self.l = (1/2 * (381/1000))*262 #x262 pois é em pixels
        
        #Posição (x,y)
        self.P0 = np.array([[x],[y],[ang]])
        # self.x = x #posição em x
        # self.y = y #posição em y
        # self.ang = ang

        # Frame base do robo
        f0Corpo =    np.array([[-10-self.r,-50],[50,-50],[100,-25],[100,25],[50,50],[-10-self.r,50]])
        
        #Pose do robo
        self.Pose = f0Corpo
        self.rotaciona(ang)
        self.translada(x,y)
        # self.F = self.rotaciona(ang)  # antigo rotaciona
        # self.F = self.translada(x,y) # antigo translada
        self.Roda = np.array([[0,0],[x,y]])

    #def draw(self, screen, ang, posXY):
    def draw(self,screen):
        pygame.draw.polygon(screen,(255,0,0),self.Pose)
        # pygame.draw.polygon(screen,(0,255,0),self.F)
        pygame.draw.circle(screen,(0,255,0),(self.P0[0][0],self.P0[1][0]),5)
        # pygame.draw.line(screen,(0,0,255),(self.x-self.r,self.y-self.l),(self.x+self.r,self.y-self.l),10)
        # pygame.draw.line(screen,(0,0,255),(self.x-self.r,self.y+self.l),(self.x+self.r,self.y+self.l),10)
        # pygame.draw.line(screen,(0,0,255),self.RodaR[0],self.RodaR[1],10)
    
    def rotaciona(self,ang):
        # self.Pose = f0Corpo           # F recebe o frame inercial fixo na origem
        self.Pose = np.array([[-10-self.r,-50],[50,-50],[100,-25],[100,25],[50,50],[-10-self.r,50]])
        # print("P0ang: ",self.P0[2][0])
        rad = math.radians(ang)    # ang convertido de grau -> rad
        sen = math.sin(rad)
        cos = math.cos(rad)
        #Matriz de rotação
        R = np.array([[cos,-sen],
              [sen,cos]])
        # Multoplicação
        for i,Fr in enumerate(self.Pose):
            self.Pose[i]=Fr@R

    
    def translada(self,deltaX,deltaY):
        #Matriz de translação
        T = np.array([[1,0,deltaX],
                      [0,1,deltaY],
                      [0,0,1]])
        
        for i,Pl in enumerate(self.Pose): #Translada cada ponto do objeto
            # print("Pl[",i,"]: ",Pl[0],", ",Pl[1])
            Faux = np.array([[1,0,Pl[0]],
                             [0,1,Pl[1]],
                             [0,0,1]])
            Faux = Faux@T
            self.Pose[i][0] = Faux[0][2]
            self.Pose[i][1] = Faux[1][2]



    # def calcVariacaoDeslocamento(self,wl,wr):
    #     vL= self.velLvelW(wl,wr)
    #     #print(self.vW)
    #     dP = np.array([ [vL * np.cos(self.vW)],
    #            [vL * np.sin(self.vW)],
    #            [self.vW]])
    #     return dP
    
    # def velLvelW(self,wr,wl):
    #     vL = self.r/2 * (wr + wl)
    #     vW = (self.r/(2*self.l) * (wr - wl))/10
    #     return vL,vW
    
    def cinematicaDireta(self,wl,wr): #move o robo atuando na vel das rodas
        wl = np.deg2rad(wl)
        wr = np.deg2rad(wr)

        vL = self.r/2 * (wr + wl)
        vW = (self.r/(2*self.l) * (wr - wl))

        # self.F = self.F + self.calcVariacaoDeslocamento(wl,wr) * 0.119 
        dP = np.array([ [vL * np.cos(vW)],
               [vL * np.sin(vW)],
               [vW]])
        
        dP = dP * 0.119

        self.P0 = self.P0 + dP

        self.rotaciona(np.rad2deg(self.P0[2][0]))
        self.translada(self.P0[0][0],self.P0[1][0])
        #print("dP:\n",dP)
        print("P0:\n",self.P0)
        print("P0_flat:\n",self.P0.flatten())


        # print("VL: ",self.vL," Vw: ",self.vW)
        # print("F_inercial:\n",self.f0Corpo)