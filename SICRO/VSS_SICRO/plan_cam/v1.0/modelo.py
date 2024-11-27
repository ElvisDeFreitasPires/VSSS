import numpy as np
import pygame
from robo import Robo
class ModeloRobo:
    #frameInercial 
    P = [[0],[0],[0]]
    r = 1/2 * (195/10)
    l = 1/2 * (381/10)
    def __init__(self,x,y):
        self.wl = 0
        self.wr = 0
        self.vL = 0
        self.vW = 0
        self.F = np.array([[x],[y],[0]])
        #self.dP = [[0],[0],[0]]
    def velLvelW(self,wr,wl):
        vL = self.r/2 * (wr + wl)
        self.vW += (self.r/(2*self.l) * (wr - wl))/10
        return vL
    
    def calcVariacaoDeslocamento(self,wl,wr):
        vL= self.velLvelW(wl,wr)
        #print(self.vW)
        dP = np.array([ [vL * np.cos(self.vW)],
               [vL * np.sin(self.vW)],
               [self.vW]])
        return dP
    def moveRobo(self,screen,wl,wr):
        wl = np.deg2rad(wl)
        wr = np.deg2rad(wr)
        self.F = self.F + self.calcVariacaoDeslocamento(wl,wr) * 0.119
        #print(self.F)
        #pygame.draw.circle(screen,(255,0,0),(self.F[0][0],self[1][0]),10)
        self.perfumaria(screen,self.F[0][0],self.F[1][0],self.vW)
        return self.F[0][0],self.F[1][0]
    
    def perfumaria(self,screen,x,y,w):
        x_ref = x
        y_ref = y
        
        pts0 = np.array(((-self.l,-self.l),(self.l,-self.l),(self.l,self.l),(-self.l,self.l)))

        rt = np.array(((np.cos(self.vW),-np.sin(self.vW)), (np.sin(self.vW),np.cos(self.vW)))) 
        pts = pts0
        
        pts[0] = pts[0] @ rt 
        pts[1] = pts[1] @ rt
        pts[2] = pts[2] @ rt
        pts[3] = pts[3] @ rt
        

        
        #print("Pts0:\n", pts0)
        #print("pts:\n", pts)
        pts = np.array(((-self.l+x_ref,-self.l+y_ref),(self.l+x_ref,-self.l+y_ref),(self.l+x_ref,self.l+y_ref),(-self.l+x_ref,self.l+y_ref)))
        print("x: ",x)
        print("y: ",y)
        print("w: ",-np.rad2deg(w))
        r = Robo(x,y,-np.rad2deg(w))
        r.draw(screen)
        #pygame.draw.polygon(screen,(0,255,0), pts)
        #pygame.draw.circle(screen,(255,0,0),(x_ref,y_ref),1)




# import numpy as np
# import pygame
# from robo import Robo
# from main import screen

# font = pygame.font.Font(None, 24)

# class ModeloRobo:
#     #frameInercial 
#     P = [[0],[0],[0]]
#     r = 1/2 * (195/10)
#     l = 1/2 * (381/10)
#     def __init__(self,s,x,y,ang):
#         self.wl = 0
#         self.wr = 0
#         self.vL = 0
#         self.vW = 0
#         self.F = np.array([[x],[y],[ang]])
#         self.perfumaria(s, x, y, ang)
#         #self.dP = [[0],[0],[0]]
#     def velLvelW(self,wr,wl):
#         vL = self.r/2 * (wr + wl)
#         vW += (self.r/(2*self.l) * (wr - wl))/10
#         return vL,vW
    
#     def calcVariacaoDeslocamento(self,wl,wr):
#         vL= self.velLvelW(wl,wr)
#         #print(self.vW)
#         dP = np.array([ [vL * np.cos(self.vW)],
#                [vL * np.sin(self.vW)],
#                [self.vW]])
#         return dP
#     def moveRobo(self,wl,wr):
#         wl = np.deg2rad(wl)
#         wr = np.deg2rad(wr)
#         self.F = self.F + self.calcVariacaoDeslocamento(wl,wr) * 0.119
#         #print(self.F)
#         #pygame.draw.circle(screen,(255,0,0),(self.F[0][0],self[1][0]),10)
#         self.perfumaria(screen,self.F[0][0],self.F[1][0],self.vW)
#         return self.F[0][0],self.F[1][0]
    
#     def perfumaria(self,x,y,w):
#         x_ref = x
#         y_ref = y
        
#         pts0 = np.array(((-self.l,-self.l),(self.l,-self.l),(self.l,self.l),(-self.l,self.l)))

#         rt = np.array(((np.cos(self.vW),-np.sin(self.vW)), (np.sin(self.vW),np.cos(self.vW)))) 
#         pts = pts0
        
#         pts[0] = pts[0] @ rt 
#         pts[1] = pts[1] @ rt
#         pts[2] = pts[2] @ rt
#         pts[3] = pts[3] @ rt
        

        
#         #print("Pts0:\n", pts0)
#         #print("pts:\n", pts)
#         pts = np.array(((-self.l+x_ref,-self.l+y_ref),(self.l+x_ref,-self.l+y_ref),(self.l+x_ref,self.l+y_ref),(-self.l+x_ref,self.l+y_ref)))
#         print("x: ",x)
#         print("y: ",y)
#         print("w: ",-np.rad2deg(w))
#         r = Robo(x,y,-np.rad2deg(w))
#         r.draw(screen)

#         #pygame.draw.polygon(screen,(0,255,0), pts)
#         #pygame.draw.circle(screen,(255,0,0),(x_ref,y_ref),1)

    
# def hud(self,P):
    
#     screen.blit(font.render(f"wl: {np.rad2deg(self.wl)}", True, "white"), (20, 720 - 100))
#     screen.blit(font.render(f"wr: {np.rad2deg(self.wr)}", True, "white"), (20, 720 - 80))
#     screen.blit(font.render(f"x: {P[0]}", True, "white"), (20, 720 - 60))
#     screen.blit(font.render(f"y: {P[1]}", True, "white"), (20, 720 - 40))
#     screen.blit(font.render(f"theta: {(P[2])}", True, "white"), (20, 720-20))


    
