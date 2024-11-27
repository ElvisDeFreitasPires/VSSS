import numpy as np
from modelo import ModeloRobo

Krho = (2/11)
Kalpha = (345/823)
Kbeta = (-2/11)

delta1 = 10
delta2 = 10




def ajustaAngulo(ang):
    #ang = np.deg2rad(ang)
    if(abs(ang)>np.pi):
        ang = ang - 2*np.pi
    return ang

def velLvelW(wr,wl):
    vL = ModeloRobo.r/2 * (wr + wl)
    vW = (ModeloRobo.r/(2*ModeloRobo.l) * (wl - wr))/10
    return vL,vW
    
def calcVariacaoDeslocamento(wl,wr):
    vL,vW= velLvelW(wl,wr)
    #print(self.vW)
    dP = np.array([ [vL * np.cos(vW)],
                    [vL * np.sin(vW)],
                    [vW]])
    return dP
    


def calcVariacaoDeslocamentoP(screen,p,o):
        
    Erro = o - p
    rho = np.sqrt(Erro[0]*Erro[0] + Erro[1]*Erro[1])
    #th = np.rad2deg(np.arctan2(Erro[0],Erro[1]))
    gamma = ajustaAngulo(np.arctan2(Erro[1],Erro[0]))
    alpha = ajustaAngulo(gamma - np.deg2rad(o[2]))
    beta  = ajustaAngulo(np.deg2rad(p[2]) - gamma)
    
    v = Krho * rho
    w = Kalpha*alpha - Kbeta*beta
    #w *= -1
    print("Rho:",rho)
    dPdt = np.array( (v*np.cos(p[2]),
                      v*np.sin(p[2]),
                      w))
    p = p + dPdt * 0.119
    print("dPdt:",dPdt)
    if( rho>delta1 or abs(alpha)>delta2 or abs(beta)>delta2):
    # if( rho>delta1 ):
    
        #print("P:",p)
        # r = Robo(p[0],p[1],np.rad2deg(p[2]))
        
        
        screen.fill("black")
        r = ModeloRobo(p[0],p[1],np.rad2deg(p[2]))

        # time.sleep(0.1)
        # r.draw(screen)
        
        calcVariacaoDeslocamentoP(screen,p,o)
        
            