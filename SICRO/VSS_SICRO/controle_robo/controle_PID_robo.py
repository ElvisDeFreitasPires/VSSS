import numpy as np
import math
import pygame
import cv2
import serial #Importa a biblioteca

while True: #Loop para a conexão com o Arduino
    try:  #Tenta se conectar, se conseguir, o loop se encerra
        arduino = serial.Serial('COM7', 9600)
        print('Arduino conectado')
        break
    except:
        print('.')
        pass


#visao
cor_laranja = np.array([[0, 143, 20],[15, 255, 255]], np.uint8)
#cor_azul = np.array([[x, x, x],[x, x, x]], np.uint8)
cor_amarelo = np.array([[17, 100, 20],[40, 255, 255]], np.uint8)
cor_verde = np.array([[35, 70, 20],[85, 255, 255]], np.uint8)
#cor_vermelha = np.array([[x, x, x],[x, x, x]], np.uint8)
#cor_roxa = np.array([[x, x, x],[x, x, x]], np.uint8)


verde_baixo = np.array([35, 70, 20], np.uint8)
verde_alto = np.array([85, 255, 255], np.uint8)
amarelo_baixo = np.array([17, 100, 20], np.uint8)
amarelo_alto = np.array([40, 255, 255], np.uint8)

def detecta_robo(frame,cor_time,cor_id):
    
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    h, s, v = cv2.split(frameHSV)
    s = cv2.add(s, 50)
    v = cv2.add(v, -100)
    frameHSV = cv2.merge([h, s, v])
    #cor time
    #mascaraAmarela = cv2.inRange(frameHSV, amarelo_baixo, amarelo_alto)
    mascaraAmarela = cv2.inRange(frameHSV, cor_time[0], cor_time[1])
    
    #cor id
    #mascaraVerde = cv2.inRange(frameHSV, verde_baixo, verde_alto)
    mascaraVerde = cv2.inRange(frameHSV, cor_id[0], cor_id[1])


    mascaraVerde = cv2.erode(mascaraVerde, None, iterations=1)
    mascaraVerde = cv2.dilate(mascaraVerde, None, iterations=2)
    mascaraAmarela = cv2.erode(mascaraAmarela, None, iterations=1)
    mascaraAmarela = cv2.dilate(mascaraAmarela, None, iterations=2)

    mascaraVerde = cv2.medianBlur(mascaraVerde, 13)
    mascaraAmarela = cv2.medianBlur(mascaraAmarela, 5)

    Frente_robo, _ = cv2.findContours(mascaraAmarela, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Tras_robo, _ = cv2.findContours(mascaraVerde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centro_frontal = None
    centro_traseiro = None
    ponto_mediox = 0
    ponto_medioy = 0
    theta = 0
    
    for c in Frente_robo:
        area = cv2.contourArea(c)
        if area > 700:
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = 1
            Frente_x = int(M["m10"] / M["m00"])
            Frente_y = int(M["m01"] / M["m00"])
            centro_frontal = (Frente_x, Frente_y)
            ContornoFrente = cv2.convexHull(c)
            cv2.drawContours(frame, [ContornoFrente], 0, (255, 0, 0), 3)

    for c in Tras_robo:
        area = cv2.contourArea(c)
        if area > 700:
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = 1
            Tras_x = int(M["m10"] / M["m00"])
            Tras_y = int(M["m01"] / M["m00"])
            centro_traseiro = (Tras_x, Tras_y)
            novoContorno = cv2.convexHull(c)
            cv2.drawContours(frame, [novoContorno], 0, (255, 0, 0), 3)

    if centro_frontal and centro_traseiro:
        ponto_mediox = int((centro_frontal[0] + centro_traseiro[0]) / 2)
        ponto_medioy = int((centro_frontal[1] + centro_traseiro[1]) / 2)

        vetor = np.array([centro_frontal[0] - centro_traseiro[0], centro_frontal[1] - centro_traseiro[1]])
        theta = np.arctan2(vetor[1], vetor[0])
        theta_deg = np.degrees(theta)

        if theta_deg < 0:
            theta_deg += 360

        tamanho_seta = 40
        seta_fim_x = int(ponto_mediox + tamanho_seta * np.cos(theta))
        seta_fim_y = int(ponto_medioy + tamanho_seta * np.sin(theta))
        cv2.arrowedLine(frame, (ponto_mediox, ponto_medioy), (seta_fim_x, seta_fim_y), (0, 0, 255), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, 'theta: {:.2f} graus'.format(theta_deg), (10, 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.circle(frame, (ponto_mediox + 10, ponto_medioy), 7, (0, 255, 255), -1)
        cv2.putText(frame, f'Jarvan I: {ponto_mediox}, {ponto_medioy}', (ponto_mediox + 10, ponto_medioy), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

    return frame, ponto_mediox, ponto_medioy, theta



laranja_baixo = np.array([0, 143, 20], np.uint8)
laranja_alto = np.array([15, 255, 255], np.uint8)


def detecta_bola(frame,cor):
    Bola_x = 0
    Bola_y = 0
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Ajustar a saturação e valor
    h, s, v = cv2.split(frameHSV)
    s = cv2.add(s, 30)
    v = cv2.add(v, -180)
    frameHSV = cv2.merge([h, s, v])
    
    #mascaraLaranja=cv2.inRange(frameHSV,laranja_baixo,laranja_alto)
    mascaraLaranja=cv2.inRange(frameHSV,cor[0],cor[1])
    
    #mascaraAmarela = cv2.inRange(frameHSV, amarelo_baixo, amarelo_alto)
    #mascaraVermelha1 = cv2.inRange(frameHSV, vermelho_baixo1, vermelho_alto1)
    #mascaraVermelha2 = cv2.inRange(frameHSV, vermelho_baixo2, vermelho_alto2)
    #mascaraVermelha = cv2.add(mascaraVermelha1, mascaraVermelha2)

    #mascaraVermelha = cv2.erode(mascaraVermelha, None, iterations=1)
    #mascaraVermelha = cv2.dilate(mascaraVermelha, None, iterations=2)
    #mascaraVermelha = cv2.medianBlur(mascaraVermelha, 13)

   # mascaraAmarela = cv2.erode(mascaraAmarela, None, iterations=1)
    #mascaraAmarela = cv2.dilate(mascaraAmarela, None, iterations=2)
    #mascaraAmarela = cv2.medianBlur(mascaraAmarela, 5)

    Bola, _ = cv2.findContours(mascaraLaranja, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in Bola:
        area = cv2.contourArea(c)
        if area > 100:
            M = cv2.moments(c)
            if M['m00'] == 0:
                M['m00'] = 1
            Bola_x = int(M['m10'] / M['m00'])
            Bola_y = int(M['m01'] / M['m00'])

            cv2.circle(frame, (Bola_x, Bola_y), 7, (0, 255, 0), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f'Bola {Bola_x},{Bola_y}', (Bola_x + 10, Bola_y), font, 0.75, (0, 255, 255), 1, cv2.LINE_AA)
            contornoBola = cv2.convexHull(c)
            cv2.drawContours(frame, [contornoBola], 0, (0, 0, 255), 3)

    return frame, Bola_x, Bola_y
#end visao

# controle PID

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
def draw_first_frame():
    screen.fill("black")
    RoboC = T2D(Rz2D(corpo, P[2]), P[0], P[1])
    RoboE = T2D(Rz2D(rodaE, P[2]), P[0], P[1])
    RoboD = T2D(Rz2D(rodaD, P[2]), P[0], P[1])
    pygame.draw.circle(screen, "red", (screen_dim[0] / 2, screen_dim[1] / 2), 5)  # Starting point
    pygame.draw.circle(screen, "white", (G[0], G[1]), 5)                         # Goal point
    pygame.draw.polygon(screen, "cyan", RoboC[:2, :].T)                          # Robot body
    pygame.draw.polygon(screen, "blue", RoboE[:2, :].T)                          # Left wheel
    pygame.draw.polygon(screen, "blue", RoboD[:2, :].T)                          # Right wheel
    pygame.draw.circle(screen, "black", (P[0], P[1]), 5)                         # Robot center
    
    hud(P)
    
    pygame.display.flip()

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
def hud(P):
    
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

P = np.array([screen_dim[0]/2, screen_dim[1]/2,np.deg2rad(90)])
G = np.array([1100, 600])

# r = escala*(0.5 * (195/1000))
# l = escala*(0.5 * (381/1000))


r = escala*(0.5 * (17/1000))
l = escala*(0.5 * (42.5/1000))


# wl = np.deg2rad(10)
# wr = np.deg2rad(20)
# v = (r/2) * (wr + wl)
# w = (r/(2*l)) * (wl - wr) #pygame considera y+ para baixo, então a diferença das w das rodas foram invertidas
dx = G[0] - P[0]
dy = G[1] - P[1]
# dth = G[2] - P[2]
rho = math.sqrt(dx**2 + dy**2)
# print(dy,dx,dth)
gamma = ajustaangulo(math.atan2(dy,dx))
alpha = ajustaangulo(gamma - P[2])
# beta = ajustaangulo(G[2] - gamma)
# krho = 2/11
# kalpha = 345/823
# kbeta = -2/11
krho = 1
kp = 10
ki = 0.01
kd = 1
dif_alpha = 0
int_alpha = 0
alpha_old = 0
vmax = 30
wmax = math.radians(30)

sim_running = False
draw_first_frame()
#end controle proporcional

#visao
# cap = cv2.VideoCapture('http://192.168.101.139:4747/video')
cap = cv2.VideoCapture(2)
#end visao

env_vLvW = ""


while running:

    
    #visao
    ret, frame = cap.read()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break
    _,x_bola,y_bola = detecta_bola(frame,cor_laranja)
    _,x_r1,y_r1,th_r1 = detecta_robo(frame,cor_amarelo,cor_verde)
 #   _,x_r2,y_r2,th_r2 = detecta_robo(frame,cor_amarelo,cor_verde)   #criar as cores para os outros robo
 #   _,x_r3,y_r3,th_r3 = detecta_robo(frame,cor_amarelo,cor_verde)

    _,x_r1,y_r1,th_r1 = detecta_robo(frame,cor_amarelo,cor_verde)
    print("x_bola:", x_bola," y_bola:",y_bola)
    print("x_robo1:", x_r1," y_robo1:",y_r1," th1:",np.rad2deg(th_r1))
    cv2.imshow('frame',frame)
    #end visao


    #controle
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            cv2.destroyAllWindows()
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            sim_running = not sim_running

    
    if sim_running:
        # fill the screen with a color to wipe away anything from last frame
        
        P = np.array([x_r1,y_r1,th_r1])
        G = np.array([x_bola,y_bola,np.deg2rad(0)])
        
        screen.fill("black")
        # RENDER YOUR GAME HERE
        # print(th)
        dx = G[0] - P[0]
        dy = G[1] - P[1]
        # dth = G[2] - P[2]
        rho = math.sqrt(dx**2 + dy**2)
        gamma = ajustaangulo(math.atan2(dy,dx))
        alpha = ajustaangulo(gamma - P[2])
        # beta = ajustaangulo(G[2] - gamma)
        v = min(krho*rho,vmax)
        if abs(alpha) > math.pi/2:
            v = -v
            alpha = ajustaangulo(gamma + math.pi)
            # beta = ajustaangulo(beta + math.pi)
        dif_alpha = alpha - alpha_old
        alpha_old = alpha
        int_alpha = int_alpha + alpha
        # v = krho * rho
        # v = min(krho*rho,vmax)
        # w = kalpha*alpha + kbeta * beta
        w = kp*alpha + ki*int_alpha + kd*dif_alpha
        w = np.sign(w)* min(abs(w),wmax)

        
        if v > 0 and w > 0:
            env_vLvW = "FE"
            if abs(v) < 10:
                env_vLvW = env_vLvW + "0"
        if v > 0 and w < 0:
            env_vLvW = "FD"
            if abs(v) < 10:
                env_vLvW = env_vLvW + "0"
        if v < 0 and w > 0:
            env_vLvW = "RE"
            if abs(v) < 10:
                v = abs(v)
                env_vLvW = env_vLvW + "0"
        if v < 0 and w < 0:
            env_vLvW = "RD"
            if abs(v) < 10:
                v = abs(v)
                env_vLvW = env_vLvW + "0"
                
        if rho < 50:
            v = 0
        env_vLvW = env_vLvW + str("{:.2f}".format(abs(v)))
        env_vLvW = env_vLvW + str("{:.2f}".format(abs(w)))
        print("rho: ",rho)
        print("Vel: ",v)
        print("th: ",w)
        print("dado enviado: ",env_vLvW)


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
        # P = P + dPdt.flatten() * dt
        P[2] = ajustaangulo(P[2])
        hud(P)
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
        pygame.draw.circle(screen, "red", (screen_dim[0]/2, screen_dim[1]/2),5)
        pygame.draw.circle(screen, "white", (G[0],G[1]),5)
        pygame.draw.polygon(screen, "cyan", RoboC[:2, :].T)
        pygame.draw.polygon(screen, "blue", RoboE[:2, :].T)
        pygame.draw.polygon(screen, "blue", RoboD[:2, :].T)
        pygame.draw.circle(screen, "black", (P[0], P[1]),5)    
        #print(distances)
        pygame.display.flip()

        dt = clock.tick(60)/100

    else:
        dt = clock.tick(60)/100

    #end controle
    
    #serial
    # cmd = input('Digite "l" para ligar e "d" para desligar. ') #Pergunta ao usuário se ele deseja ligar ou desligar o led

    # if cmd == 'l': #Se a resposta for "l", ele envia este comando ao Arduino
        # arduino.write(b'300000')
    # print("rodando...")
    arduino.write(str(env_vLvW).encode('utf-8'))
    # arduino.write(v)
    arduino.write(b'\n')
    # print(arduino.read())
    arduino.flush() #Limpa a comunicação
    # time.sleep(1)
    # print("parado")
    # arduino.write(b'0000\n')
    # time.sleep(2)
    # elif cmd == 'd': #Senão, envia o "d"
        # arduino.write('d'.encode())

    arduino.flush() #Limpa a comunicação

    #end serial
