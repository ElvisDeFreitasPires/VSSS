import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
import cv2


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




cap = cv2.VideoCapture('http://200.18.141.189:4747/video')
#cap = cv2.VideoCapture(0)
while(True):
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

"""


#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

#amarelo_baixo = np.array([20, 100, 20], np.uint8)
#amarelo_alto = np.array([40, 255, 255], np.uint8)

#vermelho_baixo1 = np.array([0, 100, 200], np.uint8)
#vermelho_baixo2 = np.array([175, 100, 20], np.uint8)
#vermelho_alto1 = np.array([8, 255, 255], np.uint8)
#vermelho_alto2 = np.array([179, 255, 255], np.uint8)

laranja_baixo = np.array([0, 143, 20], np.uint8)
laranja_alto = np.array([15, 255, 255], np.uint8)

def detecta_bola(frame):
    Bola_x = 0
    Bola_y = 0
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Ajustar a saturação e valor
    h, s, v = cv2.split(frameHSV)
    s = cv2.add(s, 30)
    v = cv2.add(v, -180)
    frameHSV = cv2.merge([h, s, v])
    
    mascaraLaranja=cv2.inRange(frameHSV,laranja_baixo,laranja_alto)
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

def main():
    rospy.init_node('detector_bola')
    pub_frame_bola = rospy.Publisher('/bola/frame_bola', Image, queue_size=10)
    pub_ponto = rospy.Publisher('/bola/posicao_bola', Point, queue_size=10)
    bridge = CvBridge()

    def image_callback(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            frame, bolax, bolay = detecta_bola(frame)
            
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub_frame_bola.publish(image_msg)

            point_msg = Point()
            point_msg.x = bolax
            point_msg.y = bolay
            point_msg.z = 0
            pub_ponto.publish(point_msg)

        except CvBridgeError as e:
            rospy.logerr(f'Erro ao converter a imagem: {e}')

    rospy.Subscriber('/camera_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
"""