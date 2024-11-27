/******************************************************************************
         Comunicação Unidirecional com mestre enviando para dois escravos
                              Sketch Escravo 1
                   Criado em 27 de Fevereiro de 2023
                por Michel Galvão (https://micsg.com.br)

  Eletrogate - Loja de Arduino \\ Robótica \\ Automação \\ Apostilas \\ Kits
                            https://www.eletrogate.com/
******************************************************************************/

// Inclusão de bibliotecas
#include <esp_now.h>            // Biblioteca para utilizar o protocolo de comunicação ESP-NOW
#include <WiFi.h>               // Biblioteca para conectar em redes Wi-Fi


// Variaveis do modelo cinematico
#define R 0.17 //raio da roda em cm
#define L 4.25 // centro do eixo em cm

//prototipo das funcoes
float cinematicaInversaMotorR(float v, float w);
float cinematicaInversaMotorL(float v, float w);
void moveRobo();


// Estrutura de dados para troca de informações
struct DataStruct {
  float velL;
  float velW;
};


// Variável para armazenar os dados recebidos
DataStruct dataStruct;


// Função que é chamada quando dados são recebidos via ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&dataStruct, incomingData, sizeof(dataStruct));  // Copia os dados recebidos para a estrutura de dados
  // temporizadorUltimaRecepcao = millis();                 // Atualiza a variável de tempo da última recepção
  /*
  // Liga o LED verde e desliga o LED vermelho
  digitalWrite(pinledVerde, HIGH);
  digitalWrite(pinledVermelho, LOW);
  */
  // Atualiza o display de LCD com informações sobre os dados recebidos
  // Serial.print("Obtido ");
  // Serial.print(len);
  // Serial.print(" Bytes ");
  // Serial.print("value: ");

  // // Se o valor inteiro for menor que 10, adicionar um zero na frente
  // if (dataStruct.velL < 10) {
  //   Serial.print("0");
  // }

  // Imprimir o valor inteiro na tela
  // Serial.print(dataStruct.velL);

  // Limpar a tela para garantir que não haja caracteres antigos
  // Serial.print("       ");
}

void setup() {

  Serial.begin(9600);
  
  pinMode(27, OUTPUT);//Definimos o pino 2 como saída.
  pinMode(26, OUTPUT);//Definimos o pino 2 como saída.
  pinMode(13, OUTPUT);//Definimos o pino 2 como saída.
  pinMode(32, OUTPUT);//Definimos o pino 2 como saída.

  ledcAttachPin(27, 1);//Atribuimos o pino 2 ao canal 0.
  ledcAttachPin(26, 0);//Atribuimos o pino 2 ao canal 1.
  ledcAttachPin(13, 3);//Atribuimos o pino 2 ao canal 2.
  ledcAttachPin(32, 2);//Atribuimos o pino 2 ao canal 3.
  
  ledcSetup(0, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcSetup(1, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcSetup(2, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcSetup(3, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.

  // Mensagem de inicialização no display de LCD
  // lcd.setCursor(0, 0);
  Serial.print("   Comunicacao  ");
  // lcd.setCursor(0, 1);
  Serial.print("     ESPNOW     ");
  // delay(1200);
  // lcd.clear();

  // Mensagem do modo de configuração mestre/escravo no display de LCD
  // lcd.setCursor(0, 0);
  Serial.print("Mestre /");
  // lcd.setCursor(0, 1);
  Serial.print("2 escravos");
  // delay(1000);
  // lcd.clear();

  // Mensagem informando que este dispositivo é escravo
  // lcd.setCursor(0, 0);
  Serial.print("Este dispositivo");
  // lcd.setCursor(0, 1);
  Serial.print("eh Escravo 1");
  // delay(1000);
  // lcd.clear();

  // Desconecta de alguma conexão WiFi anterior e define o modo estação (STA)
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);

  // Inicia a biblioteca ESP-NOW e, caso ocorra algum erro, reinicia o dispositivo
  if (esp_now_init() != ESP_OK) {
    // lcd.setCursor(0, 0);
    Serial.print("Inicializacao   ");
    // lcd.setCursor(0, 1);
    Serial.print("ESP-NOW com Erro");
    // digitalWrite(pinledVermelho, HIGH);
    delay(2000);
    ESP.restart();
  }

  // Registra a função OnDataRecv como a função a ser chamada quando receber dados via ESP-NOW
  esp_now_register_recv_cb(OnDataRecv);

  // Define o tempo atual como o temporizador da última recepção de dados
  // temporizadorUltimaRecepcao = millis();
}

void loop() {  // Função loop() que executa repetidamente enquanto a placa estiver ligada

  


  // Verifica se o tempo desde a última recepção é maior que 2 segundos
  // if (millis() - temporizadorUltimaRecepcao > 2000) {
    // digitalWrite(pinledVermelho, HIGH);  // Liga o LED vermelho indicando que a recepção ultrapassou o tempo limite
    // digitalWrite(pinledVerde, LOW);      // Desliga o LED verde
    // lcd.clear();                         // Limpa o display de LCD    
    // ledcWrite(0, cinematicaInversaMotorR(dataStruct.velL, dataStruct.velW)*7.1);//Escrevemos no canal 0, o duty cycle "i".
    // ledcWrite(1, 0);//Escrevemos no canal 0, o duty cycle "0".
    // ledcWrite(2, cinematicaInversaMotorL(dataStruct.velL, dataStruct.velW)*7.1);//Escrevemos no canal 0, o duty cycle "i".
    // ledcWrite(3, 0);//Escrevemos no canal 0, o duty cycle "0".

    moveRobo(dataStruct.velL, dataStruct.velW);

    // Mostra mensagem de tempo limite ultrapassado no display de LCD
    // lcd.setCursor(0, 0);
    // Serial.print(" quad: ");
    // Serial.println(dataStruct.quad);

    Serial.print("Vel L: ");
    Serial.print(dataStruct.velL);
    Serial.print(" Vel W: ");
    Serial.println(dataStruct.velW);

    // lcd.setCursor(0, 1);
    // Serial.print("recepcao ultrap.");
    delay(500);  // Aguarda 1 segundo antes de continuar a execução
  // }
 

}

void moveRobo(float v, float w){

  if(v>0){
    
    Serial.print("PwmR: ");
    Serial.print(cinematicaInversaMotorR(v, w));
    Serial.print("PwmL: ");
    Serial.println(cinematicaInversaMotorL(v, w));

    ledcWrite(0, cinematicaInversaMotorR(v,w));//Escrevemos no canal 0, o duty cycle "i".
    ledcWrite(1, 0);//Escrevemos no canal 0, o duty cycle "0".
    ledcWrite(2, cinematicaInversaMotorL(v, w));//Escrevemos no canal 0, o duty cycle "i".
    ledcWrite(3, 0);//Escrevemos no canal 0, o duty cycle "0".
  } else {
    Serial.print("PwmR: -");
    Serial.print(cinematicaInversaMotorR(v, w));
    Serial.print("PwmL: -");
    Serial.println(cinematicaInversaMotorL(v, w));
    
    ledcWrite(0, 0);//Escrevemos no canal 0, o duty cycle "i".
    ledcWrite(1, cinematicaInversaMotorR(v, w));//Escrevemos no canal 0, o duty cycle "0".
    ledcWrite(2, 0);//Escrevemos no canal 0, o duty cycle "i".
    ledcWrite(3, cinematicaInversaMotorL(v, w));//Escrevemos no canal 0, o duty cycle "0".
  }

}


float cinematicaInversaMotorR(float v, float w){
  /*if(v>30)
    v=30;
  return abs((v + w*L) / R);
  */
  float vr = abs((v + w*L) / R);

  if(abs(v)<10)
    return 0;
  return vr;
  
  // return min(vr,200.0f);*/
}

float cinematicaInversaMotorL(float v, float w){
  /*
  if(v>30)
    v=30;
  return abs((v - w*L) / R);
  
  */
  // wl = (v - w*l) / r
  float vl = abs((v - w*L) / R);

  if(abs(v)<10)
    return 0;
  
  return vl;
  // return min(vl,200.0f);*/

}