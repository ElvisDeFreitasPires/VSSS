/******************************************************************************
         Comunicação Unidirecional com mestre enviando para dois escravos
                              Sketch Mestre
                   Criado em 28 de Fevereiro de 2023
                por Michel Galvão (https://micsg.com.br)

  Eletrogate - Loja de Arduino \\ Robótica \\ Automação \\ Apostilas \\ Kits
                            https://www.eletrogate.com/

                            Alterado por Elvis de Freitas Pires
                            Envia dado para o motor
******************************************************************************/

#include <esp_now.h>            // Inclui a biblioteca esp_now para o uso do protocolo de comunicação ESP-NOW
#include <WiFi.h>               // Inclui a biblioteca WiFi para configuração da rede sem fio


String fraseRecebida = ""; // a String to hold incoming data
bool fraseCompleta = false;  // whether the string is complete


uint8_t slave1MacAddress[] = { 0xe8, 0xdb, 0x84, 0x14, 0xf4, 0xf0 };  // Define o endereço MAC do dispositivo escravo 1. Coloque o endereço MAC de sua placa 1 aqui
uint8_t slave2MacAddress[] = { 0xd0, 0xef, 0x76, 0x33, 0x5a, 0xe4 };  // Define o endereço MAC do dispositivo escravo 2. Coloque o endereço MAC de sua placa 2 aqui

struct DataStruct {  // Define a estrutura DataStruct para troca de informações
  // String quad;
  float velL; //velocidade linear enviada 
  float velW; //velocidade angular enviada
};

esp_now_peer_info_t peerInfo;  // Cria uma estrutura esp_now_peer_info_t, que é utilizada para registrar informações sobre um peer (dispositivo) que será adicionado à rede ESPNOW

DataStruct dataToSend;  // Declara a estrutura dataToSend



void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {  // Função chamada quando os dados são enviados
  if (memcmp(mac_addr, slave1MacAddress, 6) == 0) {
    if (status == ESP_NOW_SEND_SUCCESS) {  // Se o envio foi bem sucedido, ...
      Serial.println("   OK! slave1:");
      //lcd.print(formatarNumero(dataToSend.value));  // Escreve o valor no display
    } else {
      Serial.println("FALHA! slave1   ");
    }
  } else if (memcmp(mac_addr, slave2MacAddress, 6) == 0) {
    if (status == ESP_NOW_SEND_SUCCESS) {  // Se o envio foi bem sucedido, ...
      Serial.println("   OK! slave2:");
      //lcd.print(formatarNumero(dataToSend.value));  // Escreve o valor no display
    } else {
      Serial.println("FALHA! slave2 ");
    }
  }
}


void setup() {

  Serial.begin(9600);
  Serial2.begin(9600);
  fraseRecebida.reserve(200);

  
  WiFi.disconnect();    // Desconecta de qualquer rede WiFi previamente conectada
  WiFi.mode(WIFI_STA);  // Define o modo WiFi como Station (cliente)

  if (esp_now_init() != ESP_OK) {   // Inicializa o ESP-NOW e verifica se há erros
    Serial.println("Inicializacao ");    // Escreve a mensagem no display
    Serial.println("ESP-NOW com Erro");  // Escreve a mensagem no display
    delay(2500);                    // Espera por 2,5 segundos
    ESP.restart();                  // Reinicia o dispositivo
  }

  esp_now_register_send_cb(OnDataSent);  // Registra a função de callback que é chamada quando os dados são enviados

  peerInfo.channel = 0;                             // Define o canal de comunicação como 0 na estrutura peerInfo
  peerInfo.encrypt = false;                         // Define a encriptação como desativada na estrutura peerInfo
  memcpy(peerInfo.peer_addr, slave1MacAddress, 6);  // Copia o endereço MAC do escravo 1 para a estrutura peerInfo

  // Tenta adicionar o escravo 1 à lista de pares de comunicação ESP-NOW e verifica se foi bem sucedido
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // Caso não seja possível adicionar o escravo 1, exibe mensagem de falha no display e reinicia o dispositivo
    Serial.println("Falha ao adicionar peer1");
    delay(2500);
    ESP.restart();
  }

  memcpy(peerInfo.peer_addr, slave2MacAddress, 6);  // Copia o endereço MAC do escravo 2 para a estrutura peerInfo
  // Tenta adicionar o escravo 2 à lista de pares de comunicação ESP-NOW e verifica se foi bem sucedido
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // Caso não seja possível adicionar o escravo 2, exibe mensagem de falha no display e reinicia o dispositivo
    Serial.println("Falha ao adicionar peer2");
    delay(2500);
    ESP.restart();
  }

  // Preparando os dados a serem enviados
  // dataToSend.quad = "";
  dataToSend.velL = 0;
  dataToSend.velW = 0;
}
unsigned int dt = millis(); // varialvel para contar os segundos no while
void loop() {

  int sinal = 0;
  // print the string when a newline arrives:
  if (fraseCompleta) {

    String velL=""; //String que vai receber a parcela responsavel pela velocidade linear da frase
    String velW=""; //String que vai receber a parcela responsavel pela velocidade angular da frase
    // String dtMov=""; //tempo de duração do movimento
    /*
    if(fraseRecebida[0] == '-' && fraseRecebida[3] == '-'){
      velL+= fraseRecebida[1];velL+=fraseRecebida[2]; // retira da frase recibida a parcela da velL
      velW+=fraseRecebida[4];velW+=fraseRecebida[5];velW+=fraseRecebida[6];velW+=fraseRecebida[7];velW+=fraseRecebida[8];
      sinal=0;
    }
    else if(fraseRecebida[0] == '-'){
      velL+= fraseRecebida[1];velL+=fraseRecebida[2]; // retira da frase recibida a parcela da velL
      velW+=fraseRecebida[3];velW+=fraseRecebida[4];velW+=fraseRecebida[5];velW+=fraseRecebida[6];velW+=fraseRecebida[7];
      sinal=1;
    }
    else if(fraseRecebida[2] == '-'){
      velL+= fraseRecebida[0];velL+=fraseRecebida[1]; // retira da frase recibida a parcela da velL
      velW+=fraseRecebida[3];velW+=fraseRecebida[5];velW+=fraseRecebida[6];velW+=fraseRecebida[7];velW+=fraseRecebida[8]; // retira da frase recebida a parcela da velW
      sinal=2;
    }else{
      velL+= fraseRecebida[0];velL+=fraseRecebida[1]; // retira da frase recibida a parcela da velL
      velW+=fraseRecebida[2];velW+=fraseRecebida[3];velW+=fraseRecebida[4];velW+=fraseRecebida[5];velW+=fraseRecebida[6];
      sinal=3;
    }
    // dtMov+= fraseRecebida[4];dt+=fraseRecebida[5];  // retira da frase recibida a parcela que corresponde ao tempo de movimento
    */
    if(fraseRecebida.startsWith("FD")){ 
      // FD 09.97 0.10
      velL+= fraseRecebida[2];velL+=fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];
      velW+= fraseRecebida[7];velW+= fraseRecebida[8];velW+= fraseRecebida[9];velW+= fraseRecebida[10];
      // dataToSend.quad = "FD";
      dataToSend.velL = velL.toFloat();
      dataToSend.velW = velW.toFloat();
    }
    if(fraseRecebida.startsWith("FE")){
      // FE 12.89 -0.39
      // velL+= fraseRecebida[2];velL+=fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];
      // velW+= fraseRecebida[8];velW+= fraseRecebida[9];velW+= fraseRecebida[10];velW+= fraseRecebida[11];
      velL+= fraseRecebida[2];velL+=fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];
      velW+= fraseRecebida[7];velW+= fraseRecebida[8];velW+= fraseRecebida[9];velW+= fraseRecebida[10];
      // dataToSend.quad = "FE";
      dataToSend.velL = velL.toFloat();
      dataToSend.velW = -velW.toFloat();
    }
    if(fraseRecebida.startsWith("RE")){
      // RE -15.80 0.63
      // velL+= fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];velL+=fraseRecebida[7];
      // velW+= fraseRecebida[8];velW+= fraseRecebida[9];velW+= fraseRecebida[10];velW+= fraseRecebida[11];
      velL+= fraseRecebida[2];velL+=fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];
      velW+= fraseRecebida[7];velW+= fraseRecebida[8];velW+= fraseRecebida[9];velW+= fraseRecebida[10];
      // dataToSend.quad = "RE";
      dataToSend.velL = -velL.toFloat();
      dataToSend.velW = velW.toFloat();
    }
    if(fraseRecebida.startsWith("RD")){
      // RD -13.69 -0.34
      // velL+= fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];velL+=fraseRecebida[7];
      // velW+= fraseRecebida[9];velW+= fraseRecebida[10];velW+= fraseRecebida[11];velW+= fraseRecebida[12];
      velL+= fraseRecebida[2];velL+=fraseRecebida[3];velL+=fraseRecebida[4];velL+=fraseRecebida[5];velL+=fraseRecebida[6];
      velW+= fraseRecebida[7];velW+= fraseRecebida[8];velW+= fraseRecebida[9];velW+= fraseRecebida[10];
      // dataToSend.quad = "RD";
      dataToSend.velL = -velL.toFloat();
      dataToSend.velW = -velW.toFloat(); 
    }

    Serial2.println(fraseRecebida);
    Serial2.println(velL.toFloat());
    Serial2.println(velW.toFloat());
    // Serial.println(dtMov.toInt()*1000);
    /*
    switch (sinal) {
      case 0:
        dataToSend.velL = -velL.toFloat();
        dataToSend.velW = -velW.toFloat();
        break;
      case 1:
        dataToSend.velL = -velL.toFloat();
        dataToSend.velW = velW.toFloat();
        break;
      case 2:
        dataToSend.velL = velL.toFloat();
        dataToSend.velW = -velW.toFloat();
        break;
      case 3:
        dataToSend.velL = velL.toFloat();
        dataToSend.velW = velW.toFloat();
        break;
      default:
        dataToSend.velL = 0;
        dataToSend.velW = 0;
        break;
    }
    */
    // dataToSend.velL = fraseRecebida[0] == '-' ? -velL.toFloat() : velL.toFloat(); // atribui a variavel de envio
    // dataToSend.velW = fraseRecebida[2] == '-' ? -velW.toFloat() : velW.toFloat();
    // dataToSend.velL = velL.toFloat(); // atribui a variavel de envio
    // dataToSend.velW = velW.toFloat() ; // atribui a variavel de envio
    esp_now_send(0, (uint8_t *)&dataToSend, sizeof(DataStruct));  // Envie os dados para todos os escravos cadastrados. Isso é definido através do primeiro argumento com valor 0

    // esse trecho pode ser removido
    dt = millis(); // atualiza a variavel dt
    // while(millis() - dt < (dtMov.toInt()*1000)); // fica em loop até que se passe o tempo de movimento


    fraseRecebida = "";
    fraseCompleta = false;
  }

  while(Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    fraseRecebida += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      fraseCompleta = true;
    }
  }



  /*

  envio teste

  if(millis() - dt > 1000){
    dataToSend.velL -=1;
    dataToSend.velW =0;
    dt = millis();
  }

  esp_now_send(0, (uint8_t *)&dataToSend, sizeof(DataStruct));  // Envie os dados para todos os escravos cadastrados. Isso é definido através do primeiro argumento com valor 0
  
  Serial.print("VelL: ");
  Serial.println(dataToSend.velL);
  if(dataToSend.velL<=0){
    dataToSend.velL = 0;
    delay(2000);
    dataToSend.velL = 10;
  }
  */
}