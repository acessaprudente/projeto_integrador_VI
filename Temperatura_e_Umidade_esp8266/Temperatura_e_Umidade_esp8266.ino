

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL2ZRsa4oW_"
#define BLYNK_TEMPLATE_NAME "Sistema de Irrigação IOT"
#define BLYNK_AUTH_TOKEN "HhyFEntZ4Oq6cFjzJpHKX6qz8xzs0csI"

#include <Wire.h>                //Biblioteca para comunicação I2C
#include <SFE_BMP180.h>          //Biblioteca para sensor BMP180
#include <WiFiClient.h>          //inserido em 11/05/2024
#include <ESP8266WiFi.h>         //Biblioteca para conectar o módulo ESP8266 a uma rede Wi-Fi
#include <BlynkSimpleEsp8266.h>  //Biblioteca para uso do ESP8266 na plataforma Blynk
#include <SoftwareSerial.h>      //alterado em 12/05/2024
#include <DHT.h>                 //Inclui a biblioteca DHT Sensor Library

#define ALTITUDE 475       //Inclui a biblioteca DHT Sensor Library
#define DHTPIN 0           //Pino digital GPIO0 (pino D3) conectado ao DHT11
#define DHTTYPE DHT11      //DHT 11
DHT dht(DHTPIN, DHTTYPE);  //Inicializando o objeto dht do tipo DHT passando como parâmetro o pino (DHTPIN) e o tipo do sensor (DHTTYPE)
float u = 0.0;             //Variável responsável por armazenar a umidade lida pelo DHT11
float t = 0.0;             //Variável responsável por armazenar a temperatura lida pelo DHT11
#define sensorUmidade D5   //Atribui o pino digital D5 à variável sensorPinD
// Configurações do aplicativo e da rede Wi-Fi
char auth[] = BLYNK_AUTH_TOKEN;  //Armazena o AuthToken no array auth
char ssid[] = "PEPO";            //Rede WiFi
char pass[] = "PEPO2001";        //Senha da rede WiFi

// PRESSAO ABSOLUTA E RELATIVA
double status;            //Variável auxiliar para verificação do resultado
double temperatura;       //variável para armazenar o valor da temperatura
double pressao_abs;       //variável para armazenar o valor da pressão absoluta
double pressao_relativa;  //variável para armazenar a pressão relativa

SFE_BMP180 sensorP;

int PinoAnalogico;  // Define o pino A0 como Pino Analógico do sensor
int PinoDigital;    // Define pino D2 como Pino Digital do Sensor

int Rele = D0;  // Pino Digital D1 como Relé
int EstadoSensor = 0;
int UltimoEstSensor = 0;
int ValAnalogIn;  // Valor analógico no código





// Irrigação
void statusPlanta() {
  PinoDigital = D5;
  PinoAnalogico = A0;
  ValAnalogIn = analogRead(PinoAnalogico);
  Serial.println(analogRead(PinoAnalogico));
  int Porcento = map(ValAnalogIn, 1023, 0, 0, 100);  // Traforma o valor analógico em porcentagem
  Serial.print("Umidade: ");                         // Imprime o símbolo no valor
  Serial.print(Porcento);                            // Imprime o valor em Porcentagem no monitor Serial
  Serial.println("%");
  if (Porcento <= 0) {                   // Se a porcentagem for menor ou igual à 76%. OBS: Você pode alterar essa porcentagem
    Serial.println("Irrigando Planta");  // Imprime no monitor serial
    digitalWrite(Rele, LOW);             // Aciona Relé
  } else {                               // Caso contrario
    Serial.println("Planta Irrigada");   // Imprime a no monitor serial
    digitalWrite(Rele, HIGH);            // Desliga Relé
  }
}

// Umidade
void sensorDHT() {
  u = dht.readHumidity();     //Realiza a leitura da umidade
  t = dht.readTemperature();  //Realiza a leitura da temperatura
  Serial.print("Umidade do Ar: ");
  Serial.println(u);  //Imprime na serial a umidade
  Serial.print("Temperatura do Ar: ");
  Serial.println(t);          //Imprime na serial a temperatura
  Blynk.virtualWrite(V0, t);  //Escreve no pino virtual V0 a temperatura em graus Celsius
  Blynk.virtualWrite(V1, u);  //Escreve no pino virtual V1 a umidade em porcentagem
}

// Pressao Abs/Rel

void Pressao() {

  status = sensorP.startTemperature();             //Inicializa a leitura da temperatura
  if (status != 0) {                               //Se status for diferente de zero (sem erro de leitura)
    delay(status);                                 //Realiza uma pequena pausa para que a leitura seja finalizada
    status = sensorP.getTemperature(temperatura);  //Armazena o valor da temperatura na variável temperatura
    if (status != 0) {                             //se status for diferente de zero (sem erro de leitura)
      //Leitura da Pressão Absoluta
      status = sensorP.startPressure(3);                         //Inicializa a leitura
      if (status != 0) {                                         //se status for diferente de zero (sem erro de leitura)
        delay(status);                                           //Realiza uma pequena pausa para que a leitura seja finalizada
        status = sensorP.getPressure(pressao_abs, temperatura);  //Atribui o valor medido de pressão à variável pressao, em função da variável temperatura
        Serial.print("Pressão absoluta: ");
        Serial.println(pressao_abs, 1);                                //Imprime na serial a pressão absoluta
        if (status != 0) {                                             //se status for diferente de zero (sem erro de leitura)
          pressao_relativa = sensorP.sealevel(pressao_abs, ALTITUDE);  //Atribui o valor medido de pressão relativa à variavel pressao_relativa, em função da ALTITUDE
          Serial.print("Pressão relativa: ");
          Serial.println(pressao_relativa, 1);  //Imprime na serial a pressão relativa
        }
      }
    }
  }
  Blynk.virtualWrite(V2, pressao_abs);       //Escreve no pino virtual V2 a pressão absoluta em
  Blynk.virtualWrite(V3, pressao_relativa);  //Escreve no pino virtual V3 a pressão relativa ao nível do mar em hpa
}

void detectarUmidade() {
  if (digitalRead(sensorUmidade)) {  //Se sensorPinD for igual a 1
    Blynk.virtualWrite(V4, LOW);     //Desliga o LED de indicação no Blynk
    Serial.println("Irrigando PLanta");
  } else {                         //Senão
    Blynk.virtualWrite(V4, HIGH);  //Liga o LED de indicação no Blynk
    Serial.println("Planta Irrigada");
  }
}


void setup() {
  Serial.begin(9600);                       //alterado de 9600 para 1115200 em 11/05/2024
  Serial.println("Projeto Integrador VI");  // Imprime a frase no monitor serial
  Blynk.begin(auth, ssid, pass);
  pinMode(Rele, OUTPUT);  // Declara o Rele como Saída Digital
  pinMode(PinoDigital, INPUT);
c:
  \Users\user\Pictures\codigo_atualizado.txt
    pinMode(sensorUmidade, INPUT);  //Configura sensorChuva como entrada
  dht.begin();                      //Inicializa o sensor DHT11
  sensorP.begin();
}



void loop() {
  Blynk.run();        //Chama a função Blynk.run
  statusPlanta();     // chama a função statusPlant
  sensorDHT();        //Chama a função sensorDHT
  Pressao();          //Chama a função Pressao
  detectarUmidade();  //Chama a função detectarChuva
  delay(1000);
}
