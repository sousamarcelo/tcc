/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
//------------------------------------------------------------------------#Marcelo 05/03
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <WiFiUDP.h>
#include <NTPClient.h>

// Replace the next variables with your SSID/Password combination 
const char* ssid = "Tenda_DE5890"; // Tenda_DE5890//SILAG
const char* password = "Gnusmas_22"; // Gnusmas_22//amid1984


// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "137.184.132.170";//"192.168.2.102"; 
const char* mqttUser = "Garotas"; //----------------------------->> Marcelo 05/03 usuario mqtt
const char* mqttPassword = "teste@12"; //----------------------------->> Marcelo 05/03 usuario mqtt

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;



/* base
// Definindo pinagens // ----------------------------------------->> PINAGEM #Marcelo 04/03/2022
const int led_Pin = 19;
const int artificial_light_relay_pin = 2; // led simula luz artificial 21/03
const int air_humidifier_relay_pin = 4;
const int water_pump_relay1_pin = 15;
const int peltier_cooling_pin = 18; 
const int soil_moisture_sensor_pin = 35;
const int light_detection_sensor_pin = 34; //sensor de iluminação 21/03
const int maximun_water_level_sensor_pin = 33; //water_level_max_sensor_pin
const int minimum_water_level_sensor_pin = 32; //Sensor de nivel minimo
*/

//DEFINIÇÃO DE PINOS
#define led_Pin  19
#define pinRele1 4
#define pinRele2 16

//Defnições temperatura
#define temp_max 60    // Temperatura Maxima
#define temp_ideal 55  // Temperatura Ideal
#define temp_min 50    // Temperatura minima


//Instancia Objetos
Adafruit_BME280 bme; // I2C

//Declaração de variaveis
float temperature = 0.0;
float humidity = 0.0;

int   dry_soil = 50;

int estado = 0;

//controlando data e hora Marcelo 25/03
WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, 60000);//Cria um objeto "NTP" com as configurações.utilizada no Brasil //a.st1.ntp.br
struct tm data;//Cria a estrutura que contem as informacoes da data.
int hora;
char data_formatada[64];
int ATUALIZAR_DH;
String hora_ntp;

void setup() {
  Serial.begin(115200);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  //bme.begin(0x76);

  if (!bme.begin(0x76)) {
    Serial.println("Não foi possível encontrar um sensor BME280 válido, verifique a fiação!");
    while (1);
  }
  
  setup_wifi(); //comentando para testar se a conexão---------------------------------------------------------------------------------
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(led_Pin, OUTPUT);
  /*
  //Definindo pinagens
  
  pinMode(artificial_light_relay_pin, OUTPUT); // liga iluminação
  pinMode(soil_moisture_sensor_pin,INPUT);
  pinMode(light_detection_sensor_pin,INPUT); //Sensor de rediação/iluminação
  pinMode(water_pump_relay1_pin, OUTPUT);
  pinMode(maximun_water_level_sensor_pin, INPUT);
  pinMode(minimum_water_level_sensor_pin, INPUT); 
  pinMode(air_humidifier_relay_pin, OUTPUT);
  */

  /*
  //setando valor inicial do rele da bomba d'agua
  digitalWrite(water_pump_relay1_pin, LOW); 
  digitalWrite(air_humidifier_relay_pin, LOW); 
  */

  pinMode(pinRele1, OUTPUT);
  pinMode(pinRele2, OUTPUT);
  
  
  //controlando data e hora Marcelo 25/03
  ntp.begin();                // Inicia o protocolo
  ntp.forceUpdate();        // Atualização .          // Variável que armazena

  relogio_ntp(0);
  
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Conectando à ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }  

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço de IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Mensagem chegou no tópico: ");
  Serial.print(topic);
  Serial.print(". Messagem: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Alterando a saída para ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(led_Pin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(led_Pin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) { //----------------------------------------------->> nova funcionalidade, passando credenciais MQTT do usuario //Marcelo
      Serial.println("conectado");
      // Subscribe
      client.subscribe("esp32/output");  //----------------------->> Para se inscrever em topicos MQTT
    } else {
      Serial.print("tentativa falhou!!!, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*
// calculando umidade do Solo // --------------------------------->> #Marcelo 04/03/2023
void calculateSoilMoisture(){
  float sensorReading = 0;
  for (int i; i < 10; i++) {
    sensorReading += analogRead(soil_moisture_sensor_pin);
  }
  sensorReading /= 10.0;
  soil_moisture = map(sensorReading, 360, 4095, 100, 0);

  //Serial.println();
}
*/

/*
//Controlando nivel de agua do Reservatório
void reservoir_water_control(){
  minimun_water_level_sensor_status = digitalRead(minimum_water_level_sensor_pin);
  maximum_water_level_sensor_status = digitalRead(maximun_water_level_sensor_pin);

  if (minimun_water_level_sensor_status == 0.0){
    //Serial.println("Nival critico de reservatório de agua --> acionar solenoide <--");
    if(maximum_water_level_sensor_status == 1.0){
      //Serial.println("Nivel maximo do reservatório de agua atingido --> desligar solenoide <--");
    }
  } else { // só por segurança o comando para desligar o solenoide nesse ponto
    //Serial.println(" --> solenoide delisgada <--");
  }
}
*/

/*
void lighting_control(){
  artificial_lighting = map(analogRead(light_detection_sensor_pin), 0, 4095, 100, 0);
  //float square_ratio = artificial_lighting / 4095;
  //float square_ratio = pow(artificial_lighting, 2.0);
  //analogWrite(artificial_light_pin, !(square_ratio*255));
  if(artificial_lighting < 50.0){ //artificial_lighting < 20.0 ou relogio_ntp(3) >= "800"[utiliza o valor em horas para somar]
     digitalWrite(artificial_light_relay_pin, HIGH); //!artificial_lighting
  } else
    {digitalWrite(artificial_light_relay_pin, LOW);
  }
}  
*/

//controlando data e hora Marcelo 25/03
String relogio_ntp(int retorno) {
  //Esta condição será chamada uma unica fez para atualizar a data e hora com NTP
  if (retorno == 0 || ATUALIZAR_DH == 0)
  {
    Serial.print(" Atualizando data e hora...");
    hora = ntp.getEpochTime(); //Atualizar data e hora usando NTP online
    Serial.print(" NTP Unix: ");
    Serial.println(hora);
    timeval tv;//Cria a estrutura temporaria para funcao abaixo.
    tv.tv_sec = hora;//Atribui minha data atual. Voce pode usar o NTP para isso ou o site citado no artigo!
    settimeofday(&tv, NULL);//Configura o RTC para manter a data atribuida atualizada.
    time_t tt = time(NULL);//Obtem o tempo atual em segundos. Utilize isso sempre que precisar obter o tempo atual
    data = *gmtime(&tt);//Converte o tempo atual e atribui na estrutura
    strftime(data_formatada, 64, "%d/%m/%Y %H:%M:%S", &data);//Cria uma String formatada da estrutura "data"
    Serial.print(" Data e hora atualizada:");
    Serial.println(data_formatada);
    ATUALIZAR_DH = 1;
  }

  if (retorno == 1)
  {
    time_t tt = time(NULL);//Obtem o tempo atual em segundos. Utilize isso sempre que precisar obter o tempo atual
    data = *gmtime(&tt);//Converte o tempo atual e atribui na estrutura
    strftime(data_formatada, 64, "%d/%m/%Y %H:%M:%S", &data);//Cria uma String formatada da estrutura "data"
    hora_ntp   = data_formatada;
  }
  if (retorno == 2)
  {
    time_t tt = time(NULL);//Obtem o tempo atual em segundos. Utilize isso sempre que precisar obter o tempo atual
    data = *gmtime(&tt);//Converte o tempo atual e atribui na estrutura
    strftime(data_formatada, 64, "%d/%m/%Y", &data);//Cria uma String formatada da estrutura "data"
    hora_ntp = data_formatada;
  }
  if (retorno == 3)
  {
    time_t tt = time(NULL);//Obtem o tempo atual em segundos. Utilize isso sempre que precisar obter o tempo atual
    data = *gmtime(&tt);//Converte o tempo atual e atribui na estrutura
    strftime(data_formatada, 64, "%H%M", &data);//Cria uma String formatada da estrutura "data"
    hora_ntp = data_formatada;

  }
  return hora_ntp;

}
/*
float air_humidity_control(float humidity_Aux){
  if(humidity_Aux <= 65.0){ 
    digitalWrite(air_humidifier_relay_pin, LOW);    
  } else {
    digitalWrite(air_humidifier_relay_pin, HIGH);
  }
  return humidity_Aux;
  
}
*/

void loop() { 
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
  

  // Temperature in Celsius
 temperature = bme.readTemperature(); //------------------------------------------------------------------->> desativado temporarioamente por que o sensor está com erro de leitura 25/03
  // Umidade do ar
 humidity = bme.readHumidity();  //------------------------------------------------------------------->> desativado temporarioamente por que o sensor está com erro de leitura 25/03
  

  
  
  long now = millis();
  if (now - lastMsg > 10000) { //120000
    lastMsg = now;    
    
    /*
    // Convert the value to a char array //---------------------------------------->> conversão necessaria para publicação no topico
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature/value", tempString);

    
    
    // Convert the value to a char array //---------------------------------------->> conversão necessaria para publicação no topico
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity/value", humString);

    //Sensor de Umidade do solo, convertendo o valor em uma matriz de caracteres       
    char soilString[8];
    dtostrf(soil_moisture, 1, 2, soilString);
    Serial.print("soil Moisture: ");
    Serial.println(soilString);
    client.publish("esp32/soilMoisture/value", soilString);

    //Enviando Status do sensor de nivel maximo de agua, convertendo o valor em uma matriz de caracteres    
    char pumpRelay1StatusString[8];
    dtostrf(water_pump_relay1_status, 1, 2, pumpRelay1StatusString);
    Serial.print("water pump relay status: ");
    Serial.println(pumpRelay1StatusString);
    client.publish("esp32/waterPumpRelay1Status/value", pumpRelay1StatusString);
    
    //Enviando statos sensor de nivel maximo d'agua, convertendo o valor em uma matriz de caracteres    //atenção, avaliar real necessidade de enviar esse status do nivel do reservatório, talvez só enviar a mensagem de nivel maximo e minimo.
    char maxlevelStatusString[8];
    dtostrf(maximum_water_level_sensor_status, 1, 2, maxlevelStatusString);
    Serial.print("MaximunWaterLevelSensorStatus: ");
    Serial.println(maxlevelStatusString);
    client.publish("esp32/MaximunWaterLevelSensorStatus/value", maxlevelStatusString);

    //Enviando statos sensor de nivel maximo d'agua, convertendo o valor em uma matriz de caracteres    //atenção, avaliar real necessidade de enviar esse status do nivel do reservatório, talvez só enviar a mensagem de nivel maximo e minimo.
    char minlevelStatusString[8];
    dtostrf(minimun_water_level_sensor_status, 1, 2, minlevelStatusString);
    Serial.print("minimun_water_level_sensor_status: ");
    Serial.println(minlevelStatusString);
    client.publish("esp32/MaximunWaterLevelSensorStatus/value", minlevelStatusString);
    

    Serial.println();
     //monitorando nivel maximo do reservatório para serial
    if(maximum_water_level_sensor_status == 0) {
      Serial.println("Nivel Alto do reservatório --> SIM <--");
    } else {
      Serial.println("Nivel Alto do reservatório --> NÃO <--");
    }  
    
    if(minimun_water_level_sensor_status == 0) {
      Serial.println("Nivel Critico do reservató --> SIM <--");
    }  else {
      Serial.println("Nivel Critico do reservató --> NÃO <--");
    }

    if(water_pump_relay1_status == 1.00) {
      Serial.println("Bomba de Irrigação         --> SIM <--");
    } else {
      Serial.println("Bomba de Irrigação         --> NÃO <--");
    }

    Serial.println();    

    Serial.print("Quantidade de Radiação: ");
    Serial.println(artificial_lighting);

    */
    
    //Serial.println(relogio_ntp(0)); 

    Serial.println(relogio_ntp(1));
    //delay(1000);
    Serial.println(relogio_ntp(2));
    //delay(1000);    
    Serial.println(relogio_ntp(3));
    
    Serial.println();
    //delay(1000);        
  }    
}