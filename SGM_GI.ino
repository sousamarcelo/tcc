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
#include <Ultrasonic.h> //Inclui a biblioteca do ultrassônico



// Replace the next variables with your SSID/Password combination 
const char* ssid = "SILAG"; // Tenda_DE5890//SILAG
const char* password = "amid1984"; // Gnusmas_22//amid1984


// Add your MQTT Broker IP address, example://const char* mqtt_server = "192.168.1.144";
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
#define led_Pin                   12 // veio do padrao para testar o fluxo de app para o esp
#define RELAY_1_PIN                4 // temp min (aquecedor)
#define RELAY_2_PIN               16 // temp max (peutier)
#define RELAY_3_PIN                2 // umidificador mini
#define RELAY_4_PIN               17 // Iluminação min
#define RELAY_5_PIN               15 // bomba de irrigação
#define RELAY_6_PIN               19 // SOLENOIDE porta atualizada //Marcelo 26/05
#define RELAY_7_PIN               23 // Fan Auxiliar resfriamento
#define RADIATION_SENSOR_PIN      34 // Sensor de Radiação
#define SOIL_MOISTURE_SENSOR_PIN  35 // sensor de umidade do solo
#define SOIL_MOISTURE_SENSOR2_PIN 14 // sensor de umidade do solo 2 (em dev)
#define SOIL_MOISTURE_SENSOR3_PIN 27 // sensor de umidade do solo 3 (em dev)
#define RES_LOW_SENSOR_PIN        32 // reservatório nivel baixo
#define RES_HIGH_SENSOR_PIN       33 // reservatório nivel alto
#define trigPin                   25 // trigger sensor ultrasonic
#define echoPin                   26 // echo    sensor ultrasonic




//DEFINIÇÕES TEMPERATURA
#define TEMP_MAX   28    // Temperatura Maxima
#define TEMP_IDEAL 25    // Temperatura Ideal
#define TEMP_MIN   22    // Temperatura minima

//DEFINIÇÕES UMIDADE DO AR
#define UR_MAX   75   //70
#define UR_IDEAL 65   //65
#define UR_MIN   60   //60

//DEFINIÇÕES REDIAÇÃO
#define RA_MAX   100   //
#define RA_IDEAL  50   //
#define RA_MIN    35   //60

//DEFINIÇÕES UMIDADE DO SOLO
#define SM_MAX   100.0   //
#define SM_IDEAL  70.0  // umidade do solo ideal
#define SM_MIN    60.0   // umidade do solo minimo

//DEFINIÇÕES NIVEL DO RESERVATÓRIO DE AGUA
#define RE_MAX 1
#define RE_MIN 0

#define READING_INTERVAL_CORE 1000 // invelado de execução das leituras e rotinas
#define READING_INTERVAL_INTERNET 1000 // invelado de execução das leituras e rotinas

// DEFINIÇÕES DISTANCIA SENSOR ULTRASONIC
Ultrasonic ultrassom(trigPin, echoPin); // instanciando objeto HCSR04 (trig pin, echo pin)
#define UDISTANCE_MAX   5 // nivel maximo do reservatório // 12 nos teste de casa
#define UDISTANCE_MIN   3 // nivel minimos do reservatório


//Instancia Objetos
Adafruit_BME280 bme; // I2C

//Declaração de variaveis
float temperature =  0.0;
float humidity =     0.0;
float radiation =    0.0;
float soilMoisture = 0.0;
int   reservoirLow =   0;
int   reservoirHigh =  0;
long  distance; //SENSOR UNTRASONIC: cria a variável distancia do tipo long

//int   dry_soil = 50;

int stateTemperature =     0; // 0 = temperatura ideal | -1 = peutier aquecimento ligado | 1 = peutier resfriamento Ligado
int stateHumidity =        0; // 0 = Ar ideal | -1 = Umidificador ligado | 1 = desumidificador Ligado(não utilizado)
int stateRadiation =       0; // 0 = radiação ideal | -1 = iluminação ligado | 1 = (não utilizado) // iluminação por sensor só para testes, depois será por horario agendado
int statusSoilMoisture =   0; // 0 = umidade do solo ideal | -1 = irrigação ligado | 1 = irrigação desligada
int statusReservoirLevel = 0; // 0 = nivel do reservatório ideal | -1 = nivel baixo do reservatório  | 1 = nivel alto do reservatório
int stateLight =           0;
int stateUltrasonic =      0; // distancia sensor ultrasoic: -1 nivel baixo | 1 nivel maximo

// VARIAVEIS PARA CONTROLES DIVERSOS
unsigned long readControl; // CONTROLA TEMPO
int count_read_peltier = 0; // Controle de tempo de acionamento Fan x Peltier


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
  
  pinMode(led_Pin, OUTPUT);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
  pinMode(RELAY_4_PIN, OUTPUT);
  pinMode(RELAY_5_PIN, OUTPUT);
  pinMode(RELAY_6_PIN, OUTPUT);  
  pinMode(RELAY_7_PIN, OUTPUT);
  pinMode(RADIATION_SENSOR_PIN,      INPUT);
  pinMode(SOIL_MOISTURE_SENSOR_PIN,  INPUT);
  pinMode(SOIL_MOISTURE_SENSOR2_PIN, INPUT);
  pinMode(RES_LOW_SENSOR_PIN,        INPUT);
  pinMode(RES_HIGH_SENSOR_PIN,       INPUT);

  
  

  digitalWrite(RELAY_1_PIN, LOW); // 
  digitalWrite(RELAY_2_PIN, LOW); // 
  digitalWrite(RELAY_3_PIN, LOW);
  digitalWrite(RELAY_4_PIN, LOW); // rele da alta
  digitalWrite(RELAY_5_PIN, LOW); // rele duplo (rele da alta)
  digitalWrite(RELAY_6_PIN, LOW); // rele duplo
  digitalWrite(RELAY_7_PIN, LOW);
  
  
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

void calculateSoilMoisture(){
  float sensorReading = 0.0;
  for (int i = 0; i < 10; i++) {
    sensorReading += analogRead(SOIL_MOISTURE_SENSOR_PIN);
    //sensorReading += analogRead(SOIL_MOISTURE_SENSOR2_PIN); // Marcelo 20/05/23
  }
  
  sensorReading = sensorReading/10;
  soilMoisture = map(sensorReading, 460, 4095, 100, 0);  //map(sensorReading, 1600, 4095, 100, 0);//sensorReading, 360, 4095, 100, 0
}

void loop() { 
    
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
  

  //Controle de leitura dos sensores 

  
  if(millis() - readControl > READING_INTERVAL_CORE){   

      calculateSoilMoisture();    
      
      temperature   = bme.readTemperature(); // Temperature in Celsius
      
      humidity      = bme.readHumidity(); // Umidade do ar

      radiation     = map(analogRead(RADIATION_SENSOR_PIN), 1840, 4095, 100, 0);  // SENSOR LONGO (MOLHADO = 1840.00, SECO = 4095)  

      //reservoirLow  =  digitalRead(RES_LOW_SENSOR_PIN);
      
      //reservoirHigh =  digitalRead(RES_HIGH_SENSOR_PIN);       

      distance      = ultrassom.read(CM);//ultrassom.Ranging(CM) retorna a distancia em centímetros(CM)  //4 --> chumba teste//   
      
      //Serial.println("Sensores lidos com sucesso"); debug
    
    
          

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DA TEMPERATURA <<---
    //ATENÇÃO! RELE DUPLOS COM ACIONAMENTO INVETIDOS PARA CONTROLE DE TEMPERATURA
    switch (stateTemperature) {
      case 0:                                     //Leitura Anterior = Indicando temperatura ideal | peltier resfriamento e peltier aquecimento desligados
        if (temperature < TEMP_MIN) {             //Se Leitura Atual = Indicando tempetatura baixa
          stateTemperature = -1;
          digitalWrite(RELAY_1_PIN, HIGH);  //      
        } else if (temperature > TEMP_MAX) {      ////Se Leitura Atual = Indicando tempetatura quente          
          digitalWrite(RELAY_7_PIN, HIGH);
          stateTemperature = 0;
          count_read_peltier++;
          //Serial.println(count_read_peltier);
          if(count_read_peltier > 5){ //retarda tempo para que peltier acione depois de 5 segundos do funcionamento do Fam      //readControlPeltier       
            digitalWrite(RELAY_2_PIN, HIGH); // 
            //Serial.println(count_read_peltier);
            count_read_peltier = 0;
            stateTemperature = 1;
          }           
             
        }
        break;
      
      case -1:
        if (temperature >= TEMP_IDEAL) {
          stateTemperature = 0;
          digitalWrite(RELAY_1_PIN, LOW); // antes , HIGH (rele de alta)          
        }
        break;
      
      case 1:           
        if (temperature <= TEMP_IDEAL) {
          stateTemperature = 0;
          digitalWrite(RELAY_2_PIN, LOW); // // antes RELAY_2_PIN, HIGH (rele de alta)
          digitalWrite(RELAY_7_PIN, LOW); // desliga o Fan junto com o paltier           
        }
        break;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// --->> CONTROLE DA TEMPERATURA <<--- FIM

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DA UMIDADE DO AR <<---  obs só para baixa umidade  
    switch (stateHumidity) { 
      case 0:                                     //Leitura Anterior = Indicando temperatura ideal | peltier resfriamento e peltier aquecimento desligados
        if (humidity < UR_MIN) {             //Se Leitura Atual = Indicando umidade baixa
          stateHumidity = -1;
          digitalWrite(RELAY_3_PIN, HIGH);     
        } else if (humidity > UR_MAX) {      ////Se Leitura Atual = Indicando umidade alta
          stateHumidity = 1;        
          //digitalWrite(RELAY_2_PIN, LOW);        
        }
        break;
      
      case -1:
        if (humidity >= UR_IDEAL) {
          stateHumidity = 0;
          digitalWrite(RELAY_3_PIN, LOW);
        }
        break;
      
      case 1:
        if (humidity <= UR_IDEAL) {
          stateHumidity = 0;
          //digitalWrite(RELAY_2_PIN, HIGH);        
        }
        break;
    } 
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DA UMIDADE DO AR <<--- FIM

    /*
    ////////////////  --->> CONTROLE DE ILUMINAÇÃO <<--- PARA TESTES POR SENSOR
    
    
    switch (stateRadiation) { 
      case 0:                                     //Leitura Anterior = Indicando temperatura ideal | peltier resfriamento e peltier aquecimento desligados
        if (radiation < RA_MIN) {             //Se Leitura Atual = Indicando tempetatura fria
          stateRadiation = -1;
          digitalWrite(RELAY_4_PIN, HIGH);       
        } else if (radiation > RA_MAX) {      ////Se Leitura Atual = Indicando tempetatura quente
          stateRadiation = 1;        
          //digitalWrite(RELAY_2_PIN, LOW);        
        }
        break;
      
      case -1:
        if (radiation >= RA_IDEAL) {
          stateRadiation = 0;
          digitalWrite(RELAY_4_PIN, LOW);
        }
        break;
      
      case 1:
        if (radiation <= RA_IDEAL) {
          stateRadiation = 0;
          //digitalWrite(RELAY_4_PIN, HIGH);        
        }
        break;
    }
    

    
    ////////////////  --->> CONTROLE DE ILUMINAÇÃO <<--- FIM
    */

    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DE RESERVATÓRIO COM SENSOR ULTRASONIC
    switch (stateUltrasonic) { 
      case 0:                                     //Leitura Anterior = Indicando temperatura ideal | peltier resfriamento e peltier aquecimento desligados
        if (distance >= UDISTANCE_MAX ) {             //distancia maxi do sensor, ou seja nivel minimo
          stateUltrasonic = -1;
          digitalWrite(RELAY_6_PIN, HIGH); //Rele de alta
          //digitalWrite(RELAY_5_PIN, HIGH);
        } else if (distance <= UDISTANCE_MIN) {      ////
          stateUltrasonic = 1; //1;        
          digitalWrite(RELAY_6_PIN, LOW);  //Rele de alta       
        }
        break;
      
      case -1:
        if (distance <= UDISTANCE_MIN) {
          stateUltrasonic = 0;
          digitalWrite(RELAY_6_PIN, LOW); //Rele de alta
        }
        break;
      
      case 1:
        if (distance <= UDISTANCE_MIN) {
          stateUltrasonic = 0;
          digitalWrite(RELAY_6_PIN, LOW);      
        }
        break;      
    } 

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DE RESERVATÓRIO COM SENSOR ULTRASONIC
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DE UMUDADE DO SOLO <<---

    //int stateUltrasonicAux = stateUltrasonic;
    
      switch (statusSoilMoisture) { 
        case 0:                                     //Leitura Anterior = Indicando temperatura ideal | peltier resfriamento e peltier aquecimento desligados
          if (soilMoisture < SM_MIN && stateUltrasonic != -1)  {        //Se Leitura Atual = Indicando solo seco. \\&& reservoirLow != RE_MIN            
            statusSoilMoisture = -1;
            digitalWrite(RELAY_5_PIN, HIGH); //Rele de alta            
          } else if (soilMoisture > SM_MAX) {      ////Se Leitura Atual = Indicando tempetatura quente
            statusSoilMoisture = 1;        
            digitalWrite(RELAY_5_PIN, LOW);        
          }
          break;
        
        case -1:
          if (soilMoisture >= SM_IDEAL || stateUltrasonic == -1) {
            statusSoilMoisture = 0;
            digitalWrite(RELAY_5_PIN, LOW); //Rele de alta
          }
          break;
        
        case 1:
          if (soilMoisture <= SM_IDEAL) {
            statusSoilMoisture = 0;
            //digitalWrite(RELAY_5_PIN, HIGH);        
          }
          break;
      }     

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  --->> CONTROLE DE UMUDADE DO SOLO <<--- FIM

    /*
    switch (statusReservoirLevel) {
      case 0:
      if (reservoirLow <= RE_MIN) {
        statusReservoirLevel = -1;
        digitalWrite(RELAY_5_PIN, LOW);
        //comando: ligando solenoide
      } else if (reservoirHigh >= RE_MAX) {
        statusReservoirLevel = -1;
      }
      break;

      case -1:
      if (reservoirHigh >= RE_MAX) {
        statusReservoirLevel = 0;
        //comando: deliga solenoite
      }
      break;    
    }
    */
    
    ////////////////  --->> CONTROLE DE ILUMINAÇÃO POR FAIXA HORARIA <<--- PARA TESTES POR SENSOR
    
    
    switch (stateLight) {  // hora --> "1910"
      case 0:                                     //Leitura Anterior = Indicando temperatura ideal | peltier resfriamento e peltier aquecimento desligados
        if (relogio_ntp(3) < "0800" && relogio_ntp(3) > "1900") {             // ligar as 0800, deliga as 2000
          stateLight = -1;
          digitalWrite(RELAY_4_PIN, LOW);       
        } else if (relogio_ntp(3) >= "0800" && relogio_ntp(3) <= "1900") {      ////Se Leitura Atual = Indicando tempetatura quente
          stateLight = 1;        
          digitalWrite(RELAY_4_PIN, HIGH);        
        }
        break;
      
      case -1:
        if (relogio_ntp(3) >= "0800" && relogio_ntp(3) <= "1900") {
          stateLight = 0;
          digitalWrite(RELAY_4_PIN, HIGH);
        }
        break;
      
      case 1:
        if (relogio_ntp(3) < "0800" && relogio_ntp(3) > "1900") {
          stateLight = 0;
          digitalWrite(RELAY_4_PIN, LOW);        
        }
        break;
      
    }

    
    readControl   = millis();
  }  
  
  ////////////////  --->> CONTROLE DE ILUMINAÇÃO <<--- FIM  
  
  long now = millis();
  if (now - lastMsg > 10000) { //120000
    lastMsg = now;    
    
    
    // Convert the value to a char array //---------------------------------------->> conversão necessaria para publicação no topico
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    //Serial.print("Temperature: ");
    //Serial.println(tempString);
    client.publish("esp32/temperature/value", tempString);

    
    
    // Convert the value to a char array //---------------------------------------->> conversão necessaria para publicação no topico
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    //Serial.print("Humidity: ");
    //Serial.println(humString);
    client.publish("esp32/humidity/value", humString);

    //Sensor de Umidade do solo, convertendo o valor em uma matriz de caracteres       
    char soilString[8];
    dtostrf(soilMoisture, 1, 2, soilString);
    //Serial.print("soil Moisture: ");
    //Serial.println(soilString);
    client.publish("esp32/soilMoisture/value", soilString);

    /*
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
    */

    
    Serial.print("Hora ------------- ");    
    Serial.println(relogio_ntp(3)); // hora --> "1910"

    Serial.print("Radiação --------- ");
    Serial.print(radiation); 
    Serial.print(",  stateLight: ");  
    Serial.println(stateLight); 

    Serial.print("Tempetarura ------ ");
    Serial.print(temperature);
    Serial.print(", stateTemperature: ");
    Serial.println(stateTemperature);   

    Serial.print("Umidade do Ar ---- ");
    Serial.print(humidity);
    Serial.print(", stateHumidity: ");
    Serial.println(stateHumidity);

    Serial.print("Umidade do Solo -- ");
    Serial.print(soilMoisture);
    Serial.print(", statusSoilMoisture: ");     
    Serial.println(statusSoilMoisture);
    
    Serial.print("Sonar distancia -- ");
    Serial.print(distance);
    Serial.print(", stateUltrasonic: ");
    Serial.println(stateUltrasonic);

    Serial.println();

    Serial.println(" ----------------- ");

    /*
    Serial.print(" | Nivel do reservatório: ");
    Serial.print(distance);

    Serial.print(" - Estado: ");
    Serial.print(stateUltrasonic);

    //Serial.print(" - Estado: ");
    //Serial.print(statusReservoirLevel);

    //Serial.print(" - Estado: ");
    //Serial.print(statusReservoirLevel);
    */




    //Serial.println("----------");

    



    
    
    //Serial.println(relogio_ntp(0)); 

    //Serial.println(relogio_ntp(1)); //data e hora completa formatada --> "06/04/2023 19:10:58"
    //delay(1000);
    //Serial.println(relogio_ntp(2)); //data completa --> "06/04/2023"
    //delay(1000);    
    //Serial.println(relogio_ntp(3)); // hora --> "1910"
    
    Serial.println();
    //delay(1000);        
  }    
}