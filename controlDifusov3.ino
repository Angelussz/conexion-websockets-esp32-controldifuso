#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <DHT.h> 
#include <Ticker.h>
#else
#error "Placa de desarrollo no encontrada"
#endif

#define DHTPIN 4

#define DHTTYPE    DHT11
#define SOIL_PIN 36
DHT dht(DHTPIN, DHTTYPE);

void enviarDatosSensor();
Ticker timer;
#include <WebSocketsServer.h>
WebSocketsServer websockets(81);

//------------BOMBA AGUA ---------------------
// Motor A
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 0;
//---------------- -------------------
// ---------- Flujo --------------

#define SENSOR  2

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

// ----------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  switch (type) 
  {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] ¡Desconectado!\n", num);
      break;
    case WStype_CONNECTED: {
        IPAddress ip = websockets.remoteIP(num);
        Serial.printf("[%u] Conectado en %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        websockets.sendTXT(num, "Conectado en servidor:");
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Texto: %s\n", num, payload);
      String mensaje = String((char*)( payload));
      Serial.println(mensaje);
      
      DynamicJsonDocument doc(200); // documento (capacidad)
      DeserializationError error = deserializeJson(doc, mensaje);
      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
        }
        
      //int estadoLed = doc["Led"]; // el estado será de 0 o 1
      dutyCycle = doc["Motor"];
      ledcWrite(pwmChannel, dutyCycle);
      if(doc["Flujo"] == 0){
        totalMilliLitres = 0;
      }
      
  }
}

void setup(void)
{
//  pinMode(Led, OUTPUT);
  // ------------ ENCENDER MOTOR
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  

  ledcSetup(pwmChannel, freq, resolution);

  ledcAttachPin(enable1Pin, pwmChannel);
  ledcWrite(pwmChannel, dutyCycle); // Vel Motor
  
  // ---------------
  //------- ENCENDER MOTOR -----------
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  //-----------------

  //--------Flujo -------------
  
  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  
  //-------------------
  Serial.begin(115200);
  WiFi.softAP("redRiego", "angelo2024");
  Serial.println("soft Access Point: ");
  Serial.println(WiFi.softAPIP());

  websockets.begin();
  websockets.onEvent(webSocketEvent);
   timer.attach(2,enviarDatosSensor); 
  
}

void loop(void)
{
  websockets.loop();
}

void enviarDatosSensor() {
 int soil = analogRead(SOIL_PIN);
 float h = dht.readHumidity();
  float t = dht.readTemperature();
  //------ Flujo -----------
  
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    
    pulse1Sec = pulseCount;
    pulseCount = 0;
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) * 1000;

    totalMilliLitres += flowMilliLitres;
  }
  
  //-----------------
  if (isnan(h) || isnan(t) ) {
    Serial.println(F("Fallo de lectura en sensor DHT11!"));
    return;
    }
   
   String JSON_Data = "{\"temp\":";
          JSON_Data += t;
          JSON_Data += ",\"hum\":";
          JSON_Data += h;
          JSON_Data += ",\"soil\":";
          JSON_Data += soil;
          JSON_Data += ",\"ml\":";
          JSON_Data += totalMilliLitres;
          JSON_Data += "}";
          
   Serial.println(JSON_Data);
   websockets.broadcastTXT(JSON_Data);  // envia datos a todos los clientes conectados
}
