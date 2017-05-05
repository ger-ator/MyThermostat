#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>

#define MY_ESP8266_SSID "essid"
#define MY_ESP8266_PASSWORD "password"
#define MY_ESP8266_HOSTNAME "dimmer_pruebas"

#define SN "Rele control de potencia"
#define SV "1.1"

/*
 * Pines utilizados
 */
#define PIN_RELAY D0

/*
 * Direcciones EEPROM para guardar estado
 */
#define EEPROM_V_STATUS 0

/*
 * S_DIMMER
 */
bool relay_status = false; //true: ON - false: OFF
int demand = 0;
unsigned long ontime = 0;
MyMessage msg_status(0, V_STATUS);
MyMessage msg_dimmer(0, V_PERCENTAGE);

/*
 * S_TEMP
 */
MyMessage msg_temp(1, V_TEMP);

/*
 * Temporizado
 */
#define DALLAS_SAMPLE_RATE 71000 //ms
#define DUTY_CYCLE 10000 //ms
#define REFRESH_RATE 600000 //ms
unsigned long timing_relay;
unsigned long timing_update;
unsigned long timing_dallas;

/*
 * Misc
 */
#define REQ_ACK 1

/*
 * Monitorizado de Temperatura de la placa
 */
#define NUM_TEMP_SENSORS 2
#define ONEWIRE_BUS D3
OneWire oneWire(ONEWIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sondaTermometro [NUM_TEMP_SENSORS];
float lastTemp [NUM_TEMP_SENSORS];

void setup()
{
  WiFi.mode (WIFI_STA);
  WiFi.hostname(MY_ESP8266_HOSTNAME);
  WiFi.begin(MY_ESP8266_SSID, MY_ESP8266_PASSWORD);
  wait(1000);
  ArduinoOTA.setHostname("salon");
  ArduinoOTA.begin();
  
  relay_status = loadState(EEPROM_V_STATUS);
  pinMode(PIN_RELAY, OUTPUT);

  sensors.begin();
  for(int i = 0; i < NUM_TEMP_SENSORS; i++){
    sensors.getAddress(sondaTermometro[i], i);
  }
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);
  
  timing_relay = millis();
  timing_update = millis();
  timing_dallas = millis();
}

void presentation()
{
	sendSketchInfo(SN, SV);
	present(0, S_DIMMER, "Radiador del Salon", REQ_ACK);
  
  for(uint8_t i = 1; i <= NUM_TEMP_SENSORS; i++) {
    present(i, S_TEMP);    
  }
}

void loop()
{
  if ((unsigned long)(millis() - timing_relay) >= DUTY_CYCLE) {
    ontime = demand * 100;
    timing_relay = millis();
  }
    
  if ((relay_status) && ((unsigned long)(millis() - timing_relay) <= ontime)) {
    digitalWrite(PIN_RELAY, HIGH);
  } else {
    digitalWrite(PIN_RELAY, LOW);    
  }

  /*
   * Toma datos de los sensores de temperatura y los envia
   */
  if ((unsigned long)(millis() - timing_dallas) >= DALLAS_SAMPLE_RATE) {
    for (uint8_t i = 0; i < NUM_TEMP_SENSORS; i++) {
      float temp = sensors.getTempC(sondaTermometro[i]);
      if (temp != lastTemp[i]) {
        send(msg_temp.setSensor(i + 1).set(temp, 1));
        lastTemp [i] = temp;
      }
    }
    sensors.requestTemperatures();
    timing_dallas = millis();
  }
  
  /*
   * Actualiza datos del controlador
   */
  if((unsigned long)(millis() - timing_update) >= REFRESH_RATE) {
    send(msg_dimmer.set(demand));
    send(msg_status.set(relay_status));
    timing_update = millis();
  }
  
  ArduinoOTA.handle();
}

void receive(const MyMessage &message)
{
	if (message.type == V_STATUS) {
    if (relay_status != message.getBool()) {
      relay_status = message.getBool();
      saveState(EEPROM_V_STATUS, relay_status);
      if (message.sender != 0) send(msg_status.set(relay_status));
    }
  } 
  else if (message.type == V_PERCENTAGE) {
    if (demand != message.getInt()) {
      demand = message.getInt();
      if (message.sender != 0) send(msg_dimmer.set(demand));
    }
  }
}
