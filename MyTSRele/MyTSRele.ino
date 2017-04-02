// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>

#define SN "Rele Radiador"
#define SV "1.0"

/*
 * Pines utilizados
 */
#define PIN_RELAY 3

/*
 * Direcciones EEPROM para guardar estado
 */
#define EEPROM_V_STATUS 0

/*
 * S_DIMMER
 */
bool relay_status = false; //true: ON - false: OFF
int demand = 0;
MyMessage msg_status(0, V_STATUS);
MyMessage msg_dimmer(0, V_PERCENTAGE);

/*
 * Temporizado
 */
#define DUTY_CYCLE 10000 //ms
#define UPDATE_RATE 600000 //ms
unsigned long timing_relay;
unsigned long timing_update;

void setup()
{
  relay_status = loadState(EEPROM_V_STATUS);
  pinMode(PIN_RELAY, OUTPUT);
  timing_relay = millis();
  timing_update = millis();
}

void presentation()
{
	sendSketchInfo(SN, SV);
	present(0, S_DIMMER, "Rele control de potencia");
}

void loop()
{
  if(relay_status) {
    if (millis() - timing_relay > DUTY_CYCLE)
      timing_relay = millis();
    if (demand * 100 < (millis() - timing_relay))
      digitalWrite(PIN_RELAY, HIGH);
    else 
      digitalWrite(PIN_RELAY, LOW);
    
  }
  else {
    digitalWrite(PIN_RELAY, LOW);
  }
  
  /*
   * Actualiza datos del controlador
   */
  if((unsigned long)(millis() - timing_update) > UPDATE_RATE) {
    send(msg_dimmer.set(demand));
    send(msg_status.set(relay_status));
    timing_update = millis();
  }
}

/*
 * Set V_STATUS
 */
void setV_Status (bool new_status) {
  if (relay_status != new_status) {
    relay_status = new_status;
    saveState(EEPROM_V_STATUS, relay_status);
    send(msg_status.set(relay_status));
    if (relay_status) timing_relay = millis();
  }
}

/*
 * Set V_PERCENTAGE
 */
void setV_Percentage(int percentage) {
  if (demand != percentage) {
    demand = percentage;
    send(msg_dimmer.set(demand));
  }
}

void receive(const MyMessage &message)
{
	if (message.type == V_STATUS) {
    setV_Status(message.getBool());
  } 
  else if (message.type == V_PERCENTAGE) {
    setV_Percentage(message.getInt());
  }
}
