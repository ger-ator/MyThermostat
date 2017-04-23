// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

#include <MySensors.h>

#define SN "Termostato"
#define SV "1.0"

#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Bounce2.h>

/*
 * Pines utilizados
 */
#define PIN_BOT_UP 3
#define PIN_BOT_DOWN 4
#define PIN_BOT_ONOFF 5
#define PIN_TEMP  6

/*
 * Direcciones EEPROM para guardar estado
 */
#define EEPROM_V_SETPOINT 0
#define EEPROM_V_STATUS 4
#define EEPROM_SLAVE_NODE 5

/*
 * Controlador PID
 */
#define KP 45
#define KI 0.05
#define KD 0
double sp, pv, out;
PID myPID(&pv, &out, &sp, KP, KI, KD, DIRECT);

/*
 *  Sonda temperatura DS18B20
 */
OneWire oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
DeviceAddress sondaTermometro = {0x28, 0xFF, 0x00, 0xDC, 0x86, 0x16, 0x05, 0x91};

/*
 * Display I2C de 16x2 caracteres
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool config_mode;

/*
 * Debouncers para los pulsadores
 */
Bounce onoff = Bounce();
Bounce inc = Bounce();
Bounce dec = Bounce();

/*
 * S_HEATER
 */
bool ts_status = false; //true: ON - false: OFF
bool relay_ack = false; //true: ACK - false: NACK
int demand;
uint8_t slave_node;
MyMessage msg_setpoint(0, V_HVAC_SETPOINT_HEAT);
MyMessage msg_temp(0, V_TEMP);
MyMessage msg_status(0, V_STATUS);
MyMessage msg_demand(0, V_PERCENTAGE);

/*
 * Temporizado
 */
#define DALLAS_SAMPLE_RATE 5100 //ms
#define PID_SAMPLE_RATE 10000 //ms
#define LONG_PRESS 2000 //ms
#define BACKLIGHT_TIME 10000 //ms
unsigned long timing_dallas;
unsigned long timing_pid;
unsigned long timing_pressed;
unsigned long timing_lcdbacklight;

/*
 * Misc
 */
#define REQ_ACK 1
#define SP_INCDEC 0.5

void setup()
{
  sp = loadFloat(EEPROM_V_SETPOINT);  
  ts_status = loadState(EEPROM_V_STATUS);
  slave_node = loadState(EEPROM_SLAVE_NODE);
  if(isnan(sp) || (sp < 5.0) || (sp > 30.0)) sp = 20.0;
  if(isnan(slave_node) || slave_node < 0 || slave_node > 255) slave_node = 0;
     
  /*  
   * Configurar controlador PID.
   * Salida: de 0-100 para actuar como variable para dimmer.
   * Modo: AUTO/MAN en funcion del estado cargado.
   * SampleTime: 10sg: 
   */
  myPID.SetOutputLimits(0, 100);
  myPID.SetMode(ts_status ? AUTOMATIC : MANUAL);  
  myPID.SetSampleTime(PID_SAMPLE_RATE);

  /*
   * Configurar sensor Dallas y tomar primera lectura.
   */
  sensors.begin();
  sensors.setResolution(sondaTermometro, 12);
  sensors.requestTemperaturesByAddress(sondaTermometro);
  pv = sensors.getTempC(sondaTermometro);
  sensors.setWaitForConversion(false);
  sensors.requestTemperaturesByAddress(sondaTermometro);
  

  /*
   * Iniciar pantalla
   */
  lcd.init();
  set_PidMode();
  
  /*
   * Configuracion de pines
   */
  pinMode(PIN_BOT_ONOFF, INPUT_PULLUP);
  pinMode(PIN_BOT_UP, INPUT_PULLUP);
  pinMode(PIN_BOT_DOWN, INPUT_PULLUP);
  onoff.attach(PIN_BOT_ONOFF);
  inc.attach(PIN_BOT_UP);
  dec.attach(PIN_BOT_DOWN);
  onoff.interval(10);  
  inc.interval(10);  
  dec.interval(10);
  
  /*
   * Enviar estado inicial al controlador y al nodo esclavo
   */
  send(msg_temp.set(pv, 1));
  send(msg_setpoint.set(sp, 1));
  send(msg_status.set(ts_status).setDestination(0));
  send(msg_status.set(ts_status).setDestination(slave_node));

  timing_dallas = millis();
  timing_pid = millis();
}

void presentation()
{
	sendSketchInfo(SN, SV);
  present(0, S_HEATER, "Termostato del Salon", REQ_ACK);
}


void loop()
{
  /*
   * Lectura del sensor de temperatura
   */
  if((unsigned long)(millis() - timing_dallas) >= DALLAS_SAMPLE_RATE) {
    double last_pv = pv;
    pv = sensors.getTempC(sondaTermometro);
    sensors.requestTemperaturesByAddress(sondaTermometro);
    if (pv != last_pv) {
      send(msg_temp.set(pv, 1));
    }
    timing_dallas = millis();
  }

  /*
   * Evaluacion del PID
   */
  if ((unsigned long)(millis() - timing_pid) >= PID_SAMPLE_RATE) {
    if(!isnan(pv) && !isnan(sp)) {
      int lastDemand = demand;
      if (myPID.Compute()) {
        demand = (int) out;
        if (relay_ack) {
          if (demand != lastDemand) send(msg_demand.set(demand).setDestination(slave_node));
        } else {
          relay_ack = send(msg_status.set(ts_status).setDestination(slave_node), REQ_ACK);
          if (relay_ack) send(msg_demand.set(demand).setDestination(slave_node));
        }
      }        
    }    
    timing_pid = millis();
  }

  /*
   * Temporizado de la retroiluminacion
   */
  if ((unsigned long)(millis() - timing_lcdbacklight) >= BACKLIGHT_TIME) {
    lcd.noBacklight();
  }

  /*
   * Actualiza display
   */
  if (config_mode) {
    lcd.setCursor(0,1);
    lcd.print(slave_node);
    lcd.print("  ");        
  } else {
    lcd.setCursor(6, 0);
    lcd.print(pv, 1);
    lcd.setCursor(4, 1);
    lcd.print(sp, 1);
    lcd.setCursor(13, 1);
    lcd.print(demand);
    lcd.print("  ");
  }
  lcd.setCursor(13,0);
  lcd.print((ts_status ? "ON " : "OFF"));

  /*
   * Pulsadores
   */
  onoff.update();
  inc.update();
  dec.update();

  if(inc.fell() || dec.fell() || onoff.fell()) {
    lcd.backlight();
    timing_lcdbacklight = millis();
  }

  if(inc.fell() && dec.fell()) {
    set_ConfigMode();
  }

  if (config_mode) {
    if(inc.fell()) {
      slave_node++;
    }
    if(dec.fell()) {
      slave_node--;
    }
    if(onoff.fell()) {
      saveState(EEPROM_SLAVE_NODE, slave_node);
      set_PidMode();
    }    
  } else {
    if(inc.fell()) {
      float new_sp = sp + SP_INCDEC;
      if (new_sp <= 30.0) {
        send(msg_setpoint.set(new_sp, 1), REQ_ACK);        
      }
    }
    if(dec.fell()) {
      float new_sp = sp - SP_INCDEC;
      if (new_sp >= 5.0) {
        send(msg_setpoint.set(new_sp, 1), REQ_ACK);        
      }
    }
    if(onoff.fell()) {
      timing_pressed = millis();
    }
    if(onoff.rose()) {
      if((unsigned long)(millis() - timing_pressed) >= LONG_PRESS) {
        //EJECUTAR ESCENA APAGAR TODOS LOS NODOS
        Serial.println("pulsacion larga");
      } else {
        send(msg_status.set(!ts_status).setDestination(0), REQ_ACK);
      }
    }
  }
}

/*
 * Convierte un float a bytes y lo almacena
 * en posiciones de EEPROM
 */
void saveFloat(const uint8_t pos, const float value) {
  Serial.println("Llamada a savefloat");
  union float_bytes {
    float val;
    byte bytes[sizeof(float)];
  } data;
  data.val = value;
  for (int i = 0; i < sizeof(float); i++) {
    Serial.print("Almaceno byte ");
    Serial.println(i + 1);
    saveState(pos + i, data.bytes[i]);
  }  
}

/*
 * Carga bytes de la EEPROM y devuelve su float correspondiente
 */
float loadFloat (const uint8_t pos) {
  union float_bytes {
    float val;
    byte bytes[sizeof(float)];
  } data;
  for (int i = 0; i < sizeof(float); i++) {
    data.bytes[i] = loadState(pos + i);
  }
  return data.val;    
}

/*
 * set_ConfigMode
 */
void set_ConfigMode (void) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Slave Node: ");
  config_mode = true;  
}

/*
 * set_ConfigMode
 */
void set_PidMode (void) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp:           ");
  lcd.setCursor(0,1);
  lcd.print("Sp:      Dm:    ");
  config_mode = false;  
}

void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_HEAT) {
    float new_sp = message.getFloat();
    if ((sp != new_sp) && (new_sp >= 5.0) && (new_sp <= 30.0)) {
      sp = new_sp;
      saveFloat(EEPROM_V_SETPOINT, sp);
      //Si el mensaje NO es ACK quiere decir que se ha enviado el setpoint 
      //desde el controlador. Domoticz no actualiza el valor cuando se envia
      //aun estando configurado ACK, luego hay que enviarle el 
      //nuevo valor al controlador
      if (!message.isAck()) send(msg_setpoint.set(sp, 1));
    }    
  } else if ((message.type == V_STATUS) && (message.sender == 0)) {
    bool new_ts_status = message.getBool();
    if (ts_status != new_ts_status) {
      ts_status = new_ts_status;
      saveState(EEPROM_V_STATUS, ts_status);
      myPID.SetMode(ts_status ? AUTOMATIC : MANUAL);
      relay_ack = send(msg_status.set(ts_status).setDestination(slave_node), REQ_ACK);
    }
  }
}
