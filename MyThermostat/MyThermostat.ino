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
int demand;
MyMessage msg_setpoint(0, V_HVAC_SETPOINT_HEAT);
MyMessage msg_temp(0, V_TEMP);
MyMessage msg_status(0, V_STATUS);
//Los siguientes mensajes son para enviar datos
//al nodo esclavo que maneja el rele,
//corresponden a un S_DIMMER pero no lo meto en
//presentation
MyMessage msg_demand(1, V_PERCENTAGE);
MyMessage msg_status_relay(1, V_STATUS);

/*
 * Temporizado
 */
#define DALLAS_SAMPLE_RATE 5100 //ms
#define PID_SAMPLE_RATE 10000 //ms
unsigned long timing_dallas;
unsigned long timing_pid;
#define SHORT_REFRESH 10000 //ms
#define LONG_REFRESH 600000 //ms
unsigned long REFRESH_RATE;
unsigned long timing_refresh;

/*
 * Misc
 */
#define REQ_ACK 1
#define SP_INCDEC 0.5

void setup()
{
  sp = loadFloat(EEPROM_V_SETPOINT);
  ts_status = loadState(EEPROM_V_STATUS);
   
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
  sensors.setWaitForConversion(false);
  sensors.requestTemperaturesByAddress(sondaTermometro);
  sleep(1000);
  //pv = sensors.getTempC(sondaTermometro);
  //sensors.requestTemperaturesByAddress(sondaTermometro);
  pv = 21.0;

  /*
   * Iniciar pantalla
   */
  lcd.init();
  lcd.setCursor(0,0);
  lcd.print("PV   SP   OUT   ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  if (ts_status) lcd.backlight();
    else lcd.noBacklight();
  
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
  send(msg_status.set(ts_status), REQ_ACK);
  send(msg_setpoint.set(sp, 1), REQ_ACK);
  send(msg_status_relay.set(ts_status).setDestination(1), REQ_ACK);

  timing_dallas = millis();
  timing_pid = millis();
  timing_refresh = millis();
  REFRESH_RATE = LONG_REFRESH;
  
}

void presentation()
{
	sendSketchInfo(SN, SV);
  present(0, S_HEATER, "Radiador del Salon");
}


void loop()
{
  /*
   * Lectura del sensor de temperatura
   */
  if((unsigned long)(millis() - timing_dallas) > DALLAS_SAMPLE_RATE) {
    double last_pv = pv;
    //pv = sensors.getTempC(sondaTermometro);
    //sensors.requestTemperaturesByAddress(sondaTermometro);
    pv = 22.0;
    if (pv != last_pv) {
      send(msg_temp.set(pv, 1));
    }
    timing_dallas = millis();
  }

  /*
   * Evaluacion del PID
   */
  if ((unsigned long)(millis() - timing_pid) > PID_SAMPLE_RATE) {
    if(!isnan(pv) && !isnan(sp)) {
      int lastDemand = demand;
      if (myPID.Compute()) {
        demand = (int) out;
        if (demand != lastDemand) send(msg_demand.set(demand).setDestination(1));
      }        
    }    
    timing_pid = millis();
  }

  /*
   * Refresco de datos en controlador y esclavo
   */
  if((unsigned long)(millis() - timing_refresh) > REFRESH_RATE) {
    Serial.println("Refresco");
    bool ackok;
    ackok = send(msg_status.set(ts_status), REQ_ACK);
    ackok = ((send(msg_status_relay.set(ts_status).setDestination(1), REQ_ACK)) && ackok);
    ackok = ((send(msg_setpoint.set(sp, 1), REQ_ACK)) && ackok);
    if(ackok) REFRESH_RATE = LONG_REFRESH;
    timing_refresh = millis();
  }   

  /*
   * Actualiza display
   */
  lcd.setCursor(15,0);
  if (REFRESH_RATE == LONG_REFRESH) lcd.print("S");
  else lcd.print("F");
  lcd.setCursor(0,1);
  lcd.print(pv, 1);
  lcd.setCursor(5,1);
  lcd.print(sp, 1);
  lcd.setCursor(10,1);
  lcd.print(demand);
  lcd.print("  ");

  /*
   * Pulsadores
   */
  onoff.update();
  inc.update();
  dec.update();
  
  if(inc.fell()) {
    setV_Setpoint(sp + SP_INCDEC);
  }
  if(dec.fell()) {
    setV_Setpoint(sp - SP_INCDEC);
  }
  if(onoff.fell()) {
    setV_Status(!ts_status);
  }
}

/*
 * Convierte un float a bytes y lo almacena
 * en posiciones de EEPROM
 */
void saveFloat(const uint8_t pos, const float value) {
  union float_bytes {
    float val;
    byte bytes[sizeof(float)];
  } data;
  data.val = value;
  for (int i = 0; i < sizeof(float); i++) {
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
 * Cambia de estado encendido/apagado
 */
void setV_Status (bool new_ts_status) {
  if (ts_status != new_ts_status) {
    ts_status = new_ts_status;
    saveState(EEPROM_V_STATUS, ts_status);
    myPID.SetMode(ts_status ? AUTOMATIC : MANUAL);
    if(send(msg_status.set(ts_status), REQ_ACK) == false) REFRESH_RATE = SHORT_REFRESH;
    if(send(msg_status_relay.set(ts_status).setDestination(1), REQ_ACK) == false) REFRESH_RATE = SHORT_REFRESH;;
    if(ts_status) lcd.backlight();
      else lcd.noBacklight();
  }
}

/*
 * Fijar el valor de setpoint
 */
void setV_Setpoint(float setpoint) {
  if (sp != setpoint) {
    sp = setpoint;
    saveFloat(EEPROM_V_SETPOINT, sp);
    if(send(msg_setpoint.set(sp, 1), REQ_ACK) == false) REFRESH_RATE = SHORT_REFRESH;    
  }
}

void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_HEAT) {
    setV_Setpoint(message.getFloat());
  } else if (message.type == V_STATUS) {
    setV_Status(message.getBool());
    
  }
}
