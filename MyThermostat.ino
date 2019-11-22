#define SN "MyThermostat"
#define SV "1.3"
// Enable debug prints to serial monitor
//#define MY_DEBUG
// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER
//Wait gateway for 5sg
#define MY_TRANSPORT_WAIT_READY_MS 5000
// OTA Firmware update settings
#define MY_OTA_FIRMWARE_FEATURE
// Signing setup
#define MY_SIGNING_ATSHA204
#define MY_SIGNING_REQUEST_SIGNATURES

#include <MySensors.h>
#include <Keypad.h>
#include "OledDisplay.h"
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
   Used pins
*/
#define PIN_RELAY A0
#define PIN_TEMP  7

/*
   EEPROM addresses for config and state storage.
*/
#define EEPROM_V_SETPOINT 0
#define EEPROM_V_STATUS 4
#define EEPROM_SAFETY_ADDRESS 6
#define EEPROM_CONTROL_ADDRESS 14

/*
    DS18B20 temperature sensors.
*/
OneWire oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
DeviceAddress room_sensor, safety_sensor;
float safety_temp;
double room_temp;

/*
   PID controller
*/
#define KP 10000
#define KI 5
#define KD 0
double setpoint, pid_out;
PID myPID(&room_temp, &pid_out, &setpoint, KP, KI, KD, DIRECT);

/*
   Display OLED I2C 0,96"
*/
OledDisplay oled;

/*
   2x2 Keypad
*/
const byte ROWS = 2;
const byte COLS = 2;
byte rowPins[ROWS] = {5, 3};
byte colPins[COLS] = {6, 4};
char keys[ROWS][COLS] = {
  {'-', '+'},
  {'M', 'O'}
};
Keypad keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
double kp_setpoint;
bool kp_ts_switch;

/*
   S_HEATER
*/
MyMessage msg_setpoint(0, V_HVAC_SETPOINT_HEAT);
MyMessage msg_temp(0, V_TEMP);
MyMessage msg_status(0, V_STATUS);
bool ts_switch = false; //true: ON - false: OFF

//typedef enum 
//{ 
//    ON,
//    OFF    
//} ts_switch;

typedef enum 
{
    NONE,
    RECEIVED, 
    PENDING
} Ack;
Ack setpoint_ack, ts_switch_ack = NONE;

/*
   Temporizado
*/
#define DUTY_CYCLE 10000 //ms
#define DALLAS_RATE 2000 //ms
#define BACKLIGHT_TIME 17100 //ms
#define REFRESH_RATE_2MIN 120000 //ms
#define REFRESH_RATE_13MIN 780000 //ms

unsigned long timing_cycle;
unsigned long timing_dallas;
unsigned long timing_lcdbacklight;
unsigned long timing_temp_refresh;
unsigned long timing_ack_request;

/*
   Misc
*/
#define REQ_ACK 1
#define SP_INCDEC 0.5

void setup()
{
  /*
     Restaurar ultimo estado
  */
  setpoint = loadFloat(EEPROM_V_SETPOINT);
  ts_switch = loadState(EEPROM_V_STATUS);
  if (isnan(setpoint) || (setpoint < 5.0) || (setpoint > 30.0)) setpoint = 20.0;

  /*
     Configurar controlador PID.
     Salida: de 0-10000 ciclo de trabajo 10000ms.
     Modo: AUTO/MAN en funcion del estado cargado.
     SampleTime: 10sg:
  */
  myPID.SetOutputLimits(0, DUTY_CYCLE);
  myPID.SetSampleTime(DUTY_CYCLE);
  myPID.SetMode(ts_switch ? AUTOMATIC : MANUAL);

  /*
     Configurar sensor Dallas y tomar primera lectura.
  */
  for (uint8_t i = 0; i < 8; i++)
  {
    room_sensor[i] = loadState(EEPROM_CONTROL_ADDRESS + i);
    safety_sensor[i] = loadState(EEPROM_SAFETY_ADDRESS + i);
  }
  sensors.begin();
  sensors.setResolution(room_sensor, 12);
  sensors.setResolution(safety_sensor, 9);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);
  room_temp = sensors.getTempC(room_sensor);
  safety_temp = sensors.getTempC(safety_sensor);

  /*
     Iniciar pantalla
  */
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(font8x8);
  oled.set2X();

  /*
     Configurar pin de salida del rele
  */
  pinMode(PIN_RELAY, OUTPUT);

  /*
     Keypad
  */
  keypad.addEventListener(keypadEvent);

  timing_cycle = millis();
  timing_dallas = 0;
  timing_temp_refresh = millis();
  timing_ack_request = millis();
  timing_lcdbacklight = millis();


  send(msg_temp.set(room_temp, 1));
  send(msg_setpoint.set(setpoint, 1));
  send(msg_status.set(ts_switch));
}

void presentation()
{
  sendSketchInfo(SN, SV);
  present(0, S_HEATER);
}

void loop()
{
  /*
     Lectura de los sensores de temperatura locales.
  */
  if ((unsigned long)(millis() - timing_dallas) >= DALLAS_RATE) {
    room_temp = sensors.getTempC(room_sensor);
    safety_temp = sensors.getTempC(safety_sensor);
    sensors.requestTemperatures();
    timing_dallas = millis();
  }

  /*
     Evaluacion del lazo PID y actuacion rele.
  */
  myPID.Compute();
  digitalWrite(PIN_RELAY, 
    (ts_switch && 
    (safety_temp < 55) && 
    ((unsigned long)(millis() - timing_cycle) < pid_out)) ? HIGH : LOW);
  if ((unsigned long)(millis() - timing_cycle) >= DUTY_CYCLE) timing_cycle = millis();

  /*
     Send variables to controller
  */
  if ((unsigned long)(millis() - timing_temp_refresh) >= REFRESH_RATE_13MIN) {
    send(msg_temp.set(room_temp, 1));
    timing_temp_refresh = millis();
  }

  if ((unsigned long)(millis() - timing_ack_request) >= REFRESH_RATE_2MIN) {
    if (setpoint_ack == PENDING) {
      send(msg_setpoint.set(setpoint, 1), REQ_ACK);
    }
    if (ts_switch_ack == PENDING)  {
      send(msg_status.set(ts_switch), REQ_ACK);
    }
    timing_ack_request = millis();
  }

  /*
     Refresco del display y apagado tras un tiempo de inactividad del teclado.
     Cuando se apaga el display se asume la finalizacion de la entrada de datos
     mediante teclado y se envian al controlador.
  */
  if (oled.isEnabled()) {
    oled_refresh();
    if ((unsigned long)(millis() - timing_lcdbacklight) >= BACKLIGHT_TIME) {
      setSetpoint(kp_setpoint, true);
      setStatus(kp_ts_switch, true);
      oled.setDisabled();
    }
  }

  /*
     Realizar lectura del keypad.
     No almaceno la lectura porque uso eventos
  */
  keypad.getKey();
}

void keypadEvent(KeypadEvent key) {
  switch (keypad.getState()) {
    case PRESSED:
      timing_lcdbacklight = millis();
      if (!oled.isEnabled()) {
        kp_setpoint = setpoint;
        kp_ts_switch = ts_switch;
        oled.setEnabled();
        break;
      }

      switch (key) {
        case '+': kp_setpoint = kp_setpoint + SP_INCDEC; break;
        case '-': kp_setpoint = kp_setpoint - SP_INCDEC; break;
        case 'M': break;
        case 'O': kp_ts_switch = !kp_ts_switch; break;
      }
      break;
  }
}

void oled_refresh (void) {
  oled.home();
  oled.print(room_temp, 1); oled.println(kp_ts_switch ? "| ON" : "|OFF");
  oled.println((safety_temp > 50) ? "-ALARMA-" : "--------");
  oled.print("SP: "); oled.println(kp_setpoint, 1);
  int out_pcnt = (int)(pid_out / 100);
  if (out_pcnt < 10) {
    oled.print("Out:   ");
    oled.println(out_pcnt);
  } else if (out_pcnt < 100) {
    oled.print("Out:  ");
    oled.println(out_pcnt);
  } else {
    oled.print("Out: 100");
  }
}

/*
   Convierte un float a bytes y lo almacena
   en posiciones de EEPROM
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
   Carga bytes de la EEPROM y devuelve su float correspondiente
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
   setSetpoint
*/
bool setSetpoint (float new_setpoint, bool fromKeyPad) {
  if ((new_setpoint != setpoint) && (new_setpoint >= 5.0) && (new_setpoint <= 30.0)) {
    setpoint = new_setpoint;
    if (fromKeyPad) {
      send(msg_setpoint.set(setpoint, 1), REQ_ACK);
      setpoint_ack = PENDING;
    }
    saveFloat(EEPROM_V_SETPOINT, setpoint);
    return true;
  }
  return false;
}

/*
   setStatus
*/
bool setStatus (bool v_status, bool fromKeyPad) {
  if (v_status != ts_switch) {
    ts_switch = v_status;
    if (fromKeyPad) {
      send(msg_status.set(ts_switch), REQ_ACK);
      ts_switch_ack = PENDING;
    }
    saveState(EEPROM_V_STATUS, ts_switch);
    myPID.SetMode(ts_switch ? AUTOMATIC : MANUAL);
    return true;
  }
  return false;
}

void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_HEAT) {
    if (message.isAck())
      setpoint_ack = RECEIVED;
    else
      setSetpoint(message.getFloat(), false);
  } else if (message.type == V_STATUS) {
    if (message.isAck())
      ts_switch_ack = RECEIVED;
    else
      setStatus(message.getBool(), false);
  }
}
