#define SN "MyThermostat"
#define SV "1.4"
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
float safety_temp, room_temp, setpoint;
int power_out;

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
float kp_setpoint;
bool kp_ts_switch;

/*
   S_HEATER
*/
MyMessage msg_setpoint(0, V_HVAC_SETPOINT_HEAT);
MyMessage msg_temp(0, V_TEMP);
MyMessage msg_status(0, V_STATUS);
bool ts_switch; //true: ON - false: OFF

typedef enum
{
  NONE,
  RECEIVED,
  PENDING
} Echo;
Echo setpoint_echo, ts_switch_echo = NONE;

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
unsigned long timing_echo_request;

/*
   Misc
*/
#define REQUEST_ECHO true
#define SP_INCDEC 0.5
#define KEEP_TEMP_BAND 0.2
#define FULL_POWER (DUTY_CYCLE)
#define HIGH_POWER (FULL_POWER * 0.55)
#define LOW_POWER (FULL_POWER * 0.35)
#define ZERO_POWER 0

void setup()
{
  /*
     Last status restore
  */
  setpoint = loadFloat(EEPROM_V_SETPOINT);
  ts_switch = loadState(EEPROM_V_STATUS);
  if (isnan(setpoint) 
      || (setpoint < 5.0) 
      || (setpoint > 30.0)) setpoint = 20.0;
  power_out = 0;

  /*
     Load sensor addresses, setup and take first reading.
     Setup WaitForConversion for non blocking code.
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
     OLED 0,96" display library setup
  */
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(font8x8);
  oled.set2X();
  oled.setEnabled();

  /*
     Relay output setup
  */
  pinMode(PIN_RELAY, OUTPUT);

  /*
     Add event listener for 2x2 keypad
  */
  keypad.addEventListener(keypadEvent);
  kp_setpoint = setpoint;
  kp_ts_switch = ts_switch;

  /*
     Initialize timing counters.
  */
  timing_cycle = timing_dallas = 0;
  timing_temp_refresh = timing_echo_request = timing_lcdbacklight = millis();

  /*
     Send initial status to controller.
  */
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
  if (millis() - timing_dallas >= DALLAS_RATE) {
    room_temp = sensors.getTempC(room_sensor);
    safety_temp = sensors.getTempC(safety_sensor);
    sensors.requestTemperatures();
    timing_dallas = millis();
  }

  /*
     Power evaluation and relay actuation.
  */
  if (millis() - timing_cycle >= DUTY_CYCLE) {
    if (room_temp < setpoint - KEEP_TEMP_BAND) {
      power_out = FULL_POWER;
    } else if (room_temp < setpoint) {
      power_out =  HIGH_POWER;
    } else if (room_temp < setpoint + KEEP_TEMP_BAND) {
      power_out =  LOW_POWER;
    } else {
      power_out = ZERO_POWER;
    }
    timing_cycle = millis();
  }

  digitalWrite(PIN_RELAY,
                (ts_switch &&
                (safety_temp < 55) &&
                (millis() - timing_cycle < power_out)) ? HIGH : LOW);

  /*
     Send variables to controller
  */
  if (millis() - timing_temp_refresh >= REFRESH_RATE_13MIN) {
    send(msg_temp.set(room_temp, 1));
    timing_temp_refresh = millis();
  }

  if (millis() - timing_echo_request >= REFRESH_RATE_2MIN) {
    if (setpoint_echo == PENDING) {
      send(msg_setpoint.set(setpoint, 1), REQUEST_ECHO);
    }
    if (ts_switch_echo == PENDING)  {
      send(msg_status.set(ts_switch), REQUEST_ECHO);
    }
    timing_echo_request = millis();
  }

  /*
     Refresco del display y apagado tras un tiempo de inactividad del teclado.
     Cuando se apaga el display se asume la finalizacion de la entrada de datos
     mediante teclado y se envian al controlador.
  */
  if (oled.isEnabled()) {
    oled_refresh();
    if (millis() - timing_lcdbacklight >= BACKLIGHT_TIME) {
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
  oled.print("Out: ");
  if (power_out == FULL_POWER) {
    oled.print("100");
  } else if (power_out == ZERO_POWER) {
    oled.print("  0");
  } else {
    oled.print(" ");
    oled.print((int)((power_out * 100) / FULL_POWER));
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
      send(msg_setpoint.set(setpoint, 1), REQUEST_ECHO);
      setpoint_echo = PENDING;
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
      send(msg_status.set(ts_switch), REQUEST_ECHO);
      ts_switch_echo = PENDING;
    }
    saveState(EEPROM_V_STATUS, ts_switch);
    return true;
  }
  return false;
}

void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_HEAT) {
    if (message.isEcho())
      setpoint_echo = RECEIVED;
    else
      setSetpoint(message.getFloat(), false);
  } else if (message.type == V_STATUS) {
    if (message.isEcho())
      ts_switch_echo = RECEIVED;
    else
      setStatus(message.getBool(), false);
  }
}
