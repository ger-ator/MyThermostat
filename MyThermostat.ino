#define SN "MyThermostat"
#define SV "1.8"
// Enable debug prints to serial monitor
//#define MY_DEBUG
// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER
//#define MY_RFM69_ATC_MODE_DISABLED
//#define MY_RFM69_TX_POWER_DBM (20)
#define MY_RFM69_MAX_POWER_LEVEL_DBM (20)
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-75)
#define MY_SIGNAL_REPORT_ENABLED

//Wait gateway for 5sg
#define MY_TRANSPORT_WAIT_READY_MS 5000
// OTA Firmware update settings
#define MY_OTA_FIRMWARE_FEATURE
// Signing setup
#define MY_SIGNING_ATSHA204
#define MY_SIGNING_REQUEST_SIGNATURES

#define MY_SPLASH_SCREEN_DISABLED

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

typedef enum
{
  IN_HEATER_SENSOR,
  REMOTE_SENSOR
} T_mode;
T_mode room_temp_mode = IN_HEATER_SENSOR;

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
#define REFRESH_RATE_59MIN 3540000 //ms
#define TEMPMODE_FALLBACK 900000 //ms //15MIN

unsigned long timing_cycle;
unsigned long timing_dallas;
unsigned long timing_lcdbacklight;
unsigned long timing_temp_refresh;
unsigned long timing_sp_refresh;
unsigned long timing_ack_request;
unsigned long timing_tempmode_fallback;

/*
   Misc
*/
#define REQUEST_ECHO true
#define SP_INCDEC 0.5

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
  timing_dallas = timing_cycle = timing_tempmode_fallback = 0;
  timing_temp_refresh = timing_ack_request = timing_lcdbacklight = timing_sp_refresh = millis();

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
     Get local temperature sensors reading.
  */
  if (millis() - timing_dallas >= DALLAS_RATE) {
    if (room_temp_mode == IN_HEATER_SENSOR)
      room_temp = sensors.getTempC(room_sensor);
    safety_temp = sensors.getTempC(safety_sensor);
    sensors.requestTemperatures();
    timing_dallas = millis();
  }

  /*
     Fallback to internal sensor if no data received for X time.
  */
  if (millis() - timing_tempmode_fallback >= TEMPMODE_FALLBACK) {
    room_temp_mode = IN_HEATER_SENSOR;
  }

  /*
     Relay actuation.
  */
  if (millis() - timing_cycle >= DUTY_CYCLE) {
    digitalWrite(PIN_RELAY,
                 (ts_switch &&
                  safety_temp < 55 &&
                  room_temp < setpoint) ? HIGH : LOW);
    timing_cycle = millis();
  }

  /*
     Send room temperature to controller.
  */
  if (millis() - timing_temp_refresh >= REFRESH_RATE_13MIN) {
    send(msg_temp.set(room_temp, 1));
    timing_temp_refresh = millis();
  }
  /*
     Send setpoint to controller.
  */
  if (millis() - timing_sp_refresh >= REFRESH_RATE_59MIN) {
    send(msg_setpoint.set(setpoint, 1));
    timing_sp_refresh = millis();
  }

  /*
     Resend data that requested ack when ack is lost.
  */
  if (millis() - timing_ack_request >= REFRESH_RATE_2MIN) {
    if (setpoint_ack == PENDING) {
      send(msg_setpoint.set(setpoint, 1), REQUEST_ECHO);
    }
    if (ts_switch_ack == PENDING)  {
      send(msg_status.set(ts_switch), REQUEST_ECHO);
    }
    timing_ack_request = millis();
  }

  /*
     Display refresh and turn off when idle for BACKLIGHT_TIME ms.
     When display is turned off modified data is stored.
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
     Read the keypad.
     Further actions on event function.
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
  oled.print(room_temp, 1); oled.println(kp_ts_switch ? F("| ON") : F("|OFF"));
  oled.println(F("--------"));
  oled.print(F("SP: ")); oled.println(kp_setpoint, 1);
  oled.print(room_temp_mode == IN_HEATER_SENSOR ? F("I  ") : F("R  "));
  if (safety_temp > 55) oled.println(F("HI-HI"));
  else if (safety_temp > 50) oled.println(F("   HI"));
  else oled.println(F("   OK"));
}

/*
   Turn float into 4 byte and store in EEPROM.
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
   Load 4 bytes from EEPROM and turn to float.
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
      send(msg_status.set(ts_switch), REQUEST_ECHO);
      ts_switch_ack = PENDING;
    }
    saveState(EEPROM_V_STATUS, ts_switch);
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
  } else if (message.type == V_TEMP) {
    room_temp_mode = REMOTE_SENSOR;
    timing_tempmode_fallback = millis();
    room_temp = message.getFloat();
  }
}
