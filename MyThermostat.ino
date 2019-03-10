// Enable debug prints to serial monitor
//#define MY_DEBUG

//Dormitorio de invitados
#define MY_NODE_ID 1
//Salon
//#define MY_NODE_ID 2
//Dormitorio principal
//#define MY_NODE_ID 3
//Dormitorio de Javi
//#define MY_NODE_ID 4
//Dormitorio de Sergio
//#define MY_NODE_ID 5
//Pruebas
//#define MY_NODE_ID 9

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_IS_RFM69HW
#define MY_RFM69_NETWORKID 99

//Espera 5sg y entra al loop
#define MY_TRANSPORT_WAIT_READY_MS 5000

// OTA Firmware update settings
#define MY_OTA_FIRMWARE_FEATURE
#define OTA_WAIT_PERIOD 300
#define FIRMWARE_MAX_REQUESTS 2
#define MY_OTA_RETRY 2

// Signing setup
#define MY_SIGNING_ATSHA204
#define MY_SIGNING_REQUEST_SIGNATURES

#include <MySensors.h>

#define SN "MyThermostat"
#define SV "1.0"

#include <stdio.h>
#include <Keypad.h>
#include <SSD1306AsciiAvrI2c.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * Pines utilizados
 */
#define PIN_RELAY A0
#define PIN_TEMP  7

/*
 * Direcciones EEPROM para guardar estado
 */
#define EEPROM_V_SETPOINT 0
#define EEPROM_V_STATUS 4

/*
 * Controlador PID
 */
#define KP 10000
#define KI 5
#define KD 0
double sp, pv, out;
PID myPID(&pv, &out, &sp, KP, KI, KD, DIRECT);

/*
 *  Sonda temperatura DS18B20
 */
OneWire oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
//Despacho
DeviceAddress sondaControl = { 0x28, 0xFF, 0x16, 0x36, 0x86, 0x16, 0x04, 0x7D };
DeviceAddress sondaProteccion = { 0x28, 0x52, 0x47, 0x81, 0x0A, 0x00, 0x00, 0xDA };
//Salon
//DeviceAddress sondaControl = { 0x28, 0xFF, 0x56, 0xE3, 0x86, 0x16, 0x05, 0x4A };
//DeviceAddress sondaProteccion = { 0x10, 0xAB, 0x18, 0x3E, 0x02, 0x08, 0x00, 0x79 };
//Dormitorio Principal
//DeviceAddress sondaControl = { 0x28, 0xFF, 0x90, 0x99, 0x85, 0x16, 0x03, 0x7E };
//DeviceAddress sondaProteccion = { 0x10, 0x1F, 0x31, 0x3E, 0x02, 0x08, 0x00, 0xA9 };
//Dormitorio de Javi
//DeviceAddress sondaControl = { 0x28, 0xFF, 0xB1, 0x78, 0x71, 0x17, 0x03, 0x5A };
//DeviceAddress sondaProteccion = { 0x10, 0x6F, 0x07, 0x3E, 0x02, 0x08, 0x00, 0x29 };
//Dormitorio de Sergio
//DeviceAddress sondaControl = { 0x28, 0xFF, 0x2D, 0xC0, 0x71, 0x17, 0x03, 0x41 };
//DeviceAddress sondaProteccion = { 0x10, 0x7E, 0x09, 0x3E, 0x02, 0x08, 0x00, 0xE7 };
//Pasillo
//DeviceAddress sondaControl = { 0x28, 0xFF, 0x65, 0x0B, 0x80, 0x17, 0x04, 0x9E };
//DeviceAddress sondaProteccion = { 0x28, 0x0E, 0x4D, 0x81, 0x0A, 0x00, 0x00, 0x34 };
//Test
//DeviceAddress sondaControl = { 0x28, 0xFF, 0xC9, 0xA6, 0x71, 0x17, 0x03, 0x1D };
//DeviceAddress sondaProteccion = { 0x28, 0x52, 0x47, 0x81, 0x0A, 0x00, 0x00, 0xDA };
double tempProteccion;

/*
 * Display OLED I2C 0,96"
 */
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;
bool standby;

/*
 * Keypad de 2 filas y 2 columnas
 */
const byte ROWS = 2; //four rows
const byte COLS = 2; //three columns
byte rowPins[ROWS] = {5, 3};
byte colPins[COLS] = {6, 4};
char keys[ROWS][COLS] = {
  {'-','+'},
  {'M','O'}
};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
bool sp_changed;
bool ts_changed;

/*
 * S_HEATER
 */
MyMessage msg_setpoint(0, V_HVAC_SETPOINT_HEAT);
MyMessage msg_temp(0, V_TEMP);
MyMessage msg_status(0, V_STATUS);
bool ts_status = false; //true: ON - false: OFF
bool sp_ack_received = true;
bool ts_ack_received = true;
double temp_anterior;
/*
 * S_TEMP
 */
MyMessage msg_safetytemp(1, V_TEMP);

/*
 * Temporizado
 */
#define DUTY_CYCLE 10000 //ms
#define DALLAS_RATE 2000 //ms
#define BACKLIGHT_TIME 17100 //ms
#define REFRESH_RATE 300000 //ms //5min

unsigned long timing_cycle;
unsigned long timing_dallas;
unsigned long timing_lcdbacklight;
unsigned long timing_refresh;
unsigned long timing_long_refresh;

/*
 * Misc
 */
#define REQ_ACK 1
#define SP_INCDEC 0.5

void setup()
{
  /*
   * Restaurar ultimo estado
   */
  sp = loadFloat(EEPROM_V_SETPOINT);  
  ts_status = loadState(EEPROM_V_STATUS);
  if(isnan(sp) || (sp < 5.0) || (sp > 30.0)) sp = 20.0;
     
  /*  
   * Configurar controlador PID.
   * Salida: de 0-10000 ciclo de trabajo 10000ms.
   * Modo: AUTO/MAN en funcion del estado cargado.
   * SampleTime: 10sg: 
   */
  myPID.SetOutputLimits(0, DUTY_CYCLE);
  myPID.SetSampleTime(DUTY_CYCLE);
  myPID.SetMode(ts_status ? AUTOMATIC : MANUAL);

  /*
   * Configurar sensor Dallas y tomar primera lectura.
   */
  sensors.begin();
  sensors.setResolution(sondaControl, 12);
  sensors.setResolution(sondaProteccion, 9);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);
  pv = sensors.getTempC(sondaControl);
  tempProteccion = sensors.getTempC(sondaProteccion);
    
  /*
   * Iniciar pantalla
   */
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(font8x8);
  oled.set2X();
  standby = false;

  /*
   * Configurar pin de salida del rele
   */
  pinMode(PIN_RELAY, OUTPUT);

  /*
   * Keypad
   */
  keypad.addEventListener(keypadEvent);

  /*  
   * Envio del estado inicial al controlador.  
   */
   
  refresh_data(true);
  
  timing_cycle = millis();
  timing_dallas = 0;
  timing_refresh = millis();
  timing_long_refresh = millis();
  timing_lcdbacklight = millis();
}

void presentation()
{
	sendSketchInfo(SN, SV);
  present(0, S_HEATER);
  wait(100);
  present(1, S_TEMP);
}

void loop()
{
  /*
   * Lectura de los sensores de temperatura locales.
   */
  if ((unsigned long)(millis() - timing_dallas) >= DALLAS_RATE) {
    pv = sensors.getTempC(sondaControl);
    tempProteccion = sensors.getTempC(sondaProteccion);
    sensors.requestTemperatures();
    timing_dallas = millis();
  }

  /*
   * Evaluacion del lazo PID y actuacion rele.
   */
  myPID.Compute();   
  digitalWrite(PIN_RELAY, (ts_status && (tempProteccion < 60) && ((unsigned long)(millis() - timing_cycle) < out)) ? HIGH : LOW);
  if ((unsigned long)(millis() - timing_cycle) >= DUTY_CYCLE) timing_cycle = millis();

  /*
   * Envio de temperatura al controlador si ha
   * habido un cambio mayor a 0.1ºC
   */
  if((unsigned long)(millis() - timing_long_refresh) >= 10 * REFRESH_RATE) {
    refresh_data(true);
    timing_long_refresh = millis();
  }

  
  if((unsigned long)(millis() - timing_refresh) >= REFRESH_RATE) {
    refresh_data(false);
    timing_refresh = millis();
  }

  /*
   * Refresco del display y apagado tras un tiempo de inactividad
   * del teclado.
   * Cuando se apaga el display se asume la finalizacion de la entrada de datos 
   * mediante teclado y se envian al controlador.
   * Se realiza el salvado de datos en EEPROM aqui para minimizar el numero
   * de escrituras.
   */
  if (!standby) {
    update_screen();
    if ((unsigned long)(millis() - timing_lcdbacklight) >= BACKLIGHT_TIME) {
      display_off();
      if(sp_changed) {
        saveFloat(EEPROM_V_SETPOINT, sp);
        wait(100);
        send(msg_setpoint.set(sp, 1), REQ_ACK);
        sp_changed = false;
        sp_ack_received = false;     
      }
      if(ts_changed) {
        saveState(EEPROM_V_STATUS, ts_status);
        wait(100);
        send(msg_status.set(ts_status), REQ_ACK);
        ts_changed = false;
        ts_ack_received = false;
      }
    }
  }

  /*
   * Realizar lectura del keypad. 
   * No almaceno la lectura porque uso eventos
   */
  keypad.getKey();  
}

void keypadEvent(KeypadEvent key) {
  switch (keypad.getState()) {
    case PRESSED:
      timing_lcdbacklight = millis();
      if (standby) {
        oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
        standby = false;
        break;
      }      
      
      switch (key) {
        case '+': setSetpoint(sp + SP_INCDEC); break;
        case '-': setSetpoint(sp - SP_INCDEC); break;
        case 'M': break;
        case 'O': 
          setStatus(!ts_status); 
          ts_changed = true;
          break;
      }
      break;
    }
}

void update_screen (void) {
  oled.setCursor(0, 0);  
  oled.print(pv, 1); oled.println(ts_status ? "| ON" : "|OFF");
  if(tempProteccion > 55) {
    oled.println("-ALARMA-"); //Sacar un mensaje de alarma
  } else {
    oled.println("--------");
  }
  oled.print("SP: "); oled.println(sp, 1);
  int out_pcnt = 0;
  out_pcnt = (int)(out/100);
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
 * setSetpoint
 */
void setSetpoint (float setpoint) {
  if ((setpoint != sp) && (setpoint >= 5.0) && (setpoint <= 30.0)) {
    sp = setpoint;
    sp_changed = true;
  } 
}

/*
 * setStatus
 */
void setStatus (bool v_status) {
  if (v_status != ts_status) {
    ts_status = v_status;
    myPID.SetMode(ts_status ? AUTOMATIC : MANUAL);
    ts_changed = true;    
  }
}

void display_on (void) {
  oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
  standby = false;
  timing_lcdbacklight = millis();
}

void display_off (void) {
  oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
  standby = true;  
}

void refresh_data (bool force) {
  send(msg_safetytemp.set(tempProteccion, 1));
  if ((pv >= temp_anterior + 0.1) || (pv <= temp_anterior - 0.1) || (force)) {
    wait(100);
    send(msg_temp.set(pv, 1));
    temp_anterior = pv;
  }
  if ((!sp_ack_received) || (force)) {
    wait(100);
    send(msg_setpoint.set(sp, 1), REQ_ACK);
  }
  if ((!ts_ack_received) || (force)) {
    wait(100);
    send(msg_status.set(ts_status), REQ_ACK);
  }
}

void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_HEAT) {
    if (message.isAck()) {
      sp_ack_received = true;
    }
    else {
      display_on();
      setSetpoint(message.getFloat());
    }
  } else if (message.type == V_STATUS) {
    if (message.isAck()) {
      ts_ack_received = true;
    }
    else {
      display_on();
      setStatus(message.getBool());
    }
  } else if (message.type == V_TEMP) {
    //PENDIENTE GESTIONAR TEMPERATURA REMOTA
  } 
}
