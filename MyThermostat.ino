// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RF69_433MHZ

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

//Espera 5sg y entra al loop
#define MY_TRANSPORT_WAIT_READY_MS 5000

#include <MySensors.h>

#define SN "MyThermostat"
#define SV "2.0"

#include <stdio.h>
#include <Keypad.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * Pines utilizados
 */
#define PIN_RELAY 8
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
//Salon
//DeviceAddress sondaControl = { 0x28, 0xFF, 0x56, 0xE3, 0x86, 0x16, 0x05, 0x4A };
//DeviceAddress sondaProteccion = { 0x10, 0xAB, 0x18, 0x3E, 0x02, 0x08, 0x00, 0x79 };
//Despacho
//DeviceAddress sondaControl = { 0x28, 0xFF, 0x16, 0x36, 0x86, 0x16, 0x04, 0x7D };
//DeviceAddress sondaProteccion = { 0x10, 0xAE, 0xF8, 0x3D, 0x02, 0x08, 0x00, 0xC0 };
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
DeviceAddress sondaControl = { 0x28, 0xFF, 0x65, 0x0B, 0x80, 0x17, 0x04, 0x9E };
DeviceAddress sondaProteccion = { 0x28, 0xFF, 0xB7, 0x25, 0xB5, 0x16, 0x03, 0x8A };
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
byte rowPins[ROWS] = {4, 6};
byte colPins[COLS] = {3, 5};
char keys[ROWS][COLS] = {
  {'-','+'},
  {'M','O'}
};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

/*
 * S_HEATER
 */
bool ts_status = false; //true: ON - false: OFF
MyMessage msg_setpoint(0, V_HVAC_SETPOINT_HEAT);
MyMessage msg_temp(0, V_TEMP);
MyMessage msg_status(0, V_STATUS);
double temp_anterior;

/*
 * Temporizado
 */
#define DUTY_CYCLE 10000 //ms
#define DALLAS_RATE 2000 //ms
#define BACKLIGHT_TIME 37100 //ms
#define REFRESH_RATE 300000 //ms //5min

unsigned long timing_cycle;
unsigned long timing_dallas;
unsigned long timing_lcdbacklight;
unsigned long timing_refresh;

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
    
  /*
   * Iniciar pantalla
   */
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
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

  timing_cycle = millis();
  timing_dallas = 0;
  timing_refresh = millis();
  timing_lcdbacklight = millis();
}

void presentation()
{
	sendSketchInfo(SN, SV);
  //present(0, S_HEATER, "Radiador", REQ_ACK);
  present(0, S_HEATER);
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
  if((unsigned long)(millis() - timing_refresh) >= REFRESH_RATE) {
    if ((pv >= temp_anterior + 0.1) || (pv <= temp_anterior - 0.1)) {
      send(msg_temp.set(pv, 1));
      temp_anterior = pv;
    }    
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
      oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      standby = true;
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
        case 'O': setStatus(!ts_status); break;
      }
      break;
    }
}

void update_screen (void) {
  oled.setCursor(0, 0);  
  oled.print(pv, 1); oled.println(ts_status ? "| ON" : "|OFF");
  if(tempProteccion > 55) {
    oled.println("-ALARMA-"); //Sacar un mensaje de alarma
    timing_lcdbacklight = millis(); //No apagar pantalla para ver alarma
  } else {
    oled.println("--------");
  }
  oled.print("SP: "); oled.println(sp, 1);
  oled.print("Out: "); oled.print((int)(out/100)); oled.println(" ");
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
    saveFloat(EEPROM_V_SETPOINT, sp);
    send(msg_setpoint.set(sp, 1));
  }
}

/*
 * setStatus
 */
void setStatus (bool v_status) {
  if (v_status != ts_status) {
    ts_status = v_status;
    saveState(EEPROM_V_STATUS, ts_status);
    myPID.SetMode(ts_status ? AUTOMATIC : MANUAL);
    send(msg_status.set(ts_status));
  }
}

void receive(const MyMessage &message)
{
  if (message.type == V_HVAC_SETPOINT_HEAT) {
    setSetpoint(message.getFloat());
  } else if (message.type == V_STATUS) {
    setStatus(message.getBool());
  } else if (message.type == V_TEMP) {
    //PENDIENTE GESTIONAR TEMPERATURA REMOTA
  } 
}
