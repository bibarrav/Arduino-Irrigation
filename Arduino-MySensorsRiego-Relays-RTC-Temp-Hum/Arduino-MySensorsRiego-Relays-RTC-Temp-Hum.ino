// Enable debug prints to serial monitor
#define MY_DEBUG 
#define MY_NODE_ID 3
#define MY_PARENT_NODE_ID 1
// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
#define MY_RF24_PA_LEVEL RF24_PA_MAX // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH or RF24_PA_MAX
// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

/*NRF radio Pin configuration on Arduino MEGA
 * 50 --> MISO(7) --> 16
 * 51 --> MOSI(6) --> 15
 * 52 --> SCK(5) --> 14
 * 53 --> CSN(4) --> 46
 * 48 --> CE(3) --> 48
 * 3.3v --> VCC (2)
 * GND --> GND (1)
*/

// W5100 Ethernet module SPI enable (optional if using a shield/module that manages SPI_EN signal)
//#define MY_W5100_SPI_EN 4  

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// The W5100 ethernet module seems to have a hard time co-operate with 
// radio on the same spi bus.
#if !defined(MY_W5100_SPI_EN) && !defined(ARDUINO_ARCH_SAMD)
  #define MY_SOFTSPI
  #define MY_SOFT_SPI_SCK_PIN 14
  #define MY_SOFT_SPI_MISO_PIN 16
  #define MY_SOFT_SPI_MOSI_PIN 15
#endif  

// When W5100 is connected we have to move CE/CSN pins for NRF radio
#ifndef MY_RF24_CE_PIN 
  #define MY_RF24_CE_PIN 48
#endif
#ifndef MY_RF24_CS_PIN 
  #define MY_RF24_CS_PIN 46
#endif

#include <SPI.h>
#include <EEPROM.h>
#include <MySensors.h>
//#include "Bounce2.1/Bounce2.h"

//Bounce debouncer = Bounce(); 
//int oldValue=-1;

//#define RELAY_1  3  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
//#define NUMBER_OF_RELAYS 2 // Total number of attached relays
//#define RELAY_ON 1  // GPIO value to write to turn on attached relay
//#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

MyMessage RiegoZona1(1,V_STATUS);
MyMessage RiegoZona2(2,V_STATUS);
MyMessage RiegoZona3(3,V_STATUS);
MyMessage RiegoZona4(4,V_STATUS);
MyMessage SRelay5(5,V_STATUS);
MyMessage SRelay6(6,V_STATUS);
MyMessage EnchufesJardin(7,V_STATUS);
MyMessage PWRiego(8,V_STATUS);
MyMessage TEMP(9,V_TEMP);
MyMessage HUM(10,V_HUM);
MyMessage DuracionRiego(11,V_CUSTOM);
MyMessage ZonasRiego(12,V_CUSTOM);
MyMessage PruebaRiegoJardin(13, V_STATUS);
MyMessage ModoRiego(14, V_STATUS);
MyMessage TimeDate(15, V_TEXT);
MyMessage RiegoTimer1(16, V_TEXT);
MyMessage UsarRiegoTimer1(17, V_STATUS);
MyMessage RiegoTimer2(18, V_TEXT);
MyMessage UsarRiegoTimer2(19, V_STATUS);
MyMessage TEMP_RTC(20, V_TEMP);
MyMessage RiegoJardin(21, V_STATUS);
MyMessage RiegoDebug(22, V_STATUS);
MyMessage RiegoFlow(23, V_FLOW);
MyMessage RiegoVolume(23, V_VOLUME);
MyMessage RiegoCounter(23, V_VAR1);

// Sensor de Flujo
#define FLOW_SENSOR 3                  // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 1000              // Nummber of blinks per m3 of your meter (One rotation/liter)
#define SLEEP_MODE false               // flowvalue can only be reported when sleep mode is false.
#define MAX_FLOW 40                    // Max flow (l/min) value to report. This filters outliers.

double ppl = ((double)PULSE_FACTOR)/1000;        // Pulses per liter

volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double flow = 0;
bool pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double oldflow = 0;
double volume =0;
double oldvolume =0;
unsigned long lastSend =0;
unsigned long lastPulse =0;

unsigned long SEND_FREQUENCY = 30000; // Minimum time between send (in milliseconds). We don't want to spam the gateway.


#include <MenuSystem.h>

// Define Relay 8 channel pins
#define Relay1 31
#define Relay2 33
#define Relay3 35
#define Relay4 37
#define Relay5 39
#define Relay6 41
#define Relay7 43
#define Relay8 45 
#define RelayOn 0
#define RelayOff 1

// Program configuration variables
unsigned long TiempoRiego = 180000;
unsigned long TiempoRiegoPrueba;
int Zonas = 4;
/*
if (loadState(12)!=255) {
  Zonas = loadState(12);
} */
//bool Regar = true;

bool Regar = loadState(14)?true:false;
//bool UsarTimer1 = true;
bool UsarTimer1 = loadState(17)?true:false;
//bool UsarTimer2 = true;
bool UsarTimer2 = loadState(19)?true:false;

//Program Control Variables
unsigned long TiempoRiegoTemp;
bool Regando = false;
int ZonaRiego = 0;
bool PruebaRiego = false;
bool MostrarTempHum = false;

//LCD KeyPad
#define NUM_KEYS 5
#define Key_Pin 0 // LCD key pad analog pin
int adc_key_val[5] ={50,200,400,600,800};
int adc_key_in;
int key=-1;
int oldkey=-1;
bool PantallaEnReposo = false;
// Menu variables
bool ShowingMenu = false;
MenuSystem ms;
Menu mm("Menu Principal");
Menu mu1("Fecha-Hora");
MenuItem mu1_mi1("Ver Fecha-Hora");
MenuItem mu1_mi2("Edit Fecha-Hora");
Menu mu2("Opciones Riego");
MenuItem mu2_mi1("Ver Duracion");
MenuItem mu2_mi2("Edit Duracion");
MenuItem mu2_mi3("Ver Cant. Zonas");
MenuItem mu2_mi4("Edit Cant. Zonas");
MenuItem mu2_mi5("Prueba de Zonas");
MenuItem mu2_mi6("Ver Timer 1");
MenuItem mu2_mi7("Edit Timer 1");
MenuItem mu2_mi8("Usar Timer 1");
MenuItem mu2_mi9("Ver Timer 2");
MenuItem mu2_mi10("Edit Timer 2");
MenuItem mu2_mi11("Usar Timer 2");
MenuItem mu2_mi12("Ver Timer 3");
MenuItem mu2_mi13("Edit Timer 3");
MenuItem mu2_mi14("Usar Timer 3");
MenuItem mu2_mi15("Ver Timer 4");
MenuItem mu2_mi16("Edit Timer 4");
MenuItem mu2_mi17("Usar Timer 4");
Menu mu3("Modulos/Sensores");
MenuItem mu3_mi1("Ver Temperatura");
MenuItem mu3_mi2("Ver Humedad");
MenuItem mu3_mi3("Ver Temp Reloj");
MenuItem mu3_mi4("Ver Flujo");
MenuItem mu3_mi5("Ver Luminosidad");
Menu mu4("Sistema");
MenuItem m4_mi1("Ver Compilacion");
MenuItem m4_mi2("Ver Direccion IP");
MenuItem m4_mi3("Edit Direccion IP");
MenuItem m4_mi4("Modo Depuracion");
MenuItem m4_mi5("Ver Modo Riego");
MenuItem m4_mi6("Edit Modo Riego");
MenuItem m4_mi7("Ver MQTT-SVR IP");
MenuItem m4_mi8("Edit MQTT-SVR IP");
MenuItem m4_mi9("Ver MQTT-SVR PRT");
MenuItem m4_mi10("Edit MQTT-SVR PRT");
MenuItem m4_mi11("Reinicializar!");
// menuSelected:  State var that indicates that a menu item has been
//                selected. The plan is to update acordingly the LCD
//                if we want to use the display to other thing
//                like set up a variable (show on this example with time)
boolean menuSelected = false; // no menu item has been selected
boolean EditMode = false;// Menu-Item Data Entry (No update menu)
enum setMenuSelected_Type { NONE, DATETIME, TIMERA, TIMERB, ZONES, ZONESTIMER, IPADDRESS, MODE, MQTTIP, MQTTPORT }; // this enum is in case we want to expand this example
setMenuSelected_Type setMenu;
byte cursorPosition;
String setString;

boolean DEBUG = false;
unsigned long timeA;
unsigned long timeB;
unsigned long timeC;
unsigned long timeD;
unsigned long timeE;
  
// CONNECTIONS:
// DS3231 SDA --> SDA
// DS3231 SCL --> SCL
// DS3231 VCC --> 3.3v or 5v
// DS3231 GND --> GND

#if defined(ESP8266)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
#include <Wire.h>  // must be incuded here so that Arduino library object file references work
#include <RtcDS3231.h>

RtcDS3231 Rtc;
// Interrupt Pin Lookup Table
// (copied from Arduino Docs)
//
// CAUTION:  The interrupts are Arduino numbers NOT Atmel numbers
//   and may not match (example, Mega2560 int.4 is actually Atmel Int2)
//   this is only an issue if you plan to use the lower level interupt features
//
// Board           int.0    int.1   int.2   int.3   int.4   int.5
// ---------------------------------------------------------------
// Uno, Ethernet    2       3
// Mega2560         2       3       21      20     [19]      18 
// Leonardo         3       2       0       1       7

#define RtcSquareWavePin 19 // Mega2560
#define RtcSquareWaveInterrupt 4 // Mega2560

// marked volatile so interrupt can safely modify them and
// other code can safely read and modify them
volatile uint16_t interuptCount = 0;
volatile bool interuptFlag = false;

void InteruptServiceRoutine()
{
    // since this interupted any other running code,
    // don't do anything that takes long and especially avoid
    // any communications calls within this routine
    interuptCount++;
    interuptFlag = true;
}

#include <DHT.h>
#define DHTPIN 22     // what pin we're connected to
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

// include the library code:
#include <LiquidCrystal.h>
#define LCDBacklight 44 //LCD Pin Backlight control
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


void setup () {
    //Setup Serial speed
    Serial.begin(115200);

    //Flow Sensor
    // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
    pinMode(FLOW_SENSOR, INPUT_PULLUP);
    pulseCount = oldPulseCount = 0;

    // Fetch last known pulse count value from gw
    request(23, V_VAR1);
    lastSend = lastPulse = millis();

    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), onPulse, FALLING);
    
    // Setup all relay pins
    pinMode(Relay1, OUTPUT);
    pinMode(Relay2, OUTPUT);
    pinMode(Relay3, OUTPUT);
    pinMode(Relay4, OUTPUT);
    pinMode(Relay5, OUTPUT);
    pinMode(Relay6, OUTPUT);
    pinMode(Relay7, OUTPUT);
    pinMode(Relay8, OUTPUT);
    digitalWrite(Relay1, RelayOff);
    digitalWrite(Relay2, RelayOff);
    digitalWrite(Relay3, RelayOff);
    digitalWrite(Relay4, RelayOff);
    digitalWrite(Relay5, RelayOff);
    digitalWrite(Relay6, RelayOff);
    digitalWrite(Relay7, RelayOff);
    digitalWrite(Relay8, RelayOff);

    //Setup LCD 16x2
    pinMode(LCDBacklight, OUTPUT);
    digitalWrite(LCDBacklight, HIGH);
    timeB = millis();
    lcd.begin(16, 2);

    //Setup Digital Humidity & Temperature Sensor Module
    dht.begin();

    if (DEBUG) {
       Serial.println("Complilacion : " + String(__DATE__) + " / " + String(__TIME__));
    }
    lcd.setCursor(0,0);
    lcd.print("Ver ");
    lcd.print(__DATE__);
    lcd.setCursor(0,1);
    lcd.print("Compila.");
    lcd.print(__TIME__);

    //--------RTC SETUP ------------
    // set the interupt pin to input mode
    pinMode(RtcSquareWavePin, INPUT);
    Rtc.Begin();

    // if you are using ESP-01 then uncomment the line below to reset the pins to
    // the available pins for SDA, SCL
    // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL

    //RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Cuases:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing
        
        if (DEBUG) {
          Serial.println("RTC lost confidence in the DateTime!");  
        }
        
        // following line sets the RTC to the date & time this sketch was compiled
        // it will also reset the valid flag internally unless the Rtc device is
        // having an issue
        //Rtc.SetDateTime(compiled);
        
        // Publish SyncFlag to MQTT (Mosquitto)
        /* Sincronizacion MQTT de Reloj desde controlador a migrar a MySensors
        String pubStringTemp = "1";
        pubStringTemp.toCharArray(message_buff, pubStringTemp.length()+1);
        mqtt.publish("openHAB/Jardin/RiegoEstadoRelojSync", message_buff);
        */
    }

    if (!Rtc.GetIsRunning())
    {
        if (DEBUG) {
          Serial.println("RTC was not actively running, starting now"); 
        }
        Rtc.SetIsRunning(true);
    }

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    Rtc.Enable32kHzPin(false);
    //Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); 

    // throw away any old alarm state before we ran
    Rtc.LatchAlarmsTriggeredFlags();

    // setup external interupt 
    attachInterrupt(RtcSquareWaveInterrupt, InteruptServiceRoutine, FALLING);

    // Menu setup
    if (DEBUG) {  
      Serial.print("Inicializando Menu...");
    }

    mm.add_menu(&mu1);
    mu1.add_item(&mu1_mi1, &on_mu1_item1_selected);
    mu1.add_item(&mu1_mi2, &on_mu1_item2_selected);
    mm.add_menu(&mu2);
    mu2.add_item(&mu2_mi1, &on_mu2_item1_selected);
    mu2.add_item(&mu2_mi2, &on_mu2_item2_selected);
    mu2.add_item(&mu2_mi3, &on_mu2_item3_selected);
    mu2.add_item(&mu2_mi4, &on_mu2_item4_selected);
    mu2.add_item(&mu2_mi5, &on_mu2_item5_selected);
    mu2.add_item(&mu2_mi6, &on_mu2_item6_selected);
    mu2.add_item(&mu2_mi7, &on_mu2_item7_selected);
    mu2.add_item(&mu2_mi8, &on_mu2_item8_selected);
    mu2.add_item(&mu2_mi9, &on_mu2_item9_selected);     
    mu2.add_item(&mu2_mi10, &on_mu2_item10_selected); 
    mu2.add_item(&mu2_mi11, &on_mu2_item11_selected);
    mu2.add_item(&mu2_mi12, &on_mu2_item12_selected);
    mu2.add_item(&mu2_mi13, &on_mu2_item13_selected);
    mu2.add_item(&mu2_mi14, &on_mu2_item14_selected);
    mu2.add_item(&mu2_mi15, &on_mu2_item15_selected);
    mu2.add_item(&mu2_mi16, &on_mu2_item16_selected);
    mu2.add_item(&mu2_mi17, &on_mu2_item17_selected);
    mm.add_menu(&mu3);
    mu3.add_item(&mu3_mi1, &on_mu3_item1_selected);
    mu3.add_item(&mu3_mi2, &on_mu3_item2_selected);
    mu3.add_item(&mu3_mi3, &on_mu3_item3_selected);
    mu3.add_item(&mu3_mi4, &on_mu3_item4_selected);
    mu3.add_item(&mu3_mi5, &on_mu3_item5_selected);
    mm.add_menu(&mu4);
    mu4.add_item(&m4_mi1, &on_m4_item1_selected);
    mu4.add_item(&m4_mi2, &on_m4_item2_selected);
    mu4.add_item(&m4_mi3, &on_m4_item3_selected);
    mu4.add_item(&m4_mi4, &on_m4_item4_selected);
    mu4.add_item(&m4_mi5, &on_m4_item5_selected);
    mu4.add_item(&m4_mi6, &on_m4_item6_selected);
    mu4.add_item(&m4_mi7, &on_m4_item7_selected);
    mu4.add_item(&m4_mi8, &on_m4_item8_selected);
    mu4.add_item(&m4_mi9, &on_m4_item9_selected);
    mu4.add_item(&m4_mi10, &on_m4_item10_selected);
    mu4.add_item(&m4_mi11, &on_m4_item11_selected);
    ms.set_root_menu(&mm);
    
    if (DEBUG) {  
      Serial.println("Menu Inicializado.");
    }

    //Obtener/Enviar estados en Arduino a MySensors Gateway/Controlador
    printDateTime();
    Humedad();
    Relays(1);
    Relays(2);
    Relays(3);
    Relays(4);
    Relays(5);
    Relays(6);
    Relays(7);
    Relays(8);
    GetDuracionRiego();
    //GetIPControlador();
    GetZonasRiego();
    GetPruebaRiego();
    GetModoRiego();
    GetTimer1();
    GetTimer2();
    GetUsarTimer1();
    GetUsarTimer2();
    GetRegar();
    TemperaturaRtc();
    Temperatura();
    send(RiegoDebug.set(DEBUG));
}

//Cargar las variables con la informacion almacenada en la EEPROM
void before() { 
//  for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS;sensor++, pin++) {
    // Then set relay pins in output mode
//    pinMode(pin, OUTPUT);   
//    // Set relay to last known state (using eeprom storage) 
//    digitalWrite(pin, loadState(sensor)?RELAY_ON:RELAY_OFF);
//  if (DEBUG) {
    Serial.println("--> Leyendo valores desde EEPROM :");
    for (int i=0;i<=255;i++) {
      Serial.println("---> Sensor " + String(i) + " Valor:" + String(loadState(i)));
    }
    Serial.println("--> Lectura completada"); 
//  }
//  }
}

void presentation()
{   
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("RiegoJardin", "1.8");

  // Register all sensors to gw (they will be created as child devices)
  present(1, S_BINARY, "Riego Zona 1" );
  present(2, S_BINARY, "Riego Zona 2" );
  present(3, S_BINARY, "Riego Zona 3" );
  present(4, S_BINARY, "Riego Zona 4" );
  present(5, S_BINARY, "Relay5" );
  present(6, S_BINARY, "Relay6" );
  present(7, S_BINARY, "Enchufes Jardin" );
  present(8, S_BINARY, "Transformador Riego" );
  present(9, S_TEMP, "Temperatura Jardin" );
  present(10, S_HUM, "Humedad Jardin" );
  present(11, S_CUSTOM, "Duracion Riego");
  present(12, S_CUSTOM, "Zonas Riego");
  present(13, S_BINARY, "Prueba Riego");
  present(14, S_BINARY, "Modo Riego");
  present(15, S_INFO, "Fecha-Hora Riego");
  present(16, S_INFO, "Riego Timer1");
  present(17, S_BINARY, "Usar Riego Timer1");
  present(18, S_INFO, "Riego Timer2");
  present(19, S_BINARY, "Usar Riego Timer2");
  present(20, S_TEMP, "TemperaturaRTC Jardin");
  present(21, S_BINARY, "Riego Jardin");
  present(22, S_BINARY, "Riego DEBUG");
  present(23, S_WATER, "Flujo Riego");
}

void receive(const MyMessage &message) {
     // We only expect one type of message from controller. But we better check anyway.
     //if (message.type==V_LIGHT) {
     // Change relay state
     //digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
     // Store state in eeprom
     //saveState(message.sensor, message.getBool());
     
     switch(message.sensor) {
      case 5:
        //Process change state for Relay5
        if (message.type==V_STATUS) Relays(message.sensor, message.getInt());
        break;
      case 6:
        //Process change state for Relay6
        if (message.type==V_STATUS) Relays(message.sensor, message.getInt());
        break;
      case 7:
        //Process change state for Relay7 - Enchufes Jardin
        if (message.type==V_STATUS) Relays(message.sensor, message.getInt());
        break;
      case 11:
        //Update duraccion riego
        if (message.type==V_CUSTOM) SetDuracionRiego(message.getInt());
        break;
      case 12:
        //Update numero de zonas Riego
        if (message.type==V_CUSTOM) SetZonasRiego(message.getInt());
        break;
      case 13:
        //Update Prueba zonas Riego
        if (message.type==V_STATUS) SetPruebaRiego(message.getInt());
        break;
      case 14:
        //Update Modo Riego
        if (message.type==V_STATUS) SetModoRiego(message.getInt());
        break;
      case 15:
        //Update Fecha-Hora Rtc Riego
        if (message.type==V_TEXT) updateDateTime(message.getString());
        break;
      case 16:
        // Update Timer1 Riego
        if (message.type==V_TEXT) SetTimer1(message.getString());
        break;
      case 17:
        //Update Usar Timer1 Riego
        if (message.type==V_STATUS) SetUsarTimer1(message.getBool());
        break;
      case 18:
        // Update Timer2 Riego
        if (message.type==V_TEXT) SetTimer2(message.getString());
        break;
      case 19:
        //Update Usar Timer2 Riego
        if (message.type==V_STATUS) SetUsarTimer2(message.getBool());
        break;
      case 21:
        //Update Status Riego Jardin --> Regar/Dejar de regar
        if (message.type==V_STATUS) SetRegar(message.getBool());
        break;
      case 22:
        //Update DEBUG Flag
        if (message.type==V_STATUS) DEBUG=message.getBool();
        break;
      case 23:
        if (message.type==V_VAR1) {
          unsigned long gwPulseCount=message.getULong();
          pulseCount += gwPulseCount;
          flow=oldflow=0;
          Serial.print("Received last pulse count from gw:");
          Serial.println(pulseCount);
          pcReceived = true;
        }
       break;
     }
     /* message class Getter methods 
      char* getStream(char *buffer) const;
      char* getString(char *buffer) const;
      const char* getString() const;
      void* getCustom() const;
      bool getBool() const;
      uint8_t getByte() const;
      float getFloat() const;
      int16_t getInt() const;
      uint16_t getUInt() const;
      int32_t getLong() const;
      uint32_t getULong() const;
      // Getter for command type
      uint8_t getCommand() const;
      // Getter for ack-flag. True if this is an ack message.
      bool isAck() const;
      */
     
     // Write some debug info
     Serial.print("Incoming change for sensor:");
     Serial.print(message.sensor);
     Serial.print(", New status: ");
     Serial.println(message.getBool());
  // } 
}
/* Control por MQTT a Migrar a MySensors
    // Process Message
    if (String(topic).substring(0, String(topic).length()-1) == "openHAB/Jardin/RiegoZona"){
      Relays(String(String(topic).substring(String(topic).length()-1)).toInt(), String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoReloj"){
      updateDateTime(msgString);
    if  (String(topic) == "openHAB/Jardin/RiegoTimer1"){
      SetTimer1(msgString);
    if  (String(topic) == "openHAB/Jardin/RiegoTimer2"){
      SetTimer2(msgString);
    if  (String(topic) == "openHAB/Jardin/RiegoUsarTimer1"){
      SetUsarTimer1(String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoUsarTimer2"){
      SetUsarTimer2(String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoModo"){
      SetModoRiego(String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoPrueba"){
      SetPruebaRiego(String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoZonas"){
      SetZonasRiego(String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoDuraccion"){
      SetDuracionRiego(String(msgString).toInt());
    if  (String(topic) == "openHAB/Jardin/RiegoRegar"){
      SetRegar(String(msgString).toInt());
*/
// FlowMeter interrupt process
void onPulse() {
  if (!SLEEP_MODE) {
    unsigned long newBlink = micros();
    unsigned long interval = newBlink-lastBlink;

    if (interval!=0) {
      lastPulse = millis();
      if (interval<500000L) {
        // Sometimes we get interrupt on RISING,  500000 = 0.5sek debounce ( max 120 l/min)
        return;
      }
      flow = (60000000.0 /interval) / ppl;
    }
    lastBlink = newBlink;
  }
  pulseCount++;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop () {
    // FlowMeter loop process
    unsigned long currentTime = millis();
    // Only send values at a maximum frequency or woken up from sleep
    if (SLEEP_MODE || (currentTime - lastSend > SEND_FREQUENCY)) {
      lastSend=currentTime;
  
      if (!pcReceived) {
        //Last Pulsecount not yet received from controller, request it again
        request(23, V_VAR1);
        return;
      }
  
      if (!SLEEP_MODE && flow != oldflow) {
        oldflow = flow;
  
        Serial.print("Flujo de Riego (l/min):");
        Serial.println(flow);
  
        // Check that we dont get unresonable large flow value.
        // could hapen when long wraps or false interrupt triggered
        if (flow<((unsigned long)MAX_FLOW)) {
          send(RiegoFlow.set(flow, 2));                   // Send flow value to gw
        }
      }
  
      // No Pulse count received in 2min
      if(currentTime - lastPulse > 120000) {
        flow = 0;
      }
  
      // Pulse count has changed
      if ((pulseCount != oldPulseCount)||(!SLEEP_MODE)) {
        oldPulseCount = pulseCount;
  
        Serial.print("Flow Meter (pulsecount):");
        Serial.println(pulseCount);
  
        send(RiegoCounter.set(pulseCount));                  // Send  pulsecount value to gw in VAR1
  
        double volume = ((double)pulseCount/((double)PULSE_FACTOR));
        if ((volume != oldvolume)||(!SLEEP_MODE)) {
          oldvolume = volume;
  
          Serial.print("Volumen Riego (m3):");
          Serial.println(volume, 3);
  
          send(RiegoVolume.set(volume, 3));               // Send volume value to gw
        }
      }
    }
  
    // check RTC confidence
    if (!Rtc.IsDateTimeValid()  && DEBUG) {
        // Common Cuases:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }
    
    // publish & update reading every 30 seconds
    if (millis() > (timeA + 30000)) {
      timeA = millis();
      if (MostrarTempHum) {
        Humedad();
      }
      else {
        Temperatura();
      }
      MostrarTempHum = !MostrarTempHum;
      printDateTime();
      TemperaturaRtc();
      Relays(1);
      Relays(2);
      Relays(3);
      Relays(4);
      Relays(5);
      Relays(6);
      Relays(7);
      Relays(8);
      GetDuracionRiego();
      //GetIPControlador();
      GetZonasRiego();
      GetPruebaRiego();
      GetModoRiego();
      GetTimer1();
      GetTimer2();
      GetUsarTimer1();
      GetUsarTimer2();
      GetRegar();
      send(RiegoDebug.set(DEBUG));
     }

    // put lcd screen standby
    if (millis() > (timeB + 1800000)) {
      timeB = millis();
      PantallaEnReposo = true;
      lcd.noDisplay();
      digitalWrite(LCDBacklight, LOW);
    }
    
    // check schedule trigger (RTC Alarm interrupt) every 1 seconds
    if (millis() > (timeC + 1000)) {
      timeC = millis();
        if (Alarmed() && !Regando)
        {
            RiegoNormal();
            if (DEBUG) {
              Serial.println(">>Interupt Count: " + String(interuptCount) + "<<");
            }
        }
        else if (Regando) {
          RiegoNormal();
        }
    //    float flowsensor = digitalRead(FLOW_SENSOR);
    //    Serial.print("Flujo de Riego (l/min):");
    //    Serial.println(flowsensor);
    }

    // Check keypad press reading every 1 second and display menu when Menu (Right) button was pressed
    if ((millis() > (timeD + 1000)) || ShowingMenu) {
      timeD = millis();
      if (ShowingMenu){
        updateDisplay();
        serialHandler();
      }
      else {
        //delay(50);  // Expected to avoid bouncing pulsations
        wait(50);
        adc_key_in = analogRead(Key_Pin);    // Read the value of the pulsation
        key = get_key(adc_key_in);    // We get the button pressed
        if (key > 0 && PantallaEnReposo) {
          lcd.display();
          digitalWrite(LCDBacklight, HIGH);
          PantallaEnReposo = false;
        }
        if (key == 4) {
            timeE = millis();
            ShowingMenu = true;
            displayMenu();
            serialHandler();
        }
      }
    }

    // Exit menu for inactivity
    if ((millis() > (timeE + 10000)) && ShowingMenu) {
       lcd.clear();
       ShowingMenu = false;
       menuSelected = false;
       EditMode = false;
       setMenu = NONE;
       lcd.noBlink();
       ms.reset();
       printDateTime();
       Humedad();
    }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void GetDuracionRiego() {
  // Publish TiempoRiego to MySensors
    String pubStringTemp = String(TiempoRiego/1000);

    if (DEBUG) {
      Serial.println("--> Duracion Riego : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : DuracionRiego(11):" + pubStringTemp);
    }
    send(DuracionRiego.set(TiempoRiego/1000));
}

void SetDuracionRiego(int Tiempo) {
  TiempoRiego=Tiempo*1000;
  
  if (DEBUG) {
     Serial.println("--> Duracion Riego : " + String(TiempoRiego));
   }
   saveState(11, Tiempo);
   GetDuracionRiego();
}

void GetZonasRiego() {
    // Publish Zonas to MySensors
    String pubStringTemp = String(Zonas);

    if (DEBUG) {
      Serial.println("--> Zonas Riego : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : ZonasRiego(12):" + pubStringTemp);
    }
    send(ZonasRiego.set(Zonas));
}

void SetZonasRiego(int cZonas) {
  Zonas = cZonas;
  if (DEBUG) {
    Serial.println("--> Zonas : " + Zonas);
  }
  saveState(12, cZonas);
  GetZonasRiego();
}

void GetPruebaRiego() {
    // Publish Pruebas Riego to MySensors
    String pubStringTemp = String(PruebaRiego);

    if (DEBUG) {
      Serial.println("--> Prueba Riego : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : PruebaRiegoJardin(13):" + pubStringTemp);
    }
    send(PruebaRiegoJardin.set(PruebaRiego));
}

void SetPruebaRiego(bool Prueba) {
  if (Prueba && !Regando) {
      PruebaRiego = Prueba;
      TiempoRiegoPrueba = TiempoRiego;
      TiempoRiego = 20000;
      GetPruebaRiego();
      RiegoNormal();
  }
  
  if (DEBUG) {
     Serial.println("--> Prueba Riego : " + String(PruebaRiego));
  }
  
}

void GetModoRiego() {
    // Publish Modo Riego to MySensors
    String pubStringTemp = String(Regar);

    if (DEBUG) {
      Serial.println("--> Modo Riego : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : ModoRiego(14):" + pubStringTemp);
    }
    send(ModoRiego.set(Regar));
}

void SetModoRiego(bool ModoRiegoN) {
  Regar = ModoRiegoN;
   if (DEBUG) {
      Serial.println("--> Modo Riego : " + String(Regar));
   }
   saveState(14, ModoRiegoN);
   GetModoRiego();
}

void printDateTime() {
    #define countof(a) (sizeof(a) / sizeof(a[0]))
    RtcDateTime dt = Rtc.GetDateTime();
    char datestring[20];
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    if (!ShowingMenu){
      lcd.setCursor(0, 1);
      lcd.print(datestring);      
    }

    // Publish DateTime to MySensors
    String pubStringTemp = String(datestring);

    if (DEBUG) {
      Serial.println("--> Fecha y Hora : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : DateTime(15):" + pubStringTemp);
    }
    send(TimeDate.set(datestring));
}

void updateDateTime(String FechaHora) {
  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
  // Sample Update Date-Time Imput: 01/06/2016 01:30:22
  // Sample Update Date-Time Output: Jun 01 2016 01:30:22

  String TIME, DATE, Mes;
  RtcDateTime dt;
  Mes = FechaHora.substring(3,5);
  
  switch (Mes[1])
  {
    case '1':
      if (Mes[0] == '0') {
        Mes = "Jan"; }
      else {
        Mes = "Nov"; }
      break;
    case '2':
      if (Mes[0] == '0') {
        Mes = "Feb"; }
      else {
        Mes = "Dic"; }
      break;
    case '3':
      Mes = "Mar";
      break;
    case '4':
      Mes = "Apr";
      break;
    case '5':
      Mes = "May";
      break;
    case '6':
      Mes = "Jun";
      break;
    case '7':
      Mes = "Jul";
      break;
    case '8':
      Mes = "Aug";
      break;
    case '9':
      Mes = "Sep";
      break;
    case '0':
      Mes = "Oct";
      break;
  }

  TIME = FechaHora.substring(11,19);
  DATE = Mes + " " + FechaHora.substring(0,2) + " " + FechaHora.substring(6,10);

  if (DEBUG)
  {
    Serial.println("--> Update Date-Time: " + DATE + " / " + TIME);
  }

  char __TIME[9];
  char __DATE[12];
  TIME.toCharArray(__TIME, 9);
  DATE.toCharArray(__DATE, 12);
  
  dt = RtcDateTime(__DATE,__TIME);
  Rtc.SetDateTime(dt);
  printDateTime();
}

String GetTimer1() {
    #define countof(a) (sizeof(a) / sizeof(a[0]))
    DS3231AlarmOne dt = Rtc.GetAlarmOne();
    char datestring[20];
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u:%02u"),
            dt.Hour(),
            dt.Minute() );

    // Publish DateTime to MySensors
    String pubStringTemp = String(datestring);

    if (DEBUG) {
      Serial.println("--> Riego Timer 1 : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : RiegoTimer1(16):" + pubStringTemp);
    }
    send(RiegoTimer1.set(datestring));
    return pubStringTemp;
}

void SetTimer1(String TIME) {
    // flags define what calendar component to be checked against the current time in order
    // to trigger the alarm - see datasheet
    // A1M1 (seconds) (0 to enable, 1 to disable)
    // A1M2 (minutes) (0 to enable, 1 to disable)
    // A1M3 (hour)    (0 to enable, 1 to disable) 
    // A1M4 (day)     (0 to enable, 1 to disable)
    // DY/DT          (dayofweek == 1/dayofmonth == 0)
    //uint8_t flags[5] = { 0, 0, 0, 1, 1 };

    // Alarm 1 set to trigger hours, minutes, and seconds match
    TIME = TIME + ":00";
    char __TIME[9];
    TIME.toCharArray(__TIME, 9);
    RtcDateTime alarmTime1 = RtcDateTime(__DATE__, __TIME); // into the future

    DS3231AlarmOne alarm1(
            alarmTime1.Day(),
            alarmTime1.Hour(),
            alarmTime1.Minute(), 
            alarmTime1.Second(),
            DS3231AlarmOneControl_HoursMinutesSecondsMatch);
    Rtc.SetAlarmOne(alarm1);
    Rtc.LatchAlarmsTriggeredFlags();
    GetTimer1();
}

void GetUsarTimer1() {   
    // Publish UsarTimer1 to MySensors
    String pubStringTemp = String(UsarTimer1);

    if (DEBUG) {
      Serial.println("--> Usar Timer1 : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : UsarRiegoTimer1(17):" + pubStringTemp);
    }
    send(UsarRiegoTimer1.set(UsarTimer1));
}

void SetUsarTimer1(bool Estado) {
  UsarTimer1 = Estado;
  if (DEBUG) {
    Serial.println("--> Update Usar Timer1 : " + String(UsarTimer1));
  }
  saveState(17, Estado);
  GetUsarTimer1();
}

String GetTimer2() {
    #define countof(a) (sizeof(a) / sizeof(a[0]))
    DS3231AlarmTwo dt = Rtc.GetAlarmTwo();
    char datestring[20];
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u:%02u"),
            dt.Hour(),
            dt.Minute() );

    // Publish DateTime to MySensors
    String pubStringTemp = String(datestring);

    if (DEBUG) {
      Serial.println("--> Riego Timer 2 : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : RiegoTimer2(18):" + pubStringTemp);
    }
    send(RiegoTimer2.set(datestring));
    return pubStringTemp;
}

void SetTimer2(String TIME) {
    // flags define what calendar component to be checked against the current time in order
    // to trigger the alarm - see datasheet
    // A1M1 (seconds) (0 to enable, 1 to disable)
    // A1M2 (minutes) (0 to enable, 1 to disable)
    // A1M3 (hour)    (0 to enable, 1 to disable) 
    // A1M4 (day)     (0 to enable, 1 to disable)
    // DY/DT          (dayofweek == 1/dayofmonth == 0)
    //uint8_t flags[5] = { 0, 0, 0, 1, 1 };
    TIME = TIME + ":00";
    char __TIME[9];
    TIME.toCharArray(__TIME, 9);
    RtcDateTime alarmTime2 = RtcDateTime(__DATE__, __TIME); // into the future
    
    // Alarm 2 set to trigger at Hours and Minutes defined
    DS3231AlarmTwo alarm2(
            alarmTime2.Day(),
            alarmTime2.Hour(),
            alarmTime2.Minute(), 
            DS3231AlarmTwoControl_HoursMinutesMatch);
    Rtc.SetAlarmTwo(alarm2);
    Rtc.LatchAlarmsTriggeredFlags();
    GetTimer2();
}

void GetUsarTimer2() {   
    // Publish UsarTimer1 to MQTT (Mosquitto)
    String pubStringTemp = String(UsarTimer2);

    if (DEBUG) {
      Serial.println("--> Usar Timer2 : " + pubStringTemp);
      Serial.println("<-- Enviando Mensaje MySensors : UsarRiegoTimer2(19):" + pubStringTemp);
    } 
    send(UsarRiegoTimer2.set(UsarTimer2));
}

void SetUsarTimer2(bool Estado) {
  UsarTimer2 = Estado;
  if (DEBUG) {
    Serial.println("--> Update Usar Timer2 : " + String(UsarTimer2));
  }
  saveState(19, Estado);
  GetUsarTimer2();
}

void Relays(int RelayNumber, int RelayPower) {
    if (!ShowingMenu){
      lcd.setCursor(0, 1);
      lcd.print("Riego Zona" + String(RelayNumber));
    }
    
    if (RelayPower == RelayOn)
    {
      //No activar zonas de riego en paralelo, solo una a la vez!!!
      if ((Relays(11) == RelayOff && Relays(12) == RelayOff && Relays(13) == RelayOff && Relays(14) == RelayOff) || (RelayNumber != 1 && RelayNumber != 2 && RelayNumber != 3 && RelayNumber != 4)) {
       if (!ShowingMenu){
          lcd.print(": On  ");
        }
        digitalWrite(RelayNumber+RelayNumber+29, RelayPower);
      }
      else {
        if (!ShowingMenu){
          lcd.print(": --  ");
        }
      }
    }
    else
    {
      if (!ShowingMenu){
        lcd.print(": Off ");
      }
      digitalWrite(RelayNumber+RelayNumber+29, RelayPower);
    }
    //Revisar si se requiere guardar el estado de los relays al reiniciar...
    Relays(RelayNumber);
}

int Relays(int RelayNumber) {
    bool SendStatus = true;
    int RelayPowerState;

    if (RelayNumber > 10){
      SendStatus = false;
      RelayNumber = RelayNumber -10;
    }
    RelayPowerState = digitalRead(RelayNumber+RelayNumber+29);
 
   if (SendStatus) {
      switch(RelayNumber) {
        case 1:
          send(RiegoZona1.set(RelayPowerState));
          break;
        case 2:
          send(RiegoZona2.set(RelayPowerState));
          break;
        case 3:
          send(RiegoZona3.set(RelayPowerState));
          break;
        case 4:
          send(RiegoZona4.set(RelayPowerState));
          break;
        case 5:
          send(SRelay5.set(RelayPowerState));
          break;
        case 6:
          send(SRelay6.set(RelayPowerState));
          break;
        case 7:
          send(EnchufesJardin.set(RelayPowerState));
          break;
        case 8:
          send(PWRiego.set(RelayPowerState));
          break;         
      }
     
    }
    if (DEBUG) {
        Serial.println("--> Estado Relay" + String(RelayNumber) + " Power State : " + String(RelayPowerState));   
        Serial.println("<-- Enviando Mensaje MySensors: Relay" +String(RelayNumber) + " Power State : " + String(RelayPowerState)); 
   }
   return RelayPowerState;
}

void Humedad() { 
    // Read Humidity from module    
    float h = dht.readHumidity();

    // Print Temperature to LCD
    if (!ShowingMenu){
      lcd.setCursor(0, 0);
      lcd.print("Humedad Rel: "); 
      lcd.print((int) h);
      lcd.print("%       ");
    }
    //delay(2000);

    // Publish Humidity to MySensors
    String pubStringTemp = String(int(h));

    if (DEBUG){
      Serial.println("--> Humedad : " + pubStringTemp + "% " );
      Serial.println("<-- Enviando Mensaje MySensors: HUM(10):" + pubStringTemp);
    }
    send(HUM.set(h,1));
}

void Temperatura() {
    float t = dht.readTemperature();

    if (!ShowingMenu){
      lcd.setCursor(0, 0);
      lcd.print("Temperatura: " + String((int) t) + "C"); 
    }
    //delay(2000);

    // publish Temperature to MySensors
    String pubStringTemp = String(int(t));

    // Print Temperature to Serial
    if (DEBUG) {
      Serial.println("--> Temperatura : " + pubStringTemp + "ºC " );
      Serial.println("<-- Enviando Mensaje MySensors: TEMP(9):" + pubStringTemp);
    }
    send(TEMP.set(t,1));
}

void TemperaturaRtc() {
    //  Temperatura de modulo RTC
    RtcTemperature t = Rtc.GetTemperature();
    //lcd.setCursor(0, 0);
    //lcd.print("Temperatura Rtc: " && (int) t && "C"); 
    
    // publish Temperature to MySensors
    String pubStringTemp = (String)(int)t.AsFloat();

    if (DEBUG) {
      Serial.println("--> Temperatura Rtc: " + pubStringTemp + "ºC " );
      Serial.println("<-- Enviando Mensaje MQTT: TEMP_RTC(20):" + pubStringTemp);
    }
    send(TEMP_RTC.set(t.AsFloat(),1));
}

bool Alarmed() {
    bool wasAlarmed = false;
    if (interuptFlag)  // check our flag that gets sets in the interupt
    {
        interuptFlag = false; // reset the flag
        // this gives us which alarms triggered and
        // then allows for others to trigger again
        DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags();
        if (Regar) {
          if (flag == DS3231AlarmFlag_Alarm1 && UsarTimer1)
          {
            wasAlarmed = true;
            if (DEBUG){
              Serial.println();
              Serial.println("-->TIMER 1 - ACTIVADO!!!");
              Serial.println(); }
          }
          else if (flag == DS3231AlarmFlag_Alarm2 && UsarTimer2)
          {
            wasAlarmed = true;
            if (DEBUG) {
              Serial.println();
              Serial.println("-->TIMER 2 - ACTIVADO!!!");
              Serial.println(); }
          }
          else if (flag == DS3231AlarmFlag_AlarmBoth && UsarTimer1 && UsarTimer2) 
          {
            wasAlarmed = true;
            if (DEBUG) {
              Serial.println();
              Serial.println("-->TIMER 1 Y 2 - ACTIVADO!!!");
              Serial.println(); }
          }
        }
    }
    return wasAlarmed;
}

void RiegoNormal() {
  if (DEBUG){
    Serial.println("Procesando riego...");
  }
  if (Regando){
    // Revisar tiempo de riego
    if (millis() > (TiempoRiegoTemp + TiempoRiego)) {
      if (DEBUG) {
        Serial.println("Cambiando Zona de Riego...");
      }
      TiempoRiegoTemp = millis();
      Relays(ZonaRiego, RelayOff);
      if (ZonaRiego < Zonas) {
        ZonaRiego++;
        wait(500);
        Relays(ZonaRiego, RelayOn);
        if (Relays(ZonaRiego) == RelayOn) {
          if (DEBUG) {
            Serial.println("Cambiando Zona de Riego...OK.");          
          }
        }
        else {
          if (DEBUG) {
            Serial.println("Cambiando Zona de Riego...ERROR.");          
          }        
        }
      }
      else {
        if (!ShowingMenu) {
          lcd.setCursor(0,0);
          lcd.print("Ciclo Terminado   "); 
        }
        if (DEBUG) {
          Serial.println("Ciclo de Riego Terminado.");
        }
        ZonaRiego = 0;
        Regando = false;
        Relays(8,RelayOff); //Desactivar tranformador riego (220v a 24v AC)
        GetRegar();
        if (PruebaRiego) {
          TiempoRiego = TiempoRiegoPrueba;
          PruebaRiego = false;
          GetPruebaRiego();
        }
      }
    }
    else {
      if (DEBUG) {
        Serial.println("Verificando Zona de Riego...");
      }
      if (Relays(ZonaRiego + 10) == RelayOn) {
        if (DEBUG) {
          Serial.println("Verificando Zona de Riego...Encendido.");
        }
      }
      else {
        if (DEBUG) {
          Serial.println("Verificando Zona de Riego...Apagado, encendiendo...");
        }              
        Relays(ZonaRiego, RelayOn);
        if (Relays(ZonaRiego + 10) == RelayOn) {
          if (DEBUG) {
            Serial.println("Verificando Zona de Riego...Apagado, encendiendo...OK.");
          }
        }
        else {
          if (DEBUG) {
            Serial.println("Verificando Zona de Riego...Apagado, encendiendo...ERROR.");
          }        
        }
      }     
    }
  }
  else {
      if (!ShowingMenu) {
        lcd.setCursor(0,0);
        lcd.print("Iniciando ciclo!     "); 
      }
      if (DEBUG) {
        Serial.println("Iniciando ciclo de riego...");
      }
      Relays(8,RelayOn); //Activar transformador Riego (220v a 24v AC)
      TiempoRiegoTemp = millis();
      Regando = true;
      GetRegar();
      ZonaRiego = 1;
      Relays(1,RelayOn);
      if (Relays(1) == RelayOn) {
        if (DEBUG) {
          Serial.println("Iniciando ciclo de riego...OK.");          
        }
      }
      else {
        if (DEBUG) {
          Serial.println("Iniciando ciclo de riego...ERROR.");          
        }        
      }
  }
}

void RiegoSmart(int Duracion) {
  
}

void GetRegar() {
 // Publish Estado Riego to MySensors
    String pubStringTemp = String(Regando);

    if (DEBUG) {
      Serial.println("--> Estado Regar : " + pubStringTemp);
      Serial.println("<-- Enviando Mensage MySensors: RiegoJardin(21):" + pubStringTemp);
    }
    send(RiegoJardin.set(Regando));
}

void SetRegar(bool Estado) {
  if (!Regando && Estado) {
    if (DEBUG) {
      Serial.println("--> Estado Regar : A Regar!!!");
    }
    RiegoNormal();
  }
  else if (Regando && !Estado){
    if (DEBUG) {
      Serial.println("--> Estado Regar : Dejar de Regar!!!");
    }
    Relays(ZonaRiego, RelayOff);
    ZonaRiego = 0;
    Regando = false;
    Relays(8,RelayOff); //Desactivar transformador Riego (220v a 24v AC)
    if (PruebaRiego) {
          TiempoRiego = TiempoRiegoPrueba;
          PruebaRiego = false;
          GetPruebaRiego();
        }
    GetRegar();
  }
}

void serialHandler() {
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  wait(100);
  adc_key_in = analogRead(Key_Pin);    // Read the value of the pulsation
  key = get_key(adc_key_in);    // We get the button pressed
  wait(100);
  
  if (key !=-1)   // if keypress is detected
  {
    timeE = millis();
    if (key == 1){
      if(!menuSelected) {
          ms.prev();
          displayMenu();
      }
      else {
        switch (setMenu)
        {
        case DATETIME:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case TIMERA:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case TIMERB:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case ZONES:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<56)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case ZONESTIMER:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case IPADDRESS:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case MQTTIP:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case MQTTPORT:
          if(setString[cursorPosition]>47 && setString[cursorPosition]<57)
            setString[cursorPosition]++;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        }
      }
      if (DEBUG) {
          Serial.println("--> KeyPress : key-Up");
       }
    }
    if (key == 2)   { 
      if(!menuSelected) {
          ms.next();
          displayMenu();
      }
      else {
        switch (setMenu)
        {
        case DATETIME:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case TIMERA:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case TIMERB:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case ZONES:
          if(setString[cursorPosition]>49 && setString[cursorPosition]<57)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case ZONESTIMER:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case IPADDRESS:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case MQTTIP:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        case MQTTPORT:
          if(setString[cursorPosition]>48 && setString[cursorPosition]<58)
            setString[cursorPosition]--;
          lcd.write(setString[cursorPosition]);
          lcd.setCursor(cursorPosition,1);
          break;
        }
      }
      if (DEBUG) {
          Serial.println("--> KeyPress : key-Down");
      }
    }
    if (key == 3){
      if(!menuSelected) {
          ms.back();
          displayMenu();
      }
      else {
        if (DEBUG) {
            Serial.println("--> KeyPress : key-Left");
           // Serial.println(key);
        }
        switch (setMenu)
        {
          case DATETIME:
            if(cursorPosition>0) {
              lcd.setCursor(--cursorPosition,1);
            }
          case TIMERA:
            if(cursorPosition>12) {
              lcd.setCursor(--cursorPosition,1);
            }
          case TIMERB:
            if(cursorPosition>12) {
              lcd.setCursor(--cursorPosition,1);
            }
          case ZONES:
            if(cursorPosition>15) {
              lcd.setCursor(--cursorPosition,1);
            }
          case ZONESTIMER:
            if(cursorPosition>11) {
              lcd.setCursor(--cursorPosition,1);
            }
          case IPADDRESS:
            if(cursorPosition>0) {
              lcd.setCursor(--cursorPosition,1);
            }
          case MQTTIP:
            if(cursorPosition>0) {
              lcd.setCursor(--cursorPosition,1);
            }
          case MQTTPORT:
            if(cursorPosition>11) {
              lcd.setCursor(--cursorPosition,1);
            }
        }
      }
    }
    if (key == 4) {
      if(!menuSelected) {
          ms.select();
          displayMenu();
      }
      else {
        switch (setMenu)
        {
          case DATETIME:
          {
            setString = setString + ":00";
            updateDateTime(setString);
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("Fecha-Hora OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case TIMERA:
          {
            SetTimer1(setString.substring(10,16));
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("Hora Timer1 OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case TIMERB:
          {
            SetTimer2(setString.substring(10,16));
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("Hora Timer2 OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case ZONES:
          {
            Zonas = (setString.substring(14,16)).toInt();
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("Cant. Zonas OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case ZONESTIMER:
          {
            String setString_Temp = setString.substring(11,16);
            char __setString_Temp[5];
            setString_Temp.toCharArray(__setString_Temp, 5);
            TiempoRiego = long(__setString_Temp);
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("Duracion OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case IPADDRESS:
          {
            //char __setString_Temp[15];
            //setString.toCharArray(__setString_Temp, 15);
            //ip(__setString_Temp);
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("   IP OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case MQTTIP:
          {
            //SetTimer2(setString.substring(11,16));
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("MQTT-SVR IP OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
          case MQTTPORT:
          {
            //SetTimer2(setString.substring(11,16));
            EditMode = false;
            lcd.setCursor(0,1);
            lcd.print("MQTT-SVR PORT OK!      ");
            menuSelected = false;
            setMenu = NONE;
            lcd.noBlink();
            wait(1500);
            //delay(1500);
            displayMenu();
            break;
          }
        }
      }
      if (DEBUG) {
          Serial.println("--> KeyPress : key-Select");
         // Serial.println(key);
      }
    }
    if (key == 0) {
      if(!menuSelected) {
      // serialPrintHelp();
      }
      else if(cursorPosition<15) {
        lcd.setCursor(++cursorPosition,1);
      }
      if (DEBUG) {
          Serial.println("--> KeyPress : key-Right");
      }
    }
  }
}

// Convert the analog value read in a number of button pressed
int get_key(unsigned int input) {
  int k;
  for (k = 0; k < NUM_KEYS; k++)  {
    if (input < adc_key_val[k])    {
      return k;
    }
  }
  if (k >= NUM_KEYS) k = -1;  // Error in reading.
  return k;
}

void displayMenu() {
  lcd.clear();
  lcd.setCursor(0,0);
  // Display the menu
  Menu const* cp_menu = ms.get_current_menu();

  //lcd.print("Current menu name: ");
  lcd.print(cp_menu->get_name());
  lcd.setCursor(0,1);
  lcd.print(cp_menu->get_selected()->get_name());
}

void updateDisplay() {
  if(menuSelected && EditMode) {
    EditMode = false;

    switch(setMenu) {
      case DATETIME:
      {
        //lcd.setCursor(0,0);
        //lcd.print("Edit Fecha-Hora ");
        #define countof(a) (sizeof(a) / sizeof(a[0]))
        RtcDateTime dt = Rtc.GetDateTime();
        char datestring[20];
        snprintf_P(datestring, 
                  countof(datestring),
                  PSTR("%02u/%02u/%04u %02u:%02u"),
                  dt.Day(),
                  dt.Month(),
                  dt.Year(),
                  dt.Hour(),
                  dt.Minute() );
        lcd.setCursor(0, 1);
        lcd.print(datestring);
        setString = datestring;
        cursorPosition = 0;
        lcd.setCursor(0, 1);
        lcd.blink();
        break;
      }
      case TIMERA:
      {
        //lcd.setCursor(0,0);
        //lcd.print("Edit Timer1     ");
        String timerstring = "   Hora : " + GetTimer1();
        lcd.setCursor(0, 1);
        lcd.print(timerstring);
        setString = timerstring;
        cursorPosition = 10;
        lcd.setCursor(10, 1);
        lcd.blink();
        break;
      }
      case TIMERB:
      {
        //lcd.setCursor(0,0);
        //lcd.print("Edit Timer2     ");
        String timerstring = "   Hora : " + GetTimer2();
        lcd.setCursor(0, 1);
        lcd.print(timerstring);
        setString = timerstring;
        cursorPosition = 10;
        lcd.setCursor(10, 1);
        lcd.blink();
        break;
      }
      case ZONES:
      {
        //lcd.setCursor(0,0);
        //lcd.print("Edit Timer2     ");
        String timerstring = " Cant. Zonas : " + Zonas;
        lcd.setCursor(0, 1);
        lcd.print(timerstring);
        setString = timerstring;
        cursorPosition = 15;
        lcd.setCursor(15, 1);
        lcd.blink();
        break;
      }
      case ZONESTIMER:
      {
        //lcd.setCursor(0,0);
        //lcd.print("Edit Timer2     ");
        String timerstring = " Miliseg : " + TiempoRiego;
        lcd.setCursor(0, 1);
        lcd.print(timerstring);
        setString = timerstring;
        cursorPosition = 11;
        lcd.setCursor(11, 1);
        lcd.blink();
        break;
      }
      // add new cases as you add set menus (don't forget to add the corresponding enum)

    }
  }
}

// Menu callback function
// In this example all menu items use the same callback.

// Menu 1
void on_mu1_item1_selected(MenuItem* p_menu_item)
{ //Ver Fecha y Hora
    lcd.setCursor(0,0);
    lcd.print("Fecha y Hora    ");
    #define countof(a) (sizeof(a) / sizeof(a[0]))
    RtcDateTime dt = Rtc.GetDateTime();
    char datestring[20];
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
      lcd.setCursor(0, 1);
      lcd.print(datestring);     
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu1_item2_selected(MenuItem* p_menu_item)
{ //Editar Fecha y Hora
  setMenu = DATETIME;
  menuSelected = true;
  EditMode = true;
}

/* Menu 2
MenuItem mu2_mi1("Ver Duracion");
MenuItem mu2_mi2("Edit Duracion");
MenuItem mu2_mi3("Ver Cant. Zonas");
MenuItem mu2_mi4("Edit Cant. Zonas");
MenuItem mu2_mi5("Prueba de Zonas");
MenuItem mu2_mi6("Ver Timer 1");
MenuItem mu2_mi7("Edit Timer 1");
MenuItem mu2_mi8("Ver Timer 2");
MenuItem mu2_mi9("Edit Timer 2");
*/
void on_mu2_item1_selected(MenuItem* p_menu_item)
{ //Ver Duracion
  lcd.setCursor(0,0);
  lcd.print("Duracion         ");
  lcd.setCursor(0,1);
  lcd.print("Miliseg : ");
  lcd.print(TiempoRiego);
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item2_selected(MenuItem* p_menu_item)
{ //Edit Duracion
  setMenu = ZONESTIMER;
  menuSelected = true;
  EditMode = true;
}

void on_mu2_item3_selected(MenuItem* p_menu_item)
{ //Ver Cant. Zonas
  lcd.setCursor(0,0);
  lcd.print("Cant. Zonas      ");  
  lcd.setCursor(0,1);
  lcd.print("               ");
  lcd.print(Zonas);
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item4_selected(MenuItem* p_menu_item)
{ //Edit Cant. Zonas
  setMenu = ZONES;
  menuSelected = true;
  EditMode = true;
}

void on_mu2_item5_selected(MenuItem* p_menu_item)
{ //Prueba de Zonas
  lcd.setCursor(0,1);
  if (!PruebaRiego && !Regando)
  {
     lcd.print("--> Iniciando!!!");
     SetPruebaRiego(true);
  }
  else {
    lcd.print("--> Ocupado!!!  ");
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item6_selected(MenuItem* p_menu_item)
{ //Ver TIMER1
  String Timer1 = GetTimer1();
  lcd.setCursor(0,0);
  lcd.print("Timer 1          ");
  lcd.setCursor(0,1);
  lcd.print("           ");
  lcd.print(Timer1);
  if (DEBUG) {
    Serial.print("--> Menu Ver Timer 1 : " + Timer1);
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item7_selected(MenuItem* p_menu_item)
{ //Editar TIMER1
  setMenu = TIMERA;
  menuSelected = true;
  EditMode = true;
}

void on_mu2_item8_selected(MenuItem* p_menu_item)
{ //Usar TIMER1
  lcd.setCursor(0,0);
  lcd.print("Usar Timer 1     ");
  lcd.setCursor(0,1);
  UsarTimer1 = !UsarTimer1;
  if (UsarTimer1) {
    lcd.print("--> Encendido!  ");
  }
  else {
    lcd.print("--> Apagado!    ");
  }
  if (DEBUG) {
    Serial.print("--> Menu Usar Timer 1 : " + String(UsarTimer1));
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item9_selected(MenuItem* p_menu_item)
{ //Ver TIMER2
  String Timer2 = GetTimer2();
  lcd.setCursor(0,0);
  lcd.print("Timer 2          ");
  lcd.setCursor(0,1);
  lcd.print("           ");
  lcd.print(Timer2);
  if (DEBUG) {
    Serial.print("--> Menu Ver Timer 2 : " + String(Timer2));
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item10_selected(MenuItem* p_menu_item)
{ //Editar TIMER2
  setMenu = TIMERB;
  menuSelected = true;
  EditMode = true;
}

void on_mu2_item11_selected(MenuItem* p_menu_item)
{ //Usar TIMER2
  lcd.setCursor(0,0);
  lcd.print("Usar Timer 2     ");
  lcd.setCursor(0,1);
  UsarTimer2 = !UsarTimer2;
  if (UsarTimer2) {
    lcd.print("--> Encendido!  ");
  }
  else {
    lcd.print("--> Apagado!    ");
  }
  if (DEBUG) {
    Serial.print("--> Menu Usar Timer 2 : " + String(UsarTimer2));
  }  
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item12_selected(MenuItem* p_menu_item)
{ //Ver TIMER3
//  String Timer3 = GetTimer3();
  lcd.setCursor(0,0);
  lcd.print("Timer 3          ");
  lcd.setCursor(0,1);
  lcd.print("           ");
//  lcd.print(Timer3);
  if (DEBUG) {
//    Serial.print("-->Timer 3 : " + Timer3);
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item13_selected(MenuItem* p_menu_item)
{ //Editar TIMER3
//  setMenu = TIMERC;
  menuSelected = true;
  EditMode = true;
}

void on_mu2_item14_selected(MenuItem* p_menu_item)
{ //Usar TIMER3
  lcd.setCursor(0,0);
  lcd.print("Usar Timer 3     ");
  lcd.setCursor(0,1);
//  UsarTimer3 = !UsarTimer3;
/*  if (UsarTimer3) {
    lcd.print("--> Encendido!  ");
  }
  else {
    lcd.print("--> Apagado!    ");
  } */
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item15_selected(MenuItem* p_menu_item)
{ //Ver TIMER4
//  String Timer4 = GetTimer4();
  lcd.setCursor(0,0);
  lcd.print("Timer 4          ");
  lcd.setCursor(0,1);
  lcd.print("           ");
//  lcd.print(Timer4);
  if (DEBUG) {
//    Serial.print("-->Timer 4 : " + Timer4);
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu2_item16_selected(MenuItem* p_menu_item)
{ //Editar TIMER4
//  setMenu = TIMERD;
  menuSelected = true;
  EditMode = true;
}

void on_mu2_item17_selected(MenuItem* p_menu_item)
{ //Usar TIMER4
  lcd.setCursor(0,0);
  lcd.print("Usar Timer 4     ");
  lcd.setCursor(0,1);
//  UsarTimer4 = !UsarTimer4;
/*  if (UsarTimer2) {
    lcd.print("--> Encendido!  ");
  }
  else {
    lcd.print("--> Apagado!    ");
  } */
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

/* Menu 3
MenuItem mu3_mi1("Ver Temperatura");
MenuItem mu3_mi2("Ver Humedad");
MenuItem mu3_mi3("Ver Temp Reloj");
MenuItem mu3_mi4("Ver Flujo");
MenuItem mu3_mi5("Ver Luminosidad");
*/
void on_mu3_item1_selected(MenuItem* p_menu_item)
{ //Ver Temperatura
  lcd.setCursor(0,1);
  lcd.print("MU3-Item1 Select  ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu3_item2_selected(MenuItem* p_menu_item)
{ //Ver Humedad
  lcd.setCursor(0,1);
  lcd.print("MU3-Item2 Select  ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu3_item3_selected(MenuItem* p_menu_item)
{ //Ver Temp Reloj
  lcd.setCursor(0,1);
  lcd.print("MU3-Item3 Select  ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu3_item4_selected(MenuItem* p_menu_item)
{ //Ver Flujo
  lcd.setCursor(0,1);
  lcd.print("MU3-Item4 Select  ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_mu3_item5_selected(MenuItem* p_menu_item)
{ //Ver Luminosidad
  lcd.setCursor(0,1);
  lcd.print("MU3-Item5 Select  ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

//Menu 4
void on_m4_item1_selected(MenuItem* p_menu_item)
{ //Ver Fecha/Hora Compilacion
    lcd.setCursor(0,0);
    lcd.print("F: ");
    lcd.print(__DATE__);
    lcd.print("        ");
    lcd.setCursor(0,1);
    lcd.print("H: ");
    lcd.print(__TIME__);
    lcd.print("         ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}
    
void on_m4_item2_selected(MenuItem* p_menu_item)
{ //Ver Direccion IP
  lcd.setCursor(0,0);
  lcd.print("Direccion IP      ");
  lcd.setCursor(0,1);
  lcd.print(" ");
//  lcd.print(Ethernet.localIP());
  lcd.print("       ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_m4_item3_selected(MenuItem* p_menu_item)
{ //Editar Direccion IP
  setMenu = IPADDRESS;
  menuSelected = true;
  EditMode = true;
}
void on_m4_item4_selected(MenuItem* p_menu_item)
{ //Cambiar Modo Depuracion ON/OFF
  lcd.setCursor(0,0);
  lcd.print("Modo Depuracion ");
  lcd.setCursor(0,1);
  DEBUG = !DEBUG;
  if (DEBUG) {
    lcd.print("--> Encendido!  ");
  }
  else {
    lcd.print("--> Apagado!    ");
  }
  saveState(22, DEBUG);
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_m4_item5_selected(MenuItem* p_menu_item)
{ //Ver Modo Riego Programa/Smart!!
  lcd.setCursor(0,0);
  lcd.print("Modo Riego       ");
  lcd.setCursor(0,1);
  //Regar = !Regar;
  if (Regar) {
    lcd.print("--> Programa!   ");
  }
  else {
    lcd.print("--> Smart!      ");
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_m4_item6_selected(MenuItem* p_menu_item)
{ //Editar Modo Riego Programa/Smart!!
  lcd.setCursor(0,0);
  lcd.print("Modo Riego       ");
  lcd.setCursor(0,1);
  SetModoRiego(!Regar);
  if (Regar) {
    lcd.print("--> Programa!   ");
  }
  else {
    lcd.print("--> Smart!      ");
  }
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_m4_item7_selected(MenuItem* p_menu_item)
{ //Ver MQTT-SVR IP
  lcd.setCursor(0,0);
  lcd.print("MQTT-SVR IP       ");
  lcd.setCursor(0,1);
//  lcd.print(MQTT_SERVER);
  lcd.print("          ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_m4_item8_selected(MenuItem* p_menu_item)
{ //Edit MQTT-SVR IP
  setMenu = MQTTIP;
  menuSelected = true;
  EditMode = true;
}

void on_m4_item9_selected(MenuItem* p_menu_item)
{ //Ver MQTT-SVR PORT
  lcd.setCursor(0,0);
  lcd.print("MQTT-SVR PORT    ");
  lcd.setCursor(0,1);
  lcd.print("          ");
//  lcd.print(MQTT_SERVER_PORT);
  lcd.print("          ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}

void on_m4_item10_selected(MenuItem* p_menu_item)
{ //Edit MQTT-SVR PORT
  setMenu = MQTTPORT;
  menuSelected = true;
  EditMode = true;
}

void on_m4_item11_selected(MenuItem* p_menu_item)
{ //Reinicialiar --> Borrar variables almacenadas en memoria
  lcd.setCursor(0,1);
  lcd.print("-->Iniciando... ");
  if (DEBUG) {
    Serial.println("--> Iniciando borrado EEPROM. Espere...");    
  }

  for (int i=0;i<EEPROM_LOCAL_CONFIG_ADDRESS;i++) {
    hwWriteConfig(i,0xFF);  
  }
  if (DEBUG) {
    Serial.println("--> Borrado completado.");    
  }
  wait(300);
  lcd.print("-->Completado!  ");
  wait(1500);
  //delay(1500); // so we can look the result on the LCD
}
