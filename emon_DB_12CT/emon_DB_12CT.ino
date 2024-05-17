/*

Firmware: emon_DB_12CT

Due to the limits imposed by the data format required by the radio module, it is necessary to split the data into
two packets sent consecutively. Each packet is sent from a different NodeID, but both relate to the same datalog period
and share the same message number.

The first packet contains values pertaining to CTs 1-6, the second packet to CTs 7 - 12. 

  Change Log:
  v1.0.0: First release of 12 CT firmware based on emonLibDB_rf example
  v1.1.0: Combined emonTx4, emonTx5 and emonPi2 firmware
  v1.2.0: Serial configuration

*/

const char *firmware_version = {"1.2.0\n\r"};

// ------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------

// 1. Set hardware variant
// Options: EMONTX4, EMONTX5, EMONPI2
#define EMONTX5

// 2. Set number of voltage channels (1 for single phase, 3 for three phase)
#define NUM_V_CHANNELS 1

// 3. Set number of current channels (this should always be 12)
#define NUM_I_CHANNELS 12

// 4. Include energy readings
// #define ENABLE_ENERGY

// 5. Set pulse counting pin
// Options: 1 = pulse on digital, 2 = pulse on digital, 3 = pulse on analog (default)
#define PULSE_PIN 1

// ------------------------------------------------------------------------

// Always Serial3
#define Serial Serial3

// Include libraries
#include <Arduino.h>

// Include OLED display for emonPi2
#ifdef EMONPI2
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire oled;
#endif

// Include RFM69 library
#include <RFM69_LPL.h>
RFM69 rf;

#include <emonEProm.h> // OEM EEPROM library
#include <emonLibDB.h> // OEM Continuous Monitoring library DB

#define DATAWAIT 120    // millisecs between data packets
#define ACK1_RETRIES 8
#define ACK1_TIMEOUT 30
#define ACK2_RETRIES 8
#define ACK2_TIMEOUT 30

// EEProm.nodeID (28)
struct {
    uint32_t Msg;
    int16_t V1,V2,V3,P1,P2,P3,P4,P5,P6; 
    int32_t E1,E2,E3,E4,E5,E6; 
    uint32_t pulse;
    uint16_t ana;
} txPacket1;

/*  52 bytes
[[28]]
    nodename = emonDB_28
    [[[rx]]]
        names = MSG, Vrms1, Vrms2, Vrms3, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6, pulse, Analog
        datacodes = L, h, h, h, h, h, h, h, h, h, l, l, l, l, l, l, L, H
        scales = 1.0, 0.01, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        units = n, V, V, V, W, W, W, W, W, W, Wh, Wh, Wh, Wh, Wh, Wh, p, n
*/
        
// EEProm.nodeID (29)
struct {
    uint32_t Msg;
    int16_t V2,V3,P7,P8,P9,P10,P11,P12; 
    int32_t E7,E8,E9,E10,E11,E12; 
    uint32_t pulseD, pulseA;
} txPacket2;

/*  52 bytes
[[29]]
    nodename = emonDB_29
    [[[rx]]]
        names = MSG, Vrms2, Vrms3, P7, P8, P9, P10, P11, P12, E7, E8, E9, E10, E11, E12, digPulse, anaPulse
        datacodes = L, h, h, h, h, h, h, h, h, l, l, l, l, l, l, L, L
        scales = 1.0, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
        units = n, V, V, W, W, W, W, W, W, Wh, Wh, Wh, Wh, Wh, Wh, p, p
*/

// Settings - Stored in EEPROM and shared with config.ino
// Radio is only intended to be used for transmitting data on the emonTx4 and emonTx5
// The radio module on the emonPi2 is handled by the Raspberry Pi directly (hence it is turned off here)
struct
{
  byte RF_freq = RF69_433MHZ; // Frequency of radio module can be RFM_433MHZ (standard hardware), RFM_868MHZ or RFM_915MHZ.
  byte networkGroup = 210;    // wireless network group, must be the same as receiver. OEM default is 210
  byte nodeID = 28;           // node ID.
#ifdef EMONPI2
  byte rf_on = 0; // RF - 0 = no RF, 1 = RF on.
#else
  byte rf_on = 1; // RF - 0 = no RF, 1 = RF on.
#endif
  byte rfPower = 25;   // 7 = -10.5 dBm, 25 = +7 dBm for RFM12B; 0 = -18 dBm, 31 = +13 dBm for RFM69CW. Default = 25 (+7 dBm)
  float vCal = 101.3;  // Only single vCal for three phase in this firmware
  float lineFreq = 50; // Line Frequency = 50 Hz

  float iCal[NUM_I_CHANNELS];
  float iLead[NUM_I_CHANNELS];

  float period = 9.8;        // datalogging period - should be fractionally less than the PHPFINA database period in emonCMS
  bool pulse_enable = false;  // pulse counting
  int pulse_period = 100;    // pulse min period - 0 = no de-bounce
  bool showCurrents = false; // Print to serial voltage, current & p.f. values
  bool json_enabled = false; // JSON Enabled - false = key,Value pair, true = JSON, default = false: Key,Value pair.
} EEProm;
// sizeof: 1+1+1+1+1+(4×12)+(4×12)+4+1+2+1+1 = 110 bytes

uint16_t eepromSig = 0x0030; // oemEProm signature - see oemEProm Library documentation for details.

// This only sets the expected frequency 50Hz/60Hz
// The voltage calibration stays the same
bool USA = false;

// Enable on-line calibration when running.
bool calibration_enable = true;

// Pin definitions
#ifdef EMONTX4
const byte LEDpin = PIN_PB2;      // emonTx V4 LED
const byte DIP_switch1 = PIN_PA4; // RF node ID (default no change in node ID, switch on for nodeID + 1) switch off D8 is HIGH from internal pullup
const byte DIP_switch2 = PIN_PA5; // Voltage selection 240 / 120 V AC (default switch off 240V)  - switch off D9 is HIGH from internal pullup
#else
const byte LEDpin = PIN_PC2; // emonPi2/Tx5 LED
#endif

// Used in config.ini
static void showString (PGM_P s);

//----------------------------------------Setup--------------------------------------------------
void setup()
{
  // wdt_enable(WDTO_8S);

  // Indicator LED
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, HIGH);

  // Serial
  Serial.begin(115200);
  print_firmware_version();

// OLED display
// Very simple starting message and then
// control is passed to the Raspberry Pi
#ifdef EMONPI2
  Serial.println(F("Starting emonPi2 OLED"));
  delay(1000);
  Wire1.swap(2);
  Wire1.begin();
  Wire1.setClock(400000L);

  oled.begin(&Adafruit128x64, 0x3C);

  oled.setFont(CalLite24);
  oled.setLetterSpacing(2);

  oled.clear();
  oled.setCursor(15, 1);
  oled.print("emonPi2");
  oled.setFont(Arial14);
  oled.setCursor(40, 6);

  oled.print("starting...");
  Wire1.end();

  pinMode(PIN_PB2, INPUT);
  pinMode(PIN_PB3, INPUT);

#endif

// EmonTx4 DIP switch settings
#ifdef EMONTX4
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  if (digitalRead(DIP_switch2) == LOW)
  {
    // IF DIP switch 1 is switched on (LOW) then add 1 from nodeID
    EEProm.nodeID++;
  }
#endif

  // Default calibration values
  for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
  {
    EEProm.iCal[ch] = 20.0;
    EEProm.iLead[ch] = 3.2;
  }
  // EEProm.iCal[0] = 100.0;
  // EEProm.iCal[1] = 50.0;
  // EEProm.iCal[2] = 50.0;

  // Load config from EEPROM (if any exists)
  load_config(true);

#ifdef EMONTX4
  // Sets expected frequency 50Hz/60Hz
  if (digitalRead(DIP_switch1) == LOW)
  {
    USA = true; // 60 Hz
  }
#endif
  // ---------------------------------------------------------------------------------------

  init_radio();

  // ---------------------------------------------------------------------------------------

  double reference = read_reference();
  Serial.print(F("vrefa = "));
  Serial.println(reference, 4);

  // Apply voltage and current channel calibration
  EmonLibDB_set_vInput(1, EEProm.vCal, 0.16); // emonVS Input channel 1, voltage calibration 100, phase error 0.16°
#if NUM_V_CHANNELS == 3
  EmonLibDB_set_vInput(2, EEProm.vCal, 0.16); // emonVS Input channel 2, voltage calibration 100, phase error 0.16°
  EmonLibDB_set_vInput(3, EEProm.vCal, 0.16); // emonVS Input channel 3, voltage calibration 100, phase error 0.16°
#endif

  for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
  {
    EmonLibDB_set_cInput(ch + 1, EEProm.iCal[ch], EEProm.iLead[ch]);
  }

// Link voltage and current sensors to define the power & energy measurements
// For best precision and performance, include only the following lines that 
// apply to current/power inputs being used
#if NUM_V_CHANNELS == 3
  EmonLibDB_set_pInput(1, 1);                  // CT1, V1 (etc)
  EmonLibDB_set_pInput(2, 2);
  EmonLibDB_set_pInput(3, 3);
  EmonLibDB_set_pInput(4, 1);  
  EmonLibDB_set_pInput(5, 2);
  EmonLibDB_set_pInput(6, 3);
  EmonLibDB_set_pInput(7, 1);                  // CT7, V1 (etc)  
  EmonLibDB_set_pInput(8, 2);
  EmonLibDB_set_pInput(9, 3);
  EmonLibDB_set_pInput(10, 1);  
  EmonLibDB_set_pInput(11, 2);
  EmonLibDB_set_pInput(12, 3);
#else
  for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
  {
    EmonLibDB_set_pInput(ch + 1, 1);
  }
#endif

  /* How to measure Line-Line loads: */
/*
  EmonLibDB_set_pInput(3, 1, 2);               // CT1 between V1 & V2    
  EmonLibDB_set_pInput(2, 2, 3);               // CT2 between V2 & V3
  EmonLibDB_set_pInput(3, 3, 1);               // CT2 between V3 & V1  (etc)  
  EmonLibDB_set_pInput(4, 1, 2);  
  EmonLibDB_set_pInput(5, 2, 3);
  EmonLibDB_set_pInput(6, 3, 1);

  EmonLibDB_set_pInput(7, 1, 2);  
  EmonLibDB_set_pInput(8, 2, 3);
  EmonLibDB_set_pInput(9, 3, 1);
  EmonLibDB_set_pInput(10, 1, 2);  
  EmonLibDB_set_pInput(11, 2, 3);
  EmonLibDB_set_pInput(12, 3, 1);
*/

  // mains frequency 50Hz
  if (!USA)
  {
    EmonLibDB_cyclesPerSecond(50); // mains frequency 50Hz
  }
  else
  {
    EmonLibDB_cyclesPerSecond(60); // mains frequency 60Hz
  }
  EmonLibDB_minStartupCycles(10);         // number of cycles to let ADC run before starting first actual measurement
  EmonLibDB_datalogPeriod(EEProm.period); // period of readings in seconds - normal value for emoncms.org
  EmonLibDB_ADCCal(reference);            // ADC Reference voltage, (1.024 V)

  // Pulse counting configuration
  // Make sure appropriate solder link is made on the hardware and related TMP link is broken
  // The 'Analogue' input is not available if an extender card is fitted
  
  EmonLibDB_setPulseEnable(PULSE_PIN, EEProm.pulse_enable);
  EmonLibDB_setPulseMinPeriod(PULSE_PIN, EEProm.pulse_period, FALLING); // Trigger on the falling edge

#ifdef ENABLE_ANALOG
  EmonLibDB_setAnalogueEnable(true);
#endif

  EmonLibDB_Init(); // Start continuous monitoring.
  txPacket1.Msg = 0;
  txPacket2.Msg = 0;

  // Speed up startup by making first reading 2s
  EmonLibDB_datalogPeriod(2.0);
}

void loop()
{
  getSettings();

  if (EmonLibDB_Ready())
  {
    if (txPacket1.Msg == 0)
    {
      digitalWrite(LEDpin, LOW);
      delay(5);
      EmonLibDB_datalogPeriod(EEProm.period);
    }
    
    txPacket1.Msg++;
    txPacket1.V1 = EmonLibDB_getVrms(1) * 100.0;
    txPacket1.V2 = EmonLibDB_getVrms(2) * 100.0;
    txPacket1.V3 = EmonLibDB_getVrms(3) * 100.0;
    txPacket1.P1 = EmonLibDB_getRealPower(1);
    txPacket1.P2 = EmonLibDB_getRealPower(2);
    txPacket1.P3 = EmonLibDB_getRealPower(3);
    txPacket1.P4 = EmonLibDB_getRealPower(4);
    txPacket1.P5 = EmonLibDB_getRealPower(5);
    txPacket1.P6 = EmonLibDB_getRealPower(6);
    txPacket1.E1 = EmonLibDB_getWattHour(1);
    txPacket1.E2 = EmonLibDB_getWattHour(2);
    txPacket1.E3 = EmonLibDB_getWattHour(3);
    txPacket1.E4 = EmonLibDB_getWattHour(4);
    txPacket1.E5 = EmonLibDB_getWattHour(5);
    txPacket1.E6 = EmonLibDB_getWattHour(6);
    txPacket1.pulse = EmonLibDB_getPulseCount(1);
    txPacket1.ana = EmonLibDB_getAnalogueCount();
        
    if (EEProm.rf_on)
    {
      rf.setAddress(EEProm.nodeID);
      if (!rf.sendWithRetry(5,(byte *)&txPacket1, sizeof(txPacket1), ACK1_RETRIES, ACK1_TIMEOUT)) {
        // Serial.println("RF No Ack (1)");
      }
    }
    
    txPacket2.Msg++;
    txPacket2.V2 = EmonLibDB_getVrms(2) * 100.0;
    txPacket2.V3 = EmonLibDB_getVrms(3) * 100.0;
    txPacket2.P7 = EmonLibDB_getRealPower(7);
    txPacket2.P8 = EmonLibDB_getRealPower(8);
    txPacket2.P9 = EmonLibDB_getRealPower(9);
    txPacket2.P10 = EmonLibDB_getRealPower(10);
    txPacket2.P11 = EmonLibDB_getRealPower(11);
    txPacket2.P12 = EmonLibDB_getRealPower(12);
    txPacket2.E7 = EmonLibDB_getWattHour(7);
    txPacket2.E8 = EmonLibDB_getWattHour(8);
    txPacket2.E9 = EmonLibDB_getWattHour(9);
    txPacket2.E10 = EmonLibDB_getWattHour(10);
    txPacket2.E11 = EmonLibDB_getWattHour(11);
    txPacket2.E12 = EmonLibDB_getWattHour(12);
    txPacket2.pulseD = EmonLibDB_getPulseCount(2);
    txPacket2.pulseA = EmonLibDB_getPulseCount(3);
    
    delay(DATAWAIT);
  
    if (EEProm.rf_on)
    {
      rf.setAddress(EEProm.nodeID + 1);
      if (!rf.sendWithRetry(5,(byte *)&txPacket2, sizeof(txPacket2), ACK2_RETRIES, ACK2_TIMEOUT)) {
        // Serial.println("RF No Ack (2)");
      }
    }
    
    delay(100);

    if (EEProm.json_enabled)
    {
      // ---------------------------------------------------------------------
      // JSON Format
      // ---------------------------------------------------------------------
      Serial.print(F("{\"MSG\":")); Serial.print(txPacket1.Msg);
      // Serial.print(EmonLibDB_acPresent(1)?"  AC present ":"  AC missing ");
      
      Serial.print(F(",\"V1\":")); Serial.print(EmonLibDB_getVrms(1));
  #if NUM_V_CHANNELS == 3
      Serial.print(F(",\"V2\":")); Serial.print(EmonLibDB_getVrms(2));
      Serial.print(F(",\"V3\":")); Serial.print(EmonLibDB_getVrms(3));
  #endif
      
      Serial.print(F(",\"F\":")); Serial.print(EmonLibDB_getLineFrequency());
      
      for (uint8_t ch=1; ch<=12; ch++)
      {
        Serial.print(F(",\"P")); Serial.print(ch); Serial.print("\":"); Serial.print(EmonLibDB_getRealPower(ch));
        // Serial.print(F(",\"I"));  Serial.print(ch); Serial.print("\":"); Serial.print(EmonLibDB_getIrms(ch),3);
        // Serial.print(F(",\"VA")); Serial.print(ch); Serial.print("\":"); Serial.print(EmonLibDB_getApparentPower(ch));
        #ifdef ENABLE_ENERGY
        Serial.print(F(",\"E")); Serial.print(ch); Serial.print("\":"); Serial.print(EmonLibDB_getWattHour(ch));
        #endif
        // Serial.print(F(",\"PF")); Serial.print(ch); Serial.print("\":"); Serial.print(EmonLibDB_getPF(ch),4);
      } 
      Serial.println(F("}"));
    } 
    else 
    {

      // ---------------------------------------------------------------------
      // Key:Value format, used by EmonESP & emonhub EmonHubOEMInterfacer
      // ---------------------------------------------------------------------
      Serial.print("MSG:"); Serial.print(txPacket1.Msg); Serial.print(",");
      // Serial.print(EmonLibDB_acPresent(1)?"  AC present ":"  AC missing ");
      
      Serial.print("V1:"); Serial.print(EmonLibDB_getVrms(1)); Serial.print(",");
  #if NUM_V_CHANNELS == 3
      Serial.print("V2:"); Serial.print(EmonLibDB_getVrms(2)); Serial.print(",");
      Serial.print("V3:"); Serial.print(EmonLibDB_getVrms(3)); Serial.print(",");
  #endif
      
      Serial.print("F:"); Serial.print(EmonLibDB_getLineFrequency());
      
      for (uint8_t ch=1; ch<=12; ch++)
      {
        Serial.print(",P"); Serial.print(ch); Serial.print(":"); Serial.print(EmonLibDB_getRealPower(ch));
        // Serial.print(",I"); Serial.print(ch); Serial.print(":"); Serial.print(EmonLibDB_getIrms(ch),3);
        // Serial.print(",VA"); Serial.print(ch); Serial.print(":"); Serial.print(EmonLibDB_getApparentPower(ch));
        #ifdef ENABLE_ENERGY
        Serial.print(",E"); Serial.print(ch); Serial.print(":"); Serial.print(EmonLibDB_getWattHour(ch));
        #endif
        // Serial.print(",PF"); Serial.print(ch); Serial.print(":"); Serial.print(EmonLibDB_getPF(ch),4);
      } 
      Serial.println();
    }
    digitalWrite(LEDpin, HIGH);
    delay(50);
    digitalWrite(LEDpin, LOW);
    // End of print out ----------------------------------------------------
    delay(200);  
  }
}

void panic(uint8_t rate, uint8_t duration)
{
  while(duration)
  {
    PORTB.OUT |= PIN2_bm;
    delay(rate<<8);
    PORTB.OUT &= ~PIN2_bm;
    delay(rate<<8);
    duration--;
  }
  return;
}

double read_reference()
{
  ADC0.SAMPCTRL = 14;
  ADC0.CTRLD |= 0x0;
  VREF.ADC0REF = VREF_REFSEL_1V024_gc;
  ADC0.CTRLC = ADC_PRESC_DIV24_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;
  ADC0.CTRLA |= ADC_RESSEL_12BIT_gc;

  ADC0.MUXPOS = 7;
  unsigned long sum = 0;
  for (int i = 0; i < 10010; i++)
  {
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
      ;
    if (i > 9)
    {
      sum += ADC0.RES;
    }
  }
  double mean = sum / 10000.0;
  double reference = 0.9 / (mean / 4095.0);
  return reference;
}

void print_firmware_version() {

  Serial.println(F("firmware = emon_DB_12CT"));
  Serial.print(F("version = "));
  Serial.write(firmware_version);

  Serial.print(F("hardware = "));

#ifdef EMONTX4
  Serial.println(F("emonTx4"));
#endif
#ifdef EMONPI2
  Serial.println(F("emonPi2"));
#endif
#ifdef EMONTX5
  Serial.println(F("emonTx5"));
#endif
  Serial.print(F("voltage = "));

  Serial.print(NUM_V_CHANNELS);
  Serial.println(F("phase"));
}

void init_radio() {
  if (EEProm.rf_on)
  {

// Frequency is currently hardcoded to 433Mhz in library
#ifdef EMONTX4
    rf.setPins(PIN_PB5, PIN_PC0, PIN_PC1, PIN_PC2);
#else
    rf.setPins(PIN_PA7, PIN_PA4, PIN_PA5, PIN_PA6);
#endif

    rf.initialize(RF69_433MHZ, EEProm.nodeID, EEProm.networkGroup);
    rf.encrypt("89txbe4p8aik5kt3");    // ignored if jeelib classic
    delay(random(EEProm.nodeID * 20)); // try to avoid r.f. collisions at start-up
  }
}
