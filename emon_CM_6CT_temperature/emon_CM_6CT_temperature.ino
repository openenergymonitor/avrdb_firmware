/*
  emon_CM_6CT_temperature
  Author: Trystan Lea

  Uses EmonLibCM https://github.com/openenergymonitor/EmonLibCM
  With thanks to: Robert Wall & Robin Emley
  
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3

  -----------------------------------------

  Change Log:
  v1.0.0: First release of EmonTxV4 Continuous Monitoring Firmware (based on EmonTx v3 CM Firmware)
  v1.1.0: Fixed emonEProm implementation for AVR-DB & new serial config implementation
  v1.2.0: LowPowerLabs radio format, with option to switch to JeeLib classic or native.
  v1.3.0: Read and calibrate reference voltage at startup
  v1.4.0: Option to output serial data as JSON (Brian Orpin)
  v1.5.0: emonEProm fixed pulse count data type issue
  v1.5.1: default node id set to 17, swap nodeid DIP, zero all 6 energy values
  v1.5.2: emonEProm fixed EEWL overlap
  v1.5.3: Slightly slower sample rate to improve zero power performance
          temperature sensing disabled if no temperature sensors detected at startup
  v1.5.4: Fix emonEProm EEWL overlap properly
  v1.5.5: RFM69_LPL library update use setPins
  v1.5.6: uses version 3.0.8 of EmonLibCM avrdb branch
          reduces interference caused by DS18B20 temperature sensing
  v1.5.7: Fix disabling of temperature sensing at startup if none detected
  v1.6.0: Single firmware for emonTx4/5 & emonPi2

*/
const char *firmware_version = {"1.6.0\n\r"};
/*

  emonhub.conf node decoder
  EmonTx4: (nodeid is 17 when switch is off, 18 when switch is on)

  [[17]]
    nodename = emon_CM_6CT_temperature_17
    [[[rx]]]
      names = MSG, Vrms, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6, T1, T2, T3, pulse
      datacodes = L,h,h,h,h,h,h,h,l,l,l,l,l,l,h,h,h,L
      scales = 1,0.01,1,1,1,1,1,1,1,1,1,1,0.01,0.01,0.01,1
      units = n,V,W,W,W,W,W,W,Wh,Wh,Wh,Wh,Wh,Wh,C,C,C,p

*/

// ------------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------------

// 1. Set hardware variant
// Options: EMONTX4, EMONTX5, EMONPI2
#define EMONTX5

// 2. Set radio format
// Options: RFM69_JEELIB_CLASSIC, RFM69_JEELIB_NATIVE, RFM69_LOW_POWER_LABS
#define RFM69_LOW_POWER_LABS

// 4. Set number of current channels (this should always be 6)
#define NUM_I_CHANNELS 6

// 5. Include energy readings
#define ENABLE_ENERGY

// 6. Set pulse counting pin
// Options: 1 = pulse on digital, 2 = pulse on digital, 3 = pulse on analog (default)
#define PULSE_PIN 3

// 6. Enable analog reading (disabled by default)
// IF ENABLED CHANGE NUM_I_CHANNELS = 5
// #define ENABLE_ANALOG

// 7. EEPROM wear leveling debug (disabled by default)
// #define EEWL_DEBUG

// 8. The maximum number of temperature sensors that can be read
#define MAX_TEMPS 3

// Resolution in bits, allowed values 9 - 12. 11-bit resolution, reads to 0.125 degC
#define DS18B20_RESOLUTION 11

// ------------------------------------------------------------------------

// Always Serial3
#define Serial Serial3

// Include libraries
#include <Arduino.h>
#include <avr/wdt.h>

// Include OLED display for emonPi2
#ifdef EMONPI2
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire oled;
#endif

// Include RFM69 library
#ifdef RFM69_LOW_POWER_LABS
#include "RFM69_LPL.h"
#else
// Minimal radio library that supports both original JeeLib format and later native format
#include "RFM69_JeeLib.h"
#endif
RFM69 rf;

// EEWL_START = 102, Start EEPROM wear leveling section after config section which takes up first 99 bytes, a few bytes of padding here
// EEWL_BLOCKS = 14, 14 x 29 byte blocks = 406 bytes. EEWL_END = 102+406 = 508 a few bytes short of 512 byte limit
// EEWL is only updated if values change by more than 200Wh, this means for a typical house consuming ~4000kWh/year
// 20,000 writes per channel to EEPROM, there's a 100,000 write lifetime for any individual EEPROM byte.
// With a circular buffer of 14 blocks, this extends the lifetime from 5 years to 70 years.
// IMPORTANT! If adding to config section change EEWL_START and check EEWL_END Implications.

#include <emonEProm.h> // OEM EEPROM library
#include <emonLibCM.h> // OEM Continuous Monitoring library CM

// Payload length with 1V, 6I & Pulse = 4 + (1*2) + (6*2) + (6*4) + (3*2) + 4 = 52 bytes
// + 2 bytes for analog
typedef struct
{
  unsigned long Msg;
  int Vrms;
  int P[NUM_I_CHANNELS];
#ifdef ENABLE_ENERGY
  long E[NUM_I_CHANNELS];
#endif
  int T[MAX_TEMPS];
  unsigned long pulse;
#ifdef ENABLE_ANALOG
  int analog;
#endif
} PayloadTX; // create a data packet for the RFM
PayloadTX emon;

int allTemps[MAX_TEMPS]; // Array to receive temperature sensor readings

// Settings - Stored in EEPROM and shared with config.ino
// Radio is only intended to be used for transmitting data on the emonTx4 and emonTx5
// The radio module on the emonPi2 is handled by the Raspberry Pi directly (hence it is turned off here)
struct
{
  byte RF_freq = RF69_433MHZ; // Frequency of radio module can be RFM_433MHZ (standard hardware), RFM_868MHZ or RFM_915MHZ.
  byte networkGroup = 210;    // wireless network group, must be the same as receiver. OEM default is 210
  byte nodeID = 17;           // node ID.
#ifdef EMONPI2
  byte rf_on = 0; // RF - 0 = no RF, 1 = RF on.
#else
  byte rf_on = 1; // RF - 0 = no RF, 1 = RF on.
#endif
  byte rfPower = 25;         // 7 = -10.5 dBm, 25 = +7 dBm for RFM12B; 0 = -18 dBm, 31 = +13 dBm for RFM69CW. Default = 25 (+7 dBm)
  float vCal = 807.86;       // (6 x 10000) / 75 = 800.0
  float assumedVrms = 240.0; // Assumed Vrms when no a.c. is detected
  float lineFreq = 50;       // Line Frequency = 50 Hz

  float iCal[NUM_I_CHANNELS];
  float iLead[NUM_I_CHANNELS];

  float period = 9.8;       // datalogging period - should be fractionally less than the PHPFINA database period in emonCMS
  bool pulse_enable = true; // pulse counting
  int pulse_period = 100;   // pulse min period - 0 = no de-bounce
#ifdef EMONPI2
  bool temp_enable = false; // if no temperature sensors are detected at startup this is over ridden and temperature sensing is disabled
#else
  bool temp_enable = true; // if no temperature sensors are detected at startup this is over ridden and temperature sensing is disabled
#endif
  DeviceAddress allAddresses[MAX_TEMPS]; // sensor address data
  bool showCurrents = false;             // Print to serial voltage, current & p.f. values
  bool json_enabled = false;             // JSON Enabled - false = key,Value pair, true = JSON, default = false: Key,Value pair.
} EEProm;

uint16_t eepromSig = 0x0016; // oemEProm signature - see oemEProm Library documentation for details.

#ifdef EEWL_DEBUG
extern EEWL EVmem;
#endif

bool temp_enable = false;
DeviceAddress allAddresses[MAX_TEMPS]; // Array to receive temperature sensor addresses
/*   Example - how to define temperature sensors, prevents an automatic search
DeviceAddress allAddresses[] = {
    {0x28, 0x81, 0x43, 0x31, 0x7, 0x0, 0xFF, 0xD9},
    {0x28, 0x8D, 0xA5, 0xC7, 0x5, 0x0, 0x0, 0xD5},         // Use the actual addresses, as many as required
    {0x28, 0xC9, 0x58, 0x32, 0x7, 0x0, 0x0, 0x89}          // up to a maximum of 6
};
*/

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

//----------------------------------------Setup--------------------------------------------------
void setup()
{
  // wdt_enable(WDTO_8S);

  // Indicator LED
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, HIGH);

  // Serial
  Serial.begin(115200);

// OLED display
// Very simple starting message and then
// control is passed to the Raspberry Pi
#ifdef EMONPI2
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
    EEProm.iCal[ch] = 60.06;
    EEProm.iLead[ch] = 3.2;
  }
  EEProm.iCal[0] = 300.30;
  EEProm.iCal[1] = 150.15;
  EEProm.iCal[2] = 150.15;

  // Load config from EEPROM (if any exists)
  load_config(true);

  // Firmware version
  Serial.print(F("emon_CM_6CT_temperature V"));
  Serial.write(firmware_version);
  Serial.println();
  Serial.println(F("OpenEnergyMonitor.org"));

  if (EEProm.rf_on)
  {
    Serial.print(F("RFM69CW "));
    Serial.print(F(" Freq: "));
    if (EEProm.RF_freq == RF69_433MHZ)
      Serial.print(F("433MHz"));
    if (EEProm.RF_freq == RF69_868MHZ)
      Serial.print(F("868MHz"));
    if (EEProm.RF_freq == RF69_915MHZ)
      Serial.print(F("915MHz"));
    Serial.print(F(" Group: "));
    Serial.print(EEProm.networkGroup);
    Serial.print(F(" Node: "));
    Serial.print(EEProm.nodeID);
    Serial.println(F(" "));

#ifdef RFM69_LOW_POWER_LABS
    Serial.println("RadioFormat: LowPowerLabs");
#elif defined(RFM69_JEELIB_CLASSIC)
    Serial.println("RadioFormat: JeeLib Classic");
#elif defined(RFM69_JEELIB_NATIVE)
    Serial.println("RadioFormat: JeeLib Native");
#endif
  }

#ifdef EMONTX4
  // Sets expected frequency 50Hz/60Hz
  if (digitalRead(DIP_switch1) == LOW)
  {
    USA = true; // 60 Hz
  }
#endif
  // ---------------------------------------------------------------------------------------

  if (EEProm.rf_on)
  {
#ifdef RFM69_JEELIB_CLASSIC
    rf.format(RFM69_JEELIB_CLASSIC);
#endif

// Frequency is currently hardcoded to 433Mhz in library
#ifdef RFM69_LOW_POWER_LABS
#ifdef EMONTX4
    rf.setPins(PIN_PB5, PIN_PC0, PIN_PC1, PIN_PC2);
#else
    rf.setPins(PIN_PA7, PIN_PA4, PIN_PA5, PIN_PA6);
#endif
#endif
    rf.initialize(RF69_433MHZ, EEProm.nodeID, EEProm.networkGroup);
    rf.encrypt("89txbe4p8aik5kt3");    // ignored if jeelib classic
    delay(random(EEProm.nodeID * 20)); // try to avoid r.f. collisions at start-up
  }

  // ---------------------------------------------------------------------------------------

#ifdef EEWL_DEBUG
  EVmem.dump_buffer();
#endif

  double reference = read_reference();
  Serial.print(F("Reference voltage calibration: "));
  Serial.println(reference, 4);

  // 12 bit ADC = 4096 divisions
  // Time in microseconds for one ADC conversion: 39.333 us
  EmonLibCM_setADC(12, 39.333);
  EmonLibCM_ADCCal(reference); // ADC Reference voltage, (1.024 V)

  // Note that we are setting pin mapping here: 0,3,4,5,6,8 (skips 1,2 & 7)
  EmonLibCM_SetADC_VChannel(0, EEProm.vCal);                     // ADC Input channel, voltage calibration
  EmonLibCM_SetADC_IChannel(3, EEProm.iCal[0], EEProm.iLead[0]); // ADC Input channel, current calibration, phase calibration
  EmonLibCM_SetADC_IChannel(4, EEProm.iCal[1], EEProm.iLead[1]); // The current channels will be read in this order
  EmonLibCM_SetADC_IChannel(5, EEProm.iCal[2], EEProm.iLead[2]);
  EmonLibCM_SetADC_IChannel(6, EEProm.iCal[3], EEProm.iLead[3]);
  EmonLibCM_SetADC_IChannel(8, EEProm.iCal[4], EEProm.iLead[4]);

#ifdef ENABLE_ANALOG
  EmonLibCM_SetADC_IChannel(19, EEProm.iCal[5], EEProm.iLead[5]);
#else
  EmonLibCM_SetADC_IChannel(9, EEProm.iCal[5], EEProm.iLead[5]);
#endif

  // mains frequency 50Hz
  if (!USA)
  {
    EmonLibCM_cycles_per_second(50); // mains frequency 50Hz
  }
  else
  {
    EmonLibCM_cycles_per_second(60); // mains frequency 60Hz
  }
  EmonLibCM_datalog_period(EEProm.period); // period of readings in seconds - normal value for emoncms.org

  EmonLibCM_setAssumedVrms(EEProm.assumedVrms);

  // Pulse counting configuration
  // Make sure appropriate solder link is made on the hardware and related TMP link is broken
  // The 'Analogue' input is not available if an extender card is fitted
  
  if (PULSE_PIN == 1)
  {
    Serial.print(F("Pulse: "));
#ifdef EMONTX4
    EmonLibCM_setPulsePin(PIN_PA6); // Standard pulse input
#else
    EmonLibCM_setPulsePin(PIN_PB5); // Standard pulse input
#endif
  }
  else if (PULSE_PIN == 2)
  {
    Serial.print(F("Pulse on digital: "));
#ifdef EMONTX4
    EmonLibCM_setPulsePin(PIN_PA7); // Second digital input
#else
    EmonLibCM_setPulsePin(PIN_PC0); // Second digital input
#endif
  }
  else if (PULSE_PIN == 3)
  {
    Serial.print(F("Pulse on analog: "));
    EmonLibCM_setPulsePin(PIN_PF3); // Pulse on analog
  }

  if (EEProm.pulse_enable)
  {
    Serial.print(F("enabled"));

    Serial.print(F(", min period: "));
    Serial.print(EEProm.pulse_period);
    Serial.println(F("ms"));
  }
  else
  {
    Serial.println(F("disabled"));
  }

  EmonLibCM_setPulseEnable(EEProm.pulse_enable);
  EmonLibCM_setPulseMinPeriod(EEProm.pulse_period);

  // Temperature configuration
  EmonLibCM_setTemperatureDataPin(PIN_PB4); // OneWire data pin
#ifdef EMONTX4
  EmonLibCM_setTemperaturePowerPin(PIN_PB3);
#else
  EmonLibCM_setTemperaturePowerPin(PIN_PC1); // Temperature sensor Power Pin - 19 for emonPi2  (-1 = Not used. No sensors, or sensor are permanently powered.)
#endif
  EmonLibCM_setTemperatureResolution(DS18B20_RESOLUTION); // Resolution in bits, allowed values 9 - 12. 11-bit resolution, reads to 0.125 degC
  EmonLibCM_setTemperatureAddresses(EEProm.allAddresses); // Name of array of temperature sensors
  EmonLibCM_setTemperatureArray(allTemps);                // Name of array to receive temperature measurements
  EmonLibCM_setTemperatureMaxCount(MAX_TEMPS);            // Max number of sensors, limited by wiring and array size.

  // Pre-set Energy counters
  unsigned long p = 0;
  long e0 = 0, e1 = 0, e2 = 0, e3 = 0, e4 = 0, e5 = 0;

#ifdef ENABLE_ENERGY
  recoverEValues(&e0, &e1, &e2, &e3, &e4, &e5, &p);
  EmonLibCM_setWattHour(0, e0);
  EmonLibCM_setWattHour(1, e1);
  EmonLibCM_setWattHour(2, e2);
  EmonLibCM_setWattHour(3, e3);
  EmonLibCM_setWattHour(4, e4);
  EmonLibCM_setWattHour(5, e5);
#endif

  EmonLibCM_setPulseCount(p);

#ifdef EEWL_DEBUG
  EVmem.dump_control();
  EVmem.dump_buffer();
#endif

  EmonLibCM_TemperatureEnable(EEProm.temp_enable);
  EmonLibCM_Init(); // Start continuous monitoring.
  emon.Msg = 0;

  if (EEProm.temp_enable)
  {
    printTemperatureSensorAddresses();

    byte numSensors = EmonLibCM_getTemperatureSensorCount();
    if (numSensors == 0)
    {
      Serial.println(F("No temperature sensors detected, disabling temperature"));
      EmonLibCM_TemperatureEnable(false);
      temp_enable = false;
    }
    else
    {
      temp_enable = true;
    }
  }
  // Speed up startup by making first reading 2s
  EmonLibCM_datalog_period(2.0);
}

void loop()
{
  getSettings();

  if (EmonLibCM_Ready())
  {
    if (emon.Msg == 0)
    {
      digitalWrite(LEDpin, LOW);
      delay(5);
      EmonLibCM_datalog_period(EEProm.period);
      if (EmonLibCM_acPresent())
      {
        Serial.println(F("AC present - Real Power calc enabled"));
      }
      else
      {
        Serial.print(F("AC missing - Apparent Power calc enabled, assuming "));
        Serial.print(EEProm.assumedVrms);
        Serial.println(F(" V"));
      }
    }

    emon.Msg++;

    // Other options calculated by EmonLibCM
    // RMS Current:    EmonLibCM_getIrms(ch)
    // Apparent Power: EmonLibCM_getApparentPower(ch)
    // Power Factor:   EmonLibCM_getPF(ch)

    if (EmonLibCM_acPresent())
    {
      emon.Vrms = EmonLibCM_getVrms() * 100;
    }
    else
    {
      emon.Vrms = EmonLibCM_getAssumedVrms() * 100;
    }

    for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
    {
      emon.P[ch] = EmonLibCM_getRealPower(ch);
#ifdef ENABLE_ENERGY
      emon.E[ch] = EmonLibCM_getWattHour(ch);
#endif
    }

    if (temp_enable)
    {
      for (byte ch = 0; ch < MAX_TEMPS; ch++)
      {
        emon.T[ch] = allTemps[ch];
      }
    }

    emon.pulse = EmonLibCM_getPulseCount();

#ifdef ENABLE_ANALOG
    emon.analog = EmonLibCM_getMean(NUM_I_CHANNELS - 1);
#endif

    if (EEProm.rf_on)
    {
      PayloadTX tmp = emon;

#ifdef RFM69_LOW_POWER_LABS
      rf.sendWithRetry(5, (byte *)&tmp, sizeof(tmp));
#else
      rf.send(0, (byte *)&tmp, sizeof(tmp));
#endif

      /*
      // Replace with this to count retry attempts
      if (rf.sendWithRetry(5,(byte *)&tmp, sizeof(tmp))) {
        Serial.println("ack");
        emontx.T1 += rf.retry_count();
      } else {
        emontx.T1 += rf.retry_count()+1;
      }
      */

      delay(50);
    }

    if (EEProm.json_enabled)
    {
      // ---------------------------------------------------------------------
      // JSON Format
      // ---------------------------------------------------------------------
      Serial.print(F("{\"MSG\":"));
      Serial.print(emon.Msg);
      Serial.print(F(",\"Vrms\":"));
      Serial.print(emon.Vrms * 0.01);

      for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
      {
        Serial.print(F(",\"P"));
        Serial.print(ch + 1);
        Serial.print("\":");
        Serial.print(emon.P[ch]);
      }

#ifdef ENABLE_ENERGY
      for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
      {
        Serial.print(F(",\"E"));
        Serial.print(ch + 1);
        Serial.print("\":");
        Serial.print(emon.E[ch]);
      }
#endif

#ifdef ENABLE_CURRENT
      for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
      {
        Serial.print(F(",\"I"));
        Serial.print(ch + 1);
        Serial.print("\":");
        Serial.print(emon.I[ch]);
      }
#endif

      if (temp_enable)
      {
        for (byte ch = 0; ch < MAX_TEMPS; ch++)
        {
          if (emon.T[ch] != 30000)
          {
            Serial.print(F(",\"T"));
            Serial.print(ch + 1);
            Serial.print("\":");
            Serial.print(emon.T[ch] * 0.01);
          }
        }
      }

      Serial.print(F(",\"pulse\":"));
      Serial.print(emon.pulse);

// Analog reading
#ifdef ENABLE_ANALOG
      Serial.print(F(",\"analog\":"));
      Serial.print(emon.analog);
#endif

      Serial.println(F("}"));
      delay(60);
    }
    else
    {

      // ---------------------------------------------------------------------
      // Key:Value format, used by EmonESP & emonhub EmonHubOEMInterfacer
      // ---------------------------------------------------------------------
      Serial.print(F("MSG:"));
      Serial.print(emon.Msg);
      Serial.print(F(",Vrms:"));
      Serial.print(emon.Vrms * 0.01);

      for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
      {
        Serial.print(F(",P"));
        Serial.print(ch + 1);
        Serial.print(":");
        Serial.print(emon.P[ch]);
      }

#ifdef ENABLE_ENERGY
      for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
      {
        Serial.print(F(",E"));
        Serial.print(ch + 1);
        Serial.print(":");
        Serial.print(emon.E[ch]);
      }
#endif

#ifdef ENABLE_CURRENT
      for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
      {
        Serial.print(F(",I"));
        Serial.print(ch + 1);
        Serial.print(":");
        Serial.print(emon.I[ch]);
      }
#endif

      if (temp_enable)
      {
        for (byte ch = 0; ch < MAX_TEMPS; ch++)
        {
          if (emon.T[ch] != 30000)
          {
            Serial.print(F(",T"));
            Serial.print(ch + 1);
            Serial.print(":");
            Serial.print(emon.T[ch] * 0.01);
          }
        }
      }

      Serial.print(F(",pulse:"));
      Serial.print(emon.pulse);

// Analog reading
#ifdef ENABLE_ANALOG
      Serial.print(F(",analog:"));
      Serial.print(emon.analog);
#endif

      if (!EEProm.showCurrents)
      {
        Serial.println();
        delay(40);
      }
      else
      {
        // to show voltage, current & power factor for calibration:
        for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
        {
          Serial.print(F(",I"));
          Serial.print(ch + 1);
          Serial.print(":");
          Serial.print(EmonLibCM_getIrms(EmonLibCM_getLogicalChannel(ch + 1)), 3);
        }
        for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
        {
          Serial.print(F(",pf"));
          Serial.print(ch + 1);
          Serial.print(":");
          Serial.print(EmonLibCM_getPF(EmonLibCM_getLogicalChannel(ch + 1)), 4);
        }
        delay(80);
      }
    }
    digitalWrite(LEDpin, HIGH);
    delay(50);
    digitalWrite(LEDpin, LOW);
// End of print out ----------------------------------------------------
#ifdef ENABLE_ENERGY
    storeEValues(emon.E[0], emon.E[1], emon.E[2], emon.E[3], emon.E[4], emon.E[5], emon.pulse);
#endif
  }
  wdt_reset();
  delay(20);
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
