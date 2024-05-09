/*
  emon_DB_6CT_1phase & emon_DB_6CT_3phase
  Author: Trystan Lea

  Uses EmonLibDB https://github.com/openenergymonitor/EmonLibDB
  With thanks to: Robert Wall & Robin Emley
  
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3

  -----------------------------------------

  Change Log:
  v2.0.0: Single phase 6CT energy monitor based on EmonLibDB library
  v2.0.1: Default nodeid set to 27
  v2.0.2: Change default phase allocation to 1-2-3 1-2-3
  v2.1.0: Combined single and three phase firmware for emonTx4/5 & emonPi2

*/
const char *firmware_version = {"2.1.0\n\r"};
/*

  emonhub.conf node decoder
  EmonTx4: (nodeid is 27 when switch is off, 28 when switch is on)

  Single phase node decoder:

  [[27]]
    nodename = emon_DB_6CT_1phase
    [[[rx]]]
      names = MSG, V1, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6, pulse
      datacodes = L, h, h,h,h,h,h,h, l,l,l,l,l,l ,L
      scales = 1,0.01,1,1,1,1,1,1,1,1,1,1,1
      units = n,V,W,W,W,W,W,W,Wh,Wh,Wh,Wh,Wh,Wh,p

  Three-phase node decoder:

  [[27]]
    nodename = emon_DB_6CT_3phase
    [[[rx]]]
      names = MSG, V1, V2, V3, P1, P2, P3, P4, P5, P6, E1, E2, E3, E4, E5, E6, pulse
      datacodes = L, h,h,h, h,h,h,h,h,h, l,l,l,l,l,l ,L
      scales = 1,0.01,0.01,0.01,1,1,1,1,1,1,1,1,1,1,1
      units = n,V,V,V,W,W,W,W,W,W,Wh,Wh,Wh,Wh,Wh,Wh,p

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

// 3. Set number of voltage channels (1 for single phase, 3 for three phase)
#define NUM_V_CHANNELS 1

// 4. Set number of current channels (this should always be 6)
#define NUM_I_CHANNELS 6

// 5. Include energy readings
#define ENABLE_ENERGY

// 6. Set pulse counting pin
// Options: 1 = pulse on digital, 2 = pulse on digital, 3 = pulse on analog (default)
#define PULSE_PIN 3

// 6. Enable analog reading (disabled by default)
// #define ENABLE_ANALOG

// 7. EEPROM wear leveling debug (disabled by default)
// #define EEWL_DEBUG

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
#include <emonLibDB.h> // OEM Continuous Monitoring library DB

// Payload length with 1V, 6I & Pulse = 4 + (1*2) + (6*2) + (6*4) + 4 = 46 bytes
// Payload length with 3V, 6I & Pulse = 4 + (3*2) + (6*2) + (6*4) + 4 = 50 bytes
// + 2 bytes for analog
typedef struct
{
  unsigned long Msg;
  int V[NUM_V_CHANNELS];
  int P[NUM_I_CHANNELS];
#ifdef ENABLE_ENERGY
  long E[NUM_I_CHANNELS];
#endif
  unsigned long pulse;
#ifdef ENABLE_ANALOG
  int analog;
#endif
} PayloadTX; // create a data packet for the RFM
PayloadTX emon;

// Settings - Stored in EEPROM and shared with config.ino
// Radio is only intended to be used for transmitting data on the emonTx4 and emonTx5
// The radio module on the emonPi2 is handled by the Raspberry Pi directly (hence it is turned off here)
struct
{
  byte RF_freq = RF69_433MHZ; // Frequency of radio module can be RFM_433MHZ (standard hardware), RFM_868MHZ or RFM_915MHZ.
  byte networkGroup = 210;    // wireless network group, must be the same as receiver. OEM default is 210
  byte nodeID = 27;           // node ID.
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
  bool pulse_enable = true;  // pulse counting
  int pulse_period = 100;    // pulse min period - 0 = no de-bounce
  bool showCurrents = false; // Print to serial voltage, current & p.f. values
  bool json_enabled = false; // JSON Enabled - false = key,Value pair, true = JSON, default = false: Key,Value pair.
} EEProm;

uint16_t eepromSig = 0x0020; // oemEProm signature - see oemEProm Library documentation for details.

#ifdef EEWL_DEBUG
extern EEWL EVmem;
#endif

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
  Serial.println(F("OpenEnergyMonitor.org"));

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
  Serial.print(F("Reference voltage calibration = "));
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

  EmonLibDB_set_pInput(1, 1); // Phase 1
  EmonLibDB_set_pInput(2, 2); // Phase 2
  EmonLibDB_set_pInput(3, 3); // Phase 3
  EmonLibDB_set_pInput(4, 1); // Phase 1
  EmonLibDB_set_pInput(5, 2); // Phase 2
  EmonLibDB_set_pInput(6, 3); // Phase 3
  /*
  EmonLibDB_set_pInput(1, 1, 2);               // CT1 between V1 & V2
  EmonLibDB_set_pInput(2, 2, 3);               // CT2 between V2 & V3  (etc)
  EmonLibDB_set_pInput(3, 3, 1);
  EmonLibDB_set_pInput(4, 1, 2);
  EmonLibDB_set_pInput(5, 2, 3);
  EmonLibDB_set_pInput(6, 3, 1);
  */
#else
  for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
  {
    EmonLibDB_set_pInput(ch + 1, 1);
  }
#endif

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

  // Pre-set Energy counters
  unsigned long p = 0;
  long e0 = 0, e1 = 0, e2 = 0, e3 = 0, e4 = 0, e5 = 0;

#ifdef ENABLE_ENERGY
  recoverEValues(&e0, &e1, &e2, &e3, &e4, &e5, &p);
  EmonLibDB_setWattHour(0, e0);
  EmonLibDB_setWattHour(1, e1);
  EmonLibDB_setWattHour(2, e2);
  EmonLibDB_setWattHour(3, e3);
  EmonLibDB_setWattHour(4, e4);
  EmonLibDB_setWattHour(5, e5);
#endif

  EmonLibDB_setPulseCount(p);

#ifdef EEWL_DEBUG
  EVmem.dump_control();
  EVmem.dump_buffer();
#endif

  EmonLibDB_Init(); // Start continuous monitoring.
  emon.Msg = 0;

  // Speed up startup by making first reading 2s
  EmonLibDB_datalogPeriod(2.0);
}

void loop()
{
  getSettings();

  if (EmonLibDB_Ready())
  {
    if (emon.Msg == 0)
    {
      digitalWrite(LEDpin, LOW);
      delay(5);
      EmonLibDB_datalogPeriod(EEProm.period);
    }

    emon.Msg++;

    // Other options calculated by EmonLibCM
    // RMS Current:    EmonLibCM_getIrms(ch)
    // Apparent Power: EmonLibCM_getApparentPower(ch)
    // Power Factor:   EmonLibCM_getPF(ch)

    for (byte ch = 0; ch < NUM_V_CHANNELS; ch++)
    {
      emon.V[ch] = EmonLibDB_getVrms(ch + 1) * 100;
    }

    for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
    {
      emon.P[ch] = EmonLibDB_getRealPower(ch + 1);
#ifdef ENABLE_ENERGY
      emon.E[ch] = EmonLibDB_getWattHour(ch + 1);
#endif
    }

    emon.pulse = EmonLibDB_getPulseCount(PULSE_PIN);

#ifdef ENABLE_ANALOG
    emon.analog = EmonLibDB_getAnalogueCount();
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

      for (byte ch = 0; ch < NUM_V_CHANNELS; ch++)
      {
        Serial.print(F(",\"V"));
        Serial.print(ch + 1);
        Serial.print("\":");
        Serial.print(emon.V[ch] * 0.01);
      }
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

      if (EEProm.pulse_enable) {
        // Pulse counting
        Serial.print(F(",\"pulse\":"));
        Serial.print(emon.pulse);
      }

// Analog reading
#ifdef ENABLE_ANALOG
      Serial.print(F(",\"analog\":"));
      Serial.print(analog);
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

      for (byte ch = 0; ch < NUM_V_CHANNELS; ch++)
      {
        Serial.print(F(",V"));
        Serial.print(ch + 1);
        Serial.print(":");
        Serial.print(emon.V[ch] * 0.01);
      }
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

      if (EEProm.pulse_enable) {
        // Pulse counting
        Serial.print(F(",pulse:"));
        Serial.print(emon.pulse);
      }

// Analog reading
#ifdef ENABLE_ANALOG
      Serial.print(F(",analog:"));
      Serial.print(analog);
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
          Serial.print(EmonLibDB_getIrms(ch + 1), 3);
        }
        for (byte ch = 0; ch < NUM_I_CHANNELS; ch++)
        {
          Serial.print(F(",pf"));
          Serial.print(ch + 1);
          Serial.print(":");
          Serial.print(EmonLibDB_getPF(ch + 1), 4);
        }
        Serial.println();
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

void print_firmware_version() {

  // Firmware version
#ifdef EMONTX4
  Serial.print(F("emonTx4"));
#endif
#ifdef EMONPI2
  Serial.print(F("emonPi2"));
#endif
#ifdef EMONTX5
  Serial.print(F("emonTx5"));
#endif
  Serial.print(F("_DB_6CT_"));
  Serial.print(NUM_V_CHANNELS);
  Serial.print(F("phase v"));
  
  Serial.write(firmware_version);
}
