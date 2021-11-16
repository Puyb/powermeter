/**
   Main
   ====

*/

#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include "nrfx_nvmc.h"
#include "board.h"
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <Wire.h>
#include <bluefruit.h>
#include <SPI.h>
#include "HX711_ADC.h"

// Virtufit Etappe I
//#define DEBUG
//#define BLE_LOGGING
//#define CALIBRATE
#define DISABLE_LOGGING  // to the SD
// Crank length, in meters
#define CRANK_RADIUS 0.1725
#define LOAD_OFFSET 255904.f
#define HX711_MULT  -2466.8989547
#define GYRO_OFFSET -31
// If the wires are hooked up backwards, the force is negated => -1
// If it isn't, just set to 1.
#define HOOKEDUPLOADBACKWARDS 1
#define DEV_NAME "CyclePowerMeter"


// Universal defines

// Milliseconds to wait before go to sleep: 900000 = 15 minutes 
#define MILLIS_TO_SLEEP 900000 

// A7 for Feather (non express)
#define VBATPIN PIN_A6

// The pause for the loop, and based on testing the actual
// calls overhead take about 20ms themselves E.g. at 50ms delay,
// that means a 50ms delay, plus 20ms to poll. So 70ms per loop,
// will get ~14 samples/second.
#define LOOP_DELAY 70

// Min pause How often to crunch numbers and publish an update (millis)
// NOTE If this value is less than the time it takes for one crank
// rotation, we will not report a crank revolution. In other words,
// if the value is 1000 (1 second), cadence under 60 RPM won't register.
#define MIN_UPDATE_FREQ 1500

// NOTE LED is automatically lit solid when connected,
// we don't currently change it, default Feather behavior
// is nice.
// TODO Though not optimal for power, not sure how much it takes.
#define LED_PIN LED_BUILTIN
#define SD_CS_PIN PIN_A3
#define GYRO_INT_PIN A4

// Interrupt related variables (must be volatile)
volatile long timeFirstSleepCheck=0;
volatile long Sleepy=0;

// Bluetooth
uint8_t connection_count = 0;

// NVRAM settings_struct (used by calibration)
int nvram_page_address=0x00024000;

typedef struct settings_struct {
  unsigned char calibrated; 

  int gyro_offset = 0;
  long load_offset = 0;
  float load_multiplier = 0;
} nvram_settings_struct;
nvram_settings_struct nvram_settings;

//HX711 pins:
#define HX711_dout 4 //mcu > HX711 dout pin (from example)
#define HX711_sck 5 //mcu > HX711 sck pin (from example)
static boolean newForceDataReady = 0;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//HX711 EEPROM calibration settings saving/loading
const int calVal_eepromAdress = 0;
unsigned long t = 0;

//MPU6050 constructor:
Adafruit_MPU6050 mpu;


void setup() {
  Wire.begin();

  Serial.begin(115200);
  int cnt=0;
  while ( !Serial && (cnt++ < 300)) delay(10);   // for nrf52840 with native usb

  timeFirstSleepCheck=0;
  Sleepy = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup, calibrate our other components
  gyroSetup();
  loadSetup();
  bleSetup();

  Serial.println("Setup completed");
  Serial.println("");
  Serial.println("Send 'c' from serial monitor to calibrate the load sensor.");
  delay(200);
}

void loop() {
  // Vars for polling footspeed
  static float avgDps = 0.f;
  // Cadence is calculated by increasing total revolutions.
  // TODO it's possible this rolls over, about 12 hours at 90RPM for 16 bit unsigned.
  static uint16_t totalCrankRevs = 0;
  // Vars for force
  static float avgForce = 0.f;

  // We only publish every once-in-a-while.
  static long lastUpdate = millis();
  // Other things (like battery) might be on a longer update schedule for power.
  static long lastInfrequentUpdate = millis();
  // To find the average values to use, count the num of samples
  // between updates.
  static int16_t numPolls = 0;
  bool pedaling = false;

  // During every loop, we just want to get samples to calculate
  // one power/cadence update every interval we update the central.
  
  // Degrees per second
  avgDps = getNormalAvgVelocity();

  // Now get force from the load cell.
  avgForce = getAvgForce();

  numPolls += 1;

#ifdef BLE_LOGGING
  blePublishLog("F%.1f|%.1f %d", force, dps, numPolls);
#endif

#ifdef DEBUG
  // Just print these values to the serial, something easy to read.
  Serial.print(F("Force: ")); Serial.println(avgForce);
  Serial.print(F("DPS:   ")); Serial.println(dps);
#endif  // DEBUG

//  if (Bluefruit.connected()) {
  if (connection_count > 0) {
    // We have a central connected
    long timeNow = millis();
    long timeSinceLastUpdate = timeNow - lastUpdate;
   
    if (timeSinceLastUpdate > updateTime(avgDps, &pedaling) && numPolls > 2) {
      // Convert dps to mps
      float mps = getCircularVelocity(avgDps);

      // That's all the ingredients, now we can find the power.
      int16_t power = calcPower(mps, avgForce);

#ifdef DEBUG
      // Just print these values to the serial, something easy to read.
      Serial.print(F("Pwr: ")); Serial.println(power);
#endif  // DEBUG

      // The time since last update, as published, is actually at
      // a resolution of 1/1024 seconds, per the spec. BLE will convert, just send
      // the time, in millis.
      if (pedaling) {
        totalCrankRevs += 1;
      }
      blePublishPower(power, totalCrankRevs, timeNow);

#ifdef BLE_LOGGING
      // It's not even useful for sending cadence to the computer, ironically.
      int16_t cadence = getCadence(avgDps);
      // Log chars over BLE, for some insight when not wired to a
      // laptop. Need to keep total ASCII to 20 chars or less.
      blePublishLog("B%.1f %.1f %d", avgForce, mps, power);
      blePublishLog("%d: %d polls", millis() / 1000, numPolls);
#endif

      // Reset the latest update to now.
      lastUpdate = timeNow;
      // Let the averages from this polling period just carry over.
      numPolls = 1;

      // And check the battery, don't need to do it nearly this often though.
      // 1000 ms / sec * 60 sec / min * 5 = 5 minutes
      if ((timeNow - lastInfrequentUpdate) > (1000 * 60 * 5)) {
        float batPercent = checkBatt();
        blePublishBatt(batPercent);
        lastInfrequentUpdate = timeNow;
      }
    }
  }

  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 'c') calibrateLoadCell(); //calibrate
  }

  // Pass-through USB/Bluethooth (BLE) data
  bleuart_data_transfer();

  // Should we go to sleep?
  gyroCheckSleepy(pedaling);

  delay(LOOP_DELAY);

// Request CPU to enter low-power mode until an event/interrupt occurs
//  waitForEvent();
}

/**
   Figure out how long to sleep in the loops. We'd like to tie the update interval
   to the cadence, but if they stop pedaling need some minimum.

   Return update interval, in milliseconds.
*/
float updateTime(float dps, bool *pedaling) {
  // So knowing the dps, how long for 360 degrees?
  float del = min(MIN_UPDATE_FREQ, 1000.f * (360.f / dps));
  if (del < MIN_UPDATE_FREQ) {
    // Let the caller know we didn't just hit the max pause,
    // the cranks are spinning.
    *pedaling = true;
  }
  // Because need to account for delay, on average get 1 rotation. Empirically the overhead
  // for calls in the loop is about 20 ms, plus the coded loop delay. If we account for half of that,
  // we should be on average right on 1 rotation. We want to be within half of the overhead time
  // for a perfect 360 degrees.
  return (del - (0.5 * (LOOP_DELAY + 30)));
}

/**
   Given the footspeed (angular velocity) and force, power falls out.

   Returns the power, in watts. Force and distance over time.
*/
int16_t calcPower(double footSpeed, double force) {
  // Multiply it all by 2, because we only have the sensor on 1/2 the cranks.
  return (2 * force * footSpeed);
}

/**

*/
uint8_t checkBatt() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // Board divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  // TODO would be cool to convert to an accurate percentage, but takes
  // some science because I read it's not a linear discharge. For now
  // make it super simple, based off this info from Adafruit:
  //
  // Lipoly batteries are 'maxed out' at 4.2V and stick around 3.7V
  // for much of the battery life, then slowly sink down to 3.2V or
  // so before the protection circuitry cuts it off. By measuring the
  // voltage you can quickly tell when you're heading below 3.7V
  if (measuredvbat > 4.1) {
    return 100;
  } else if (measuredvbat > 3.9) {
    return 90;
  } else if (measuredvbat > 3.7) {
    return 70;
  } else if (measuredvbat > 3.5) {
    return 40;
  } else if (measuredvbat > 3.3) {
    return 20;
  } else {
    return 5;
  }
}
