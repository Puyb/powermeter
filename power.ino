/**
   Main
   ====

  Copy binary to output (dist) folder:
  copy /Y ..\output\power.ino.zip dist
*/

#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include "board.h"
#include <Wire.h>
#include <bluefruit.h>
#include <SPI.h>
#include "HX711_ADC.h"


#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_TinyUSB.h> // for Serial

//#define DEBUG

#define DEV_NAME "Cycle Power Meter"
#define NRF52840_XXAA
#define gn 9.80665 // gravity constant

// Virtufit Etappe I: crank length, in meters
// (to be added to the calibration procedure!)
#define CRANK_RADIUS 0.1725

// Milliseconds to wait before go to sleep: 900000 = 15 minutes 
#define MILLIS_TO_SLEEP 900000 

// HX711 on-board hardware switch default is 10 Hz (alternative: 80 Hz)
#define HX711_RATE 10 

// The window in which the crank position (tilt and roll) is assumed to be located in the 'measuring position'
// (crank with measurement device horizontal to the front) in rotational degrees
#define CRANK_POSITION_WINDOW 10

// To check if we reached the opposite position of the measuring position 
// for this we allow a 90 degrees (+/-45) detection window, because precision doesn't matter here
#define CRANK_OPP_POSITION_WINDOW 90

// If the number of radians per seconds is less than this, we assume the user stopped pedaling
#define STAND_STILL_RPS (0.25 * PI)

// Pin-outs
#define LED_PIN LED_BUILTIN
#define SD_CS_PIN PIN_A3
#define GYRO_INT_PIN A4

// Interrupt related variables (must be volatile)
volatile long timeFirstSleepCheck=0;
volatile long Sleepy=0;
volatile long connectedStart=0;
volatile boolean newLoadDataReady=0;
volatile uint8_t connection_count = 0; // bluetooth connection count
volatile int last_connection_count=0; // for automatically printing the help text

// Bluetooth
#define PWR_MEAS_CHAR_LEN 8 // Bluetooth package length
bool show_values=false; // print raw values
int16_t test_power=0; // for testing
uint16_t test_totalCrankRev=0; // for testing
uint16_t test_totalCrankRev_inc=0; // for testing

#define LOAD_OFFSET_DEFAULT 8745984
#define LOAD_MULTIPLIER_DEFAULT 810.1
#define CALIBRATIONS_FILENAME    "/calibrations.txt"

typedef struct settings_struct {
  unsigned char calibrated; 

  int gyro_offset = 0;
  long load_offset = 0;
  float load_multiplier = 0;
} nvram_settings_struct;
nvram_settings_struct nvram_settings;

//HX711 pins:
#define HX711_dout A0 //mcu > HX711 dout pin (was: 4)
#define HX711_sck A1 //mcu > HX711 sck pin (was: 5)

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//HX711 EEPROM calibration settings saving/loading
const int calVal_eepromAdress = 0;
unsigned long t = 0;

//MPU6050 constructor:
Adafruit_MPU6050 mpu;

//Internal filesystem
using namespace Adafruit_LittleFS_Namespace;
File file(InternalFS);

// Session stats
static long lastSessionStart;
static long lastSessionEnd = 0;
static long lastSessionTotalCount = 0;
static float lastSessionTotalPower = 0;
static uint16_t totalCrankRevs = 0; // TODO it's possible this rolls over, about 12 hours at 90RPM for 16 bit unsigned.


//
// Setup
//
void setup() {
  Wire.begin();

  Serial.begin(115200);
  int cnt=0;
  while ( !Serial && (cnt++ < 300)) delay(10);   // for nrf52840 with native usb

  timeFirstSleepCheck=0;
  Sleepy = 0;
  lastSessionStart = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup, calibrate our other components
  gyroSetup();
  loadSetup();
  bleSetup();
  setupBattery();

  Serial.printf("Setup completed.\n\n");
  Serial.printf("Enter 'h' for help.\n\n");
  delay(200);
}

//
// Main loop
//
void loop() {
  static bool oppositeOfMeasurementPositionReached = false;

  // Moving average of the velocity in rad/second
  float avgRad = 0;

  // Vars for force
  static float avgForce = 0.f;

  // We only publish every once-in-a-while.
  static long lastMeasurement = millis();
  static long lastStopMessage = millis();

  // Other things (like battery) might be on a longer update schedule for power.
  static long lastInfrequentUpdate = millis();

  bool pedaling = false;
  float Zroll, Ztilt; 

  // Get moving average velocity in rad per second
  avgRad = MA_cadence(getZrot());

  // Get the crank Z position
  getZtilt(&Zroll, &Ztilt);
  
  // Check if we stopped pedaling every 2000 ms
  if ((avgRad < STAND_STILL_RPS) &&
      ((millis() - lastStopMessage) >= 2000)) {
    // Reset timer
    lastStopMessage = millis();

    // Only set end-time if we were cycling
    if(lastSessionEnd == 0) lastSessionEnd = millis();

    // Reset timer to prevent false error message when we start pedaling again
    lastMeasurement = millis();

    // We are not pedaling. Report this to the bluetooth host
    blePublishPower(0, totalCrankRevs, millis()); // zero power, no cadence (resend same totalCrankRevs)
    if (show_values) {
        printfLog("%.1fN * %.2fm/s = %dW (Z=%0.0f/%0.0f, STOP)\n", getAvgForce(), CRANK_RADIUS * avgRad, 0, Ztilt, Zroll);
    }

    // Input for sleep timer
    pedaling = false;
  }
  // We are pedaling => Check if we reached the opposite position of the measuring position 
  // for this we allow a large detection window, because precision doesn't matter here
  else if ((Ztilt > 0-(CRANK_OPP_POSITION_WINDOW/2)) && (Ztilt < 0+(CRANK_OPP_POSITION_WINDOW/2)) && 
            (Zroll > 90-(CRANK_OPP_POSITION_WINDOW/2)) && (Zroll < 90+(CRANK_OPP_POSITION_WINDOW/2))) {
    oppositeOfMeasurementPositionReached = true;  
  }
  // We are pedaling => Check if we reached the measuring position. We apply a small detection window for highest precision 
  else if (oppositeOfMeasurementPositionReached &&
           (Ztilt > 0-(CRANK_POSITION_WINDOW/2)) && (Ztilt < 0+(CRANK_POSITION_WINDOW/2)) && 
           (Zroll > -90-(CRANK_POSITION_WINDOW/2)) && (Zroll < -90+(CRANK_POSITION_WINDOW/2))) {  

    // Reset the timer
    lastMeasurement = millis();

    // Wait for full rotation after this measurement
    oppositeOfMeasurementPositionReached = false;  

    // We are pedaling. Update the counter for the bluetooth host
    pedaling = true;
    totalCrankRevs++;

    // Reset end-time
    lastSessionEnd=0;

    // Get the moving average force from the load cell (library)
    avgForce = getAvgForce();

    // Get the circular velocity of the rider's foot in m/s
    float mps = CRANK_RADIUS * avgRad; 

    // Multiply it all by 2, because we only have the sensor on 1/2 the cranks
    int16_t power = 2 * mps * avgForce;

    if((test_power>0) || (test_totalCrankRev_inc>0))
    {
      test_totalCrankRev += test_totalCrankRev_inc;
      blePublishPower(test_power, test_totalCrankRev, millis());
      printfLog("Fake: Force=%d  Cad=%d\n", test_power, test_totalCrankRev);
    }
    else
    {
      if (show_values) {
          printfLog("%.1fN, %.2fm/s, %dW, %0.0f/%0.0f, %d\n", avgForce, mps, power, Ztilt, Zroll,totalCrankRevs);
      }
      blePublishPower(power, totalCrankRevs, millis());

      lastSessionTotalPower += power;
      lastSessionTotalCount++;
    }
  }
  // If the pedals are moving, check if we missed too many measurement positions
  else if ((avgRad >= STAND_STILL_RPS) && ((millis() - lastMeasurement) >= 8000)) {
    // Reset timer 
    lastMeasurement = millis();

    // Report ERROR situation
    printfLog("ERROR: Pedaling but no measurement position detected within 8 seconds. Try to increase CRANK_POSITION_WINDOW (hard-coded).\n");

    printfLog("%.1fN, %.1frad/s, Z=%0.0f/%0.0f\n", getAvgForce(), getZrot(), Ztilt, Zroll);
  }

  // Print help text on bluetooth connection
  printHelpOnConnect();

  // Check the battery: don't need to do it nearly this often though.
  // 1000 ms / sec * 60 sec / min * 5 = 5 minutes
  if ((millis() - lastInfrequentUpdate) > (1000 * 60 * 5)) {
    blePublishBatt();
    lastInfrequentUpdate = millis();
  }

  // Check if we can go to sleep
  gyroCheckSleepy(pedaling);

  // Read user input
  readUserInput();
  
  // Request CPU to enter low-power mode until an event/interrupt occurs
  // but mind this bug: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/637
  waitForEvent();
}

// Print help text after an arbitrary wait time (to allow the user to press UART in the App)
void printHelpOnConnect() {
  if (connection_count > 0) {
    if(connection_count != last_connection_count) {
      if (connectedStart == 0) connectedStart = millis();
      if((millis() - connectedStart) > (1000*6))
      {
        last_connection_count = connection_count;
        connectedStart = 0;

        printHelp();
      }
    }
  }
  else
  {
    last_connection_count = 0;
  }
}

void lastSessionStats() {
  float duration_min = (lastSessionEnd - lastSessionStart) / (1000.f * 60.f);
  if ((duration_min > 0) && (lastSessionTotalCount > 0) && (totalCrankRevs > 0)) {
    printfLog("Session stats (since last restart):\n");
    printfLog("Duration: %.0f minutes\n", duration_min);
    printfLog("   Power: %.1f W\n", lastSessionTotalPower / lastSessionTotalCount);
    printfLog(" Cadence: %.1f rpm (incl. stops)\n\n", totalCrankRevs / duration_min);
  }
  else {
    printfLog("No stats available yet.\n\n");
  }
}

void printHelp() {
  printfLog("=================\n");
  printfLog("Power Cycle Meter\n");
  printfLog("=================\n\n");

  if (nvram_settings.load_offset == LOAD_OFFSET_DEFAULT) {
    printfLog("\nLoad-cell defaults loaded:\n");
  }
  printfLog("    Load offset cal: %d\n",nvram_settings.load_offset);
  printfLog("Load multiplier cal: %.1f\n\n",nvram_settings.load_multiplier); 

  printfLog("        Temperature: %.1f Celsius\n", getTemperature());
  blePublishBatt(); // Broadcast and show battery level 

  printfLog("Commands:\n");
  printfLog(" h : show this Help text\n");
  printfLog(" l : Show last session stats\n");
  printfLog(" m : Monitor power & cadence\n");
  printfLog(" f : Fake power & cadence\n");
  printfLog(" c : Calibrate load sensor\n");
  printfLog(" s : enter Sleep mode\n");
  printfLog("\n");
}

//
// Read user input (via serial and/or bluetooth)
//
void readUserInput() {
  char buf[64]={'\0'};
  GetUserInput(buf);
  if (buf[0] == 'c') calibrateLoadCell();
  if (buf[0] == 's') enterSleepMode();
  if (buf[0] == 'h') printHelp(); 
  if (buf[0] == 'l') lastSessionStats(); 
  if (buf[0] == 'f') {
    if (test_power > 0) {
      test_power = 0;
      test_totalCrankRev = 0;
      test_totalCrankRev_inc = 0;
    }
    else {
      testBT(); //test bluetooth
    }
  }
  if (buf[0] == 'm') {
    if (show_values) {
      show_values = false;
    }
    else {
      show_values = true;
    }
  }
}

float MA_cadence(float value) {
  const byte nvalues = 64;            // Arbitrary number for the moving average window size

  static byte current = 0;            // Index for current value
  static byte cvalues = 0;            // Count of values read (<= nvalues)
  static float sum = 0;               // Rolling sum
  static float values[nvalues];

  sum += value;

  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    sum -= values[current];

  values[current] = value;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return sum/cvalues;
}