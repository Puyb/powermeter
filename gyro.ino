/**
 * MPU6050/gyroscope specific code. Initialize, helpers to do angular math.
 *
 * TODO big improvements I think to use the sleep mode and interrupts provided
 * by this MPU.
 */
#include <Arduino_nRF5x_lowPower.h> // LowPower Library for nRF5x
#include <wiring.h>

#define BOARD_PIN_A4 = 2 // see pin mapping below
#define CALIBRATION_SAMPLES 40
volatile int WakeupCount=0;

/*    NRF52840 pin mapping

  // D0 .. D13
  25,  // D0  is P0.25 (UART TX)
  24,  // D1  is P0.24 (UART RX 
  10,  // D2  is P0.10 (NFC2)
  47,  // D3  is P1.15 (LED1)
  42,  // D4  is P1.10 (LED2)
  40,  // D5  is P1.08
   7,  // D6  is P0.07
  34,  // D7  is P1.02 (Button)
  16,  // D8  is P0.16 (NeoPixel)
  26,  // D9  is P0.26
  27,  // D10 is P0.27
   6,  // D11 is P0.06
   8,  // D12 is P0.08
  41,  // D13 is P1.09

  // D14 .. D21 (aka A0 .. A7)
   4,  // D14 is P0.04 (A0)
   5,  // D15 is P0.05 (A1)
  30,  // D16 is P0.30 (A2)
  28,  // D17 is P0.28 (A3)
   2,  // D18 is P0.02 (A4)
   3,  // D19 is P0.03 (A5)
  29,  // D20 is P0.29 (A6, Battery)
  31,  // D21 is P0.31 (A7, ARef)

  // D22 .. D23 (aka I2C pins)
  12,  // D22 is P0.12 (SDA)
  11,  // D23 is P0.11 (SCL)

  // D24 .. D26 (aka SPI pins)
  15,  // D24 is P0.15 (SPI MISO)
  13,  // D25 is P0.13 (SPI MOSI)
  14,  // D26 is P0.14 (SPI SCK )

  // QSPI pins (not exposed via any header / test point)
  19,  // D27 is P0.19 (QSPI CLK)
  20,  // D28 is P0.20 (QSPI CS)
  17,  // D29 is P0.17 (QSPI Data 0)
  22,  // D30 is P0.22 (QSPI Data 1)
  23,  // D31 is P0.23 (QSPI Data 2)
  21,  // D32 is P0.21 (QSPI Data 3)

  // The remaining NFC pin
   9,  // D33 is P0.09 (NFC1, exposed only via test point on bottom of board)

  // Thus, there are 34 defined pins

  // The remaining pins are not usable:
  //
  //
  // The following pins were never listed as they were considered unusable
  //  0,      // P0.00 is XL1   (attached to 32.768kHz crystal)
  //  1,      // P0.01 is XL2   (attached to 32.768kHz crystal)
  // 18,      // P0.18 is RESET (attached to switch)
  // 32,      // P1.00 is SWO   (attached to debug header)
  // 
  // The remaining pins are not connected (per schematic)
  // 33,      // P1.01 is not connected per schematic
  // 35,      // P1.03 is not connected per schematic
  // 36,      // P1.04 is not connected per schematic
  // 37,      // P1.05 is not connected per schematic
  // 38,      // P1.06 is not connected per schematic
  // 39,      // P1.07 is not connected per schematic
  // 43,      // P1.11 is not connected per schematic
  // 44,      // P1.12 is not connected per schematic
  // 45,      // P1.13 is not connected per schematic
  // 46,      // P1.14 is not connected per schematic
*/

/**
 *  Calibrate and initialize the gyroscope
 */
void gyroSetup() {
  // "gyro" is defined in main, Arduino implicitly smashes these files together
  // to compile, so it's in scope.
  timeFirstSleepCheck=0;
  
  gyro.initialize();
  // Set to +/- 1000dps.
  gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

#ifdef DEBUG
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(gyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif // DEBUG

// TODO make a calibration mode. For now, test manually.
// 5 tests sitting at the kitchen table, offsets were   -39 -39 -39 -38 -38
#ifdef CALIBRATE
  // Calibrate the gyro
  gyro.setZGyroOffset(0);
  float sumZ = 0;
  int16_t maxSample = -32768;
  int16_t minSample = 32767;
  // Read n-samples
  for (uint8_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    delay(5);
    int16_t reading = gyro.getRotationZ();
    if (reading > maxSample) maxSample = reading;
    if (reading < minSample) minSample = reading;
    sumZ += reading;
  }

  // Throw out the two outliers
  sumZ -= minSample;
  sumZ -= maxSample;

  // Two fewer than the calibration samples because we took out two outliers.
  float deltaZ = sumZ / (CALIBRATION_SAMPLES - 2);
  deltaZ = -1 * deltaZ;
#ifdef DEBUG
  Serial.printf("Discounting max (%d) and min (%d) samples.\n", maxSample, minSample);
  Serial.printf("Gyro calculated offset: %f\n", deltaZ); 
#endif // DEBUG
#endif // CALIBRATE

  // In lieu of being able to store results from a calibration mode...
#ifndef CALIBRATE
  float deltaZ = GYRO_OFFSET;
#endif // CALIBRATE

  // Set that calibration
  gyro.setZGyroOffset(deltaZ);

  // Set zero motion detection
  gyro.setIntZeroMotionEnabled(true);
  gyro.setZeroMotionDetectionThreshold(4);
  // 1 LSB = 64ms. So 30s = 
  gyro.setZeroMotionDetectionDuration(80);
  gyro.setInterruptLatchClear(true);

  pinMode(GYRO_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), motionDetectChange, RISING);

#ifdef DEBUG
  dumpSettings();
#endif
}

void gyroCheckSleepy() {
//  int button;

//  pinMode(PIN_DFU, INPUT);
//  button=digitalRead(PIN_DFU);
//  Serial.printf("Button: %d\n", button);

  if ((timeFirstSleepCheck > 0) && ((millis() - timeFirstSleepCheck) > MILLIS_TO_SLEEP)) 
  {
    detachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN));
    Bluefruit.autoConnLed(false);
    digitalWrite(LED_CONN, LOW);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.printf("Going to low-power mode.. (wakeups = %d)\n", WakeupCount);
    delay(1000);

//      prepare_MPU6050_for_sleep();

//      gyro.reset();
//    gyro.resetGyroscopePath();
//    gyro.initialize();
//      gyro.setAccelXSelfTest();
//    gyro.setAccelerometerPowerOnDelay(3);

/*    gyro.setInterruptMode(1); // (0=active-high, 1=active-low)
    gyro.setInterruptDrive(0); // (0=push-pull, 1=open-drain)
    gyro.setInterruptLatch(0); // (0=50us-pulse, 1=latch-until-int-cleared)
    gyro.setInterruptLatchClear(1); // (0=status-read-only, 1=any-register-read)
    gyro.setIntEnabled(0);
    gyro.setIntMotionEnabled(true);
    gyro.setDHPFMode(1); // filtermode: 1 (5Hz)
    gyro.setMotionDetectionThreshold(1);
    gyro.setMotionDetectionDuration(40);

    pinMode(GYRO_INT_PIN, INPUT); */
//    attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), WakeUp, RISING);
//    nRF5x_lowPower.enableWakeupByInterrupt(GYRO_INT_PIN, RISING);
    nrf_gpio_cfg_input(2,NRF_GPIO_PIN_NOPULL); //Configure button as input
    nrf_gpio_cfg_sense_input(2, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
    sd_power_system_off();
//    NRF_POWER->SYSTEMOFF = 1; // SYSTEM OFF mode #1
//    nRF5x_lowPower.powerMode(POWER_MODE_OFF);
  }
}

void WakeUp() {
    WakeupCount++;
}

void motionDetectChange() {
  uint8_t motion = gyro.getMotionStatus();

  Serial.printf("Motion detected: %d\n", motion);

  if (motion) {
    // motion to zero-motion detected
    if (!Sleepy) {
      Sleepy = 1;
      timeFirstSleepCheck=millis();
      Serial.println("No motion detected. Low-power mode will be activated after timeout.");
    }
  } else {
    // zero-motion to motion detected
    Serial.println("Wakey wakey let's get crankey.");

    // Setup, calibrate our other components
    timeFirstSleepCheck=0;
    Sleepy = 0;
  }
}

/**
 * This doesn't do anything but echo applied setting on the MPU.
 */
void dumpSettings() {
  Serial.println();
  Serial.printf(" * Gyroscope Sleep Mode: %d\n", gyro.getSleepEnabled() ? "Enabled" : "Disabled");
  Serial.printf(" * Gyroscope offset:     %d\n", gyro.getZGyroOffset());
}

/**
 * Gets a normalized averaged rotational velocity calculation. The MPU6050 library supports a
 * normalized gyroscope reading, which trims off outliers and scales the values to deg/s.
 *
 * An exponential average is applied to further smooth data, with weight of WEIGHT. I don't
 * love this, becaues it means no value is every entirely discarded, but exponential decay
 * probably makes it effectively the same. Maybe something to revisit.
 *
 * Returns a value for foot speed, in degrees/second.
 */
float getNormalAvgVelocity(const float & lastAvg) {
  const static double WEIGHT = 0.90;
  // At +/- 250 degrees/s, the LSB/deg/s is 131. Per the mpu6050 spec.
  /* FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  */
  const static float SENSITIVITY = 32.8;

  // Request new data from the MPU. The orientation obviously dictates
  // which x/y/z value we're interested in, but right now Z.
  // Use the absolute value. Cause who knows if the chip is just backwards.
  float rotz = abs(gyro.getRotationZ() / SENSITIVITY);
  if (rotz < 90) {
    // Magic number here, but less than 90 dps is less than 1 crank rotation 
    // in 4 seconds (15 RPM), just assume it's noise from the road bumps.
    rotz = 0.f;
  }
  // Return a rolling average, including the last reading.
  // e.g. if weight is 0.90, it's 10% what it used to be, 90% this new reading.
  float newavg = (rotz * WEIGHT) + (lastAvg * (1 - WEIGHT));

  return newavg;
}

/**
 * Provide the average rate, in degrees/second.
 *
 * Returns the circular velocity of the rider's foot. Takes in the crank's averaged rotational
 * velocity, converts it to radians, multiplies by the crank radius, and returns the converted
 * value.
 *
 * Value returned is in meters/second
 */
float getCircularVelocity(float dps) {
  // 2 * PI * radians = 360 degrees  -- by definition
  // dps degrees/second * (PI / 180) rad/degree = rad/second
  // (rad/sec) / (rad/circumference) = (rad/sec) / 2 * PI = ratio of a circumference traveled, still per second
  // 2 * PI * CRANK_RADIUS = circumference  -- by definition, that's a circle
  // ratio of circumference traveled * circumference = meters/second

  // It all comes out to:
  // m/s = ((dps * (PI / 180)) / (2 * PI)) * (2 * PI * CRANK_RADIUS);
  // And simplifies to:
  return (dps * PI * CRANK_RADIUS) / 180;
}

/**
 *  Provide angular velocity, degrees/sec.
 *
 *  Returns a new cadence measurement.
 *
 *  Note this isn't necessary for power measurement, but it's a gimme addon
 *  given what we already have and useful for the athlete.
 *
 *  Returns an int16 of cadence, rotations/minute.
 */
int16_t getCadence(float dps) {
  // Cadence is the normalized angular velocity, times 60/360, which
  // converts from deg/s to rotations/min. x * (60/360) = x / 6.
  return dps / 6;
}

/**
 *  Determine current angle of the crank arm. Based on the acceleration
 *  for gravity.
 */
int16_t getAngle() {
  // Sensitivity for 2g is 48
  static const int16_t SENS = 48;

  // TODO not certain how to do this yet. If we calibrate on the fly to
  // get known values, we have to worry about the orientation of the cranks
  // when that's done. We could calibrate as a 1-time thing but that's less
  // preferable because.. what if it drifts? If we don't calibrate, that
  // could still be ok, because what we really want is to know the "peaks",
  // min and max. Those are when the cranks are perpendicular to the ground.
  // And the mins are straight up and down. Downside there is that the max
  // values will change with the acceleration of cadence, so we'd have to
  // almost continuously figure out what they are?

  // For now, just return the raw X acceleration.
  return gyro.getAccelerationX() / SENS;
}


