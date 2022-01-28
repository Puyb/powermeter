/**
 * MPU6050/gyroscope specific code. Initialize, helpers to do angular math.
 *
 * TODO big improvements I think to use the sleep mode and interrupts provided
 * by this MPU.
 */
#include <wiring.h>
#include "board.h"

#define CALIBRATION_SAMPLES 40
volatile int WakeupCount=0;


/**
 *  Calibrate and initialize the gyroscope
 */
void gyroSetup() {
  // "gyro" is defined in main, Arduino implicitly smashes these files together
  // to compile, so it's in scope.
  timeFirstSleepCheck=0;

  // Try to initialize! (address = 0x68)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found");

  // disable 'wakeup' interupt
  mpu.setMotionInterrupt(false);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  mpu.setHighPassFilter(MPU6050_HIGHPASS_UNUSED);
  mpu.setClock(MPU6050_INTR_8MHz); // Keep clock running on internal clock 


  Serial.println("");
  delay(100);

  // Setup sleep
  timeFirstSleepCheck=0;

//  attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), motionDetectChange, RISING); 
}

void gyroCheckSleepy(bool pedaling) {
  if (!pedaling) {
    if ((timeFirstSleepCheck > 0) && ((millis() - timeFirstSleepCheck) > MILLIS_TO_SLEEP)) enterSleepMode();

    // Not pedaling => start sleep timer
    else if(timeFirstSleepCheck == 0) timeFirstSleepCheck = millis();
  }
  // Pedaling => reset sleep timer
  else if(timeFirstSleepCheck > 0) timeFirstSleepCheck=0;
}

void enterSleepMode() {
  Bluefruit.autoConnLed(false);
  digitalWrite(LED_CONN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  printfLog("Going to low-power mode..\n");
  delay(1000);

  // Power down loadcell
  LoadCell.powerDown();

  // Put MPU6050 in low power (wild guess, to be measured)
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_UNUSED);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); ///< Docs imply this disables the filter

  // Set sample rate = GYROSCOPE Sample Rate / (1 + SampleRateDivisor)
  //                 = 8kHz/(1 + 999) = 8 Hz
  mpu.setSampleRateDivisor(999);


 /*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7 RESERVED
 */


  // Cycling-mode and/or disabling non-used axis seems incompatible with the motion interrupt
/*  
  mpu.setClock(MPU6050_INTR_8MHz); // Keep clock running on internal clock 
  mpu.setCycleRate(MPU6050_CYCLE_1_25_HZ); 
  mpu.enableSleep(false);
  mpu.enableCycle(true);
  mpu.disableTemp(true);
  mpu.enableStandby(STBY_XA + STBY_YA + STBY_XG + STBY_YG); // STBY_XA, STBY_YA, STBY_ZA, STBY_XG, STBY_YG, STBY_ZG
*/

  // Set zero motion detection interrupt at gyro MPU6050
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(false);  
  mpu.setInterruptPinPolarity(false); // active high
  mpu.setMotionInterrupt(true);

  // Enable wake-up by motion interrupt and power-down the Arduino board
  pinMode(GYRO_INT_PIN, INPUT_PULLUP);
  nrf_gpio_cfg_input(2,NRF_GPIO_PIN_NOPULL); 
  nrf_gpio_cfg_sense_input(2, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
  sd_power_system_off();
}

float getZrot() {
  static float zrot_prev=0;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  if (g.gyro.z != zrot_prev) {
    zrot_prev = g.gyro.z;
    newZrotDataReady++;
  }

//  printfLog("%d %d %d\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);

  return abs(g.gyro.z); // should this be abs????
}

void getZtilt(float *roll, float *z) {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  double x_Buff = float(a.acceleration.x);
  double y_Buff = float(a.acceleration.y);
  double z_Buff = float(a.acceleration.z);
  *roll = atan2(y_Buff , z_Buff) * 57.3;
  *z = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
}

float getTemperature() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);
  return(temp.temperature);
}

