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

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

  Serial.println("");
  delay(100);

  // Setup sleep
  Sleepy = 1;
  timeFirstSleepCheck=0;

//  attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), motionDetectChange, RISING); 
}

void gyroCheckSleepy(bool pedaling) {
  if (!pedaling) {
    if ((timeFirstSleepCheck > 0) && ((millis() - timeFirstSleepCheck) > MILLIS_TO_SLEEP)) 
    {
      enterSleepMode();
    }
    else
    {
      // Not pedaling => start sleep timer
      if(timeFirstSleepCheck == 0)
      {
        int sleeptime=MILLIS_TO_SLEEP/(1000*60);
        timeFirstSleepCheck = millis();
//        printfLog("No activity. Going for a nap in %d minutes\n",sleeptime);
      }
    }
  }
  else
  {
    // Pedaling => reset sleep timer
    if(timeFirstSleepCheck > 0)
    {
      timeFirstSleepCheck=0;
//      printfLog("Activity detected. Sleep timer reset\n");
    }
  }
}

void enterSleepMode() {
  Bluefruit.autoConnLed(false);
  digitalWrite(LED_CONN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  printfLog("Going to low-power mode..\n");
  delay(1000);

  // Power down loadcell
  LoadCell.powerDown();

  // Set zero motion detection at gyro MPU6050
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(false);  
  mpu.setInterruptPinPolarity(false); // active high
  mpu.setMotionInterrupt(true);

  // Enable wake-up by motion interrupt
  pinMode(GYRO_INT_PIN, INPUT_PULLUP);
  nrf_gpio_cfg_input(2,NRF_GPIO_PIN_NOPULL); 
  nrf_gpio_cfg_sense_input(2, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
  sd_power_system_off();
}

/**
 * Gets a normalized averaged rotational velocity calculation. The MPU6050 library supports a
 * normalized gyroscope reading, which trims off outliers and scales the values to deg/s.
 *
 * An exponential average is applied to further smooth data, with weight of WEIGHT. I don't
 * love this, becaues it means no value is every entirely discarded, but exponential decay
 * probably makes it effectively the same. Maybe something to revisit.
 *
 * Returns a value for foot speed, in rad/second.
 */
float getNormalAvgVelocity() {
  float rotz = getZrot();
  if (rotz < (PI/4)) {
    // Magic number (45 degrees) here, but less than 45 dps is less than 1 crank rotation 
    // in 4 seconds (7 RPM), just assume it's noise from the road bumps.
    rotz = 0.f;
  }

  return rotz;
}

float getZrot() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

//  printfLog("%d %d %d\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);

  return abs(g.gyro.z);
}

// TODO: Implement QUATERNION routines (for 360 degrees Ztilt instead of +90/-90) from here:
//       https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino

/*       -> Include gravity (in #else statement)

#ifdef USE_OLD_DMPGETYAWPITCHROLL
uint8_t MPU6050_6Axis_MotionApps20::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}
#else 
uint8_t MPU6050_6Axis_MotionApps20::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);
    if (gravity -> z < 0) {
        if(data[1] > 0) {
            data[1] = PI - data[1]; 
        } else { 
            data[1] = -PI - data[1];
        }
    }
    return 0;
}
#endif
*/
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

