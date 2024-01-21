#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

uint32_t vbat_pin = PIN_VBAT;             // A7 for feather nRF52832, A6 for nRF52840

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

// For XIAO BLE
#define VBAT_DIVIDER      (0.337748344F)  // 510k + 1M voltage divider on VBAT (510k / (1M + 510k)
#define VBAT_DIVIDER_COMP (2.960784314F)  // Compensation factor for the VBAT divider

// For nRF52832
//#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
//#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


float readVBAT(void) {
  float raw;

  // enable VBAT reading (specific to XIAO BLE, see https://wiki.seeedstudio.com/XIAO_BLE/#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging )
  digitalWrite(VBAT_ENABLE, LOW);

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(10);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // disable VBAT reading (specific to XIAO BLE, see https://wiki.seeedstudio.com/XIAO_BLE/#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging )
  digitalWrite(VBAT_ENABLE, HIGH);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

uint8_t mvToPercent(float mvolts) {
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600) {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}

void setupBattery() {
  readVBAT();
}
