/**
 * Force-sensor calibration routines
 *
 */

/* This is an example sketch on how to find correct calibration factor for your HX711:

Scale Multiplier
I hung nothing on the pedal (except the light rope used to hang other weights) and tared to get a tare offset of 275010. Then started hanging weights, it was already zero'd.
Don't worry about the multiplier or anything, just hang a known weight and record what the scale says. For example... Hanging 5kg we got about 114600. And at 15kg it read around 343300. Note these both varied by hundreds one second to the next, but those seemed to be roughly the median, watching numbers tick by.
For watts we actually need newtons, which are kg/m/s^2. With gravity, that's 9.8 newtons per kg.
So y = mx + b.
5kg (49 N) 
5 * 9.8 = m * 114600 + 0 
49 = m * 114600 
49/114600 = m 
0.00042757417 = m \
15kg
15 * 9.8 = m * 343300 + 0 
147 = m * 343300 
147 / 343300 = m 
0.00042819691 = m \
So we'll go with a load multiplier of 0.0004278. The HX711 library divides by this multiplier, not multiplies, so we need to put it under 1.
1 / 0.0004278 = 2337.541

Scale Tare
Just tare the load cell and mark what offset it gets.

Gyro Tare
Same idea, run the programmed calibration and see what offset it comes up with.

*/


#define NUM_PIXELS 1

Adafruit_NeoPixel pixel(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB);


// Check if calibration-button is pressed
bool DFU_button_pressed() {
  if(digitalRead(BOARD_PIN_DFU) == LOW) {
    digitalWrite(LED_BUILTIN, LOW);

    // Wait for calibration-button to be released
    while(digitalRead(BOARD_PIN_DFU) == LOW) {
    }
    digitalWrite(LED_BUILTIN, HIGH);
    return true;
  }
  else {
    return false;
  }
}

void neopixel_flash(int n) {
  pixel.begin();

  for (int i = 0; i < n; i++) {
    pixel.setPixelColor(0, 0, 0, 0);
    pixel.show();
    delay(500);
    pixel.setPixelColor(0, 255, 0, 0);
    pixel.show();
    delay(100);
  }
}

void calibrateSetup() {
  // read settings from NVRAM
 
//  memcpy(&nvram_settings, NVRAM_SETTINGS_PAGE_ADDR, sizeof(nvram_settings));
  if(nvram_settings.calibrated == 0xff) {
    Serial.println("Please calibrate first");
  }
}

void calibrate_load_sensor() {
  int phase = 1;
  float scale5 = 0;
  float scale15 = 0;
  float m5 = 0;
  float m15 = 0;

  Serial.println("Starting calibration..");

  neopixel_flash(1);
  load.tare(50);
  Serial.println("Phase 1: No weight on the pedal");
  
  while (!DFU_button_pressed()) {  
    nvram_settings.load_offset = load.get_offset();
    Serial.printf("Tare value: %d | Press button if ready | Reset to cancel\n", nvram_settings.load_offset);
    delay(500);
  }
  load.tare(nvram_settings.load_offset);

  neopixel_flash(2);
  Serial.println("Phase 2: 5 kg weight on the pedal");
  
  while (!DFU_button_pressed()) {  
    // Get current load
    scale5 = load.get_scale();
    Serial.printf("Scale value (5 kg): %d | Press button if ready | Reset to cancel\n", scale5);
    delay(500);
  }

  neopixel_flash(3);
  Serial.println("Phase 3: 15 kg weight on the pedal");
  
  while (!DFU_button_pressed()) {  
    // Get current load
    scale15 = load.get_scale();
    Serial.printf("Scale value (15 kg): %d | Press button if ready | Reset to cancel\n", scale15);
    delay(500);
  }

  m5=5*9.8/scale5;
  m15=15*9.8/scale15;
  nvram_settings.load_multiplier = 1 / ((m5+m15)/2);

  Serial.printf("Tare (load_offset) value = %f\n", nvram_settings.load_offset);
  Serial.printf("Load multiplier for HX711 library = %f\n", nvram_settings.load_multiplier);

//  nrfx_nvmc_page_erase(NVRAM_SETTINGS_PAGE_ADDR);
//  nrfx_nvmc_bytes_write((unsigned char *)&nvram_settings, NVRAM_SETTINGS_PAGE_ADDR, sizeof(nvram_settings));

  Serial.println("Calibration finished.");
  delay(5000);
}
