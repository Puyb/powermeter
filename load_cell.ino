/**
 * Force and load cell-specific code and helpers. HX711 chip.
 */
// This offset value is obtained by calibrating the scale with known
// weights, currently manually with a separate sketch.

// It's 'm' as in:
// 'y = m*x + b'
// where 'y' is Newtons (our desired answer, kilograms at
// acceleration of gravity), 'x' is the raw reading of the load
// cell, and 'b' is the tare offset. So this multiplier is the
// scale needed to translate raw readings to units of Newtons.
// (defined in main file)

void loadSetup() {
  Serial.println();
  Serial.println("Starting-up loadcell HX711...");

  LoadCell.begin();
  InternalFS.begin();

  file.open(CALIBRATIONS_FILENAME, FILE_O_READ);
  if ( file )
  {
    uint32_t readlen;
    readlen = file.read((char*)&nvram_settings, sizeof(nvram_settings));
    file.close();

    if(nvram_settings.load_multiplier == 0) nvram_settings.load_multiplier = 640.0;

    LoadCell.setCalFactor(nvram_settings.load_multiplier);
    LoadCell.setTareOffset(nvram_settings.load_offset);          

    Serial.printf("Calibrations found.\n");
    Serial.printf("Load offset set to: %d\n",nvram_settings.load_offset);
    Serial.printf("Load multiplier set to: %f\n",nvram_settings.load_multiplier); 
  } else
  {
    Serial.printf("No calibrations found!\n");
  }

  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

//interrupt routine:
void dataReadyISR() {
  if (LoadCell.update()) {
    newLoadDataReady = 1;
  }
}

/**
 * Get the current force from the load cell. Returns an exponentially
 * rolling average, in Newtons.
 */
float getAvgForce() {
  static float currentData = 0;

  if (newLoadDataReady) {
      currentData = abs(LoadCell.getData());
  }

  return (currentData);
}
