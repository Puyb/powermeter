/**
 * Load sensor calibration routines
 */

void testBT() {
  bool _resume = false;
  char buf[64]={'\0'};
    
  printfLog("Enter power value [0-400]:\n");

  _resume = false;
  while (_resume == false) {
    GetUserInput(buf);
    if (buf[0] != '\0') {
      test_power = atoi(buf);
      if (test_power != 0) {
        printfLog("Power set to: %d\n\n",test_power);
        _resume = true;
      }
    }
  }

  printfLog("Enter crank revs increase value [1-5]:\n");

  _resume = false;
  while (_resume == false) {
    GetUserInput(buf);
    if (buf[0] != '\0') {
      test_totalCrankRev_inc = atoi(buf);
      if (test_totalCrankRev_inc != 0) {
        printfLog("Crank revs increase set to: %d\n\n",test_totalCrankRev_inc);
        _resume = true;
      }
    }
  }
}

void calibrateLoadCell() {
  char buf[64]={'\0'};

  detachInterrupt(digitalPinToInterrupt(HX711_dout));

  printfLog("***\n");
  printfLog("Start calibration:\n");
  printfLog("Step of the bike to ensure\n");
  printfLog("there's no load on the pedals.\n\n");
  printfLog("Then send 't' to set the tare offset.\n\n");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    GetUserInput(buf);
    if (buf[0] == 't') 
    {
      printfLog("Performing tare (zeroing) procedure..\n");
      LoadCell.tare();
      nvram_settings.load_offset=LoadCell.getTareOffset();
      printfLog("Load offset set to: %d\n\n", nvram_settings.load_offset);
      _resume = true;
    }
  }

  printfLog("Provide your exact weight in kg:\n\n");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    GetUserInput(buf);
    if (buf[0] != '\0') {
      known_mass = atof(buf);
      if (known_mass != 0) {
        printfLog("Provided calibration weight is %.1fkg (%.1f Newton per pedal)\n\n",known_mass, gn * known_mass / 2);
        _resume = true;
      }
    }
  }

  printfLog("\nNow stand-up on your pedals,\n");
  printfLog("\nwith the pedals horizontally,\n");
  printfLog("\nthe load-sensor pedal to the front,\n");
  printfLog("evenly balancing your weight,\n");
  printfLog("and press 's' to start calibration\n\n");

  _resume = false;
  while (_resume == false) {
    GetUserInput(buf);
    if (buf[0] == 's') _resume = true;
  }

  printfLog("Calibrating..\n");
  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  // Get and set the new calibration value for 0.5 * user-weight in Newtons
  nvram_settings.load_multiplier = LoadCell.getNewCalibration(gn * known_mass * 0.5); 

  printfLog("Load multiplier set to: %.1f\n\n",nvram_settings.load_multiplier);
  printfLog("\nSave calibration values? (y/n)\n\n");

  _resume = false;
  while (_resume == false) {
    GetUserInput(buf);
    if (buf[0] == 'y') {
      if( file.open(CALIBRATIONS_FILENAME, FILE_O_WRITE) )
      {
        uint32_t readlen;
        file.seek(0); // Overcome weird behaviour of LittleFS to always APPEND instead of OVERWRITE
        file.write((char*)&nvram_settings, sizeof(nvram_settings));
        file.close();
        Serial.printf("Calibrations saved.\n");
      } else
      {
        printfLog("Saving the Calibrations failed!\n");
      }
      _resume = true;
    }
    else if (buf[0] == 'n') {
      printfLog("Value not saved to EEPROM/UICR\n");
      _resume = true;
    }
  }

  printfLog("End of calibration\n");
  printfLog("***\n");
  printfLog("To re-calibrate, send 'c' from bluetooth or serial monitor.\n");
  printfLog("***\n\n");

  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}
