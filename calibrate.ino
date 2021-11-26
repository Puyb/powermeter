/**
 * Load sensor calibration routines
 */


void calibrateLoadCell() {
  char buf[64]={'\0'};

  printfLog("***\n");
  printfLog("Start calibration:\n");
  printfLog("Place the load cell a level stable surface.\n");
  printfLog("Remove any load applied to the load cell.\n");
  printfLog("Send 't' to set the tare offset.\n\n");

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

  printfLog("Now, place your known mass on the loadcell.\n");
  printfLog("Then send the weight of this mass in kg (i.e. 100).\n\n");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    GetUserInput(buf);
    if (buf[0] != '\0') {
      known_mass = atof(buf);
      if (known_mass != 0) {
        printfLog("Known mass is: %f\n",known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  nvram_settings.load_multiplier = LoadCell.getNewCalibration(known_mass); //get and set the new calibration value

  printfLog("Load multiplier set to: %f\n\n",nvram_settings.load_multiplier);
  printfLog("Save calibration values? (y/n)\n");

  _resume = false;
  while (_resume == false) {
    GetUserInput(buf);
    if (buf[0] == 'y') {
      if( file.open(CALIBRATIONS_FILENAME, FILE_O_WRITE) )
      {
        uint32_t readlen;
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
}

void getStoredCalibrationFactor(void)
{
  InternalFS.begin();
  file.open(CALIBRATIONS_FILENAME, FILE_O_READ);
  if ( file )
  {
    uint32_t readlen;
    readlen = file.read((char*)&nvram_settings, sizeof(nvram_settings));
    file.close();

   	LoadCell.setCalFactor(nvram_settings.load_multiplier);
    LoadCell.setTareOffset(nvram_settings.load_offset);          

    Serial.printf("Calibrations found.\n");
    Serial.printf("Load offset set to: %d\n",nvram_settings.load_offset);
    Serial.printf("Load multiplier set to: %f\n",nvram_settings.load_multiplier);	
  } else
  {
    Serial.printf("No calibrations found!\n");
  }
}
