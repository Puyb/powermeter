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
      printfLog("Tare procedure completed\n\n");
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
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  printfLog("New calibration value has been set to: %f\n",newCalibrationValue);
  printfLog("Use this as calibration value (calFactor) in your project sketch.\n");
  printfLog("Save this value to EEPROM? (y/n)\n");

  _resume = false;
  while (_resume == false) {
    GetUserInput(buf);
    if (buf[0] == 'y') {
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
      EEPROM.begin(512);
      EEPROM.put(calVal_eepromAdress, newCalibrationValue);
      EEPROM.commit();
      EEPROM.get(calVal_eepromAdress, newCalibrationValue);
      printfLog("Value %f", newCalibrationValue);
      printfLog(" saved to EEPROM");
#else
      printfLog("EEPROM not supported on this platform\n");	
#endif
      _resume = true;
    }
    else if (buf[0] == 'n') {
      printfLog("Value not saved to EEPROM\n");
      _resume = true;
    }
  }

  printfLog("End of calibration\n");
  printfLog("***\n");
  printfLog("To re-calibrate, send 'c' from serial monitor.\n");
  printfLog("***\n\n");
}
