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

  printfLog("New calibration value has been set to: %f\n",nvram_settings.load_multiplier);
  printfLog("Use this as calibration value (calFactor) in your project sketch or save it to EEPROM/UICR.\n");
  printfLog("Save this value to EEPROM/UICR? (y/n)\n");

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

void app_mwu_enable(void)
{
  NRF_MWU->REGIONENSET
    = ((MWU_REGIONENSET_RGN0WA_Set << MWU_REGIONENSET_RGN0WA_Pos) | (MWU_REGIONENSET_PRGN0WA_Set << MWU_REGIONENSET_PRGN0WA_Pos));
}

void app_mwu_disable(void)
{
  NRF_MWU->REGIONENCLR
    = ((MWU_REGIONENCLR_RGN0WA_Clear << MWU_REGIONENCLR_RGN0WA_Pos) | (MWU_REGIONENCLR_PRGN0WA_Clear << MWU_REGIONENCLR_PRGN0WA_Pos));
}

/*
// write a float to the UICR
// x = 0..31
*/
void write_UICR_CUSTOMER_float(int x, float value)
{
  unsigned long long_value = *((unsigned long*)&value);
  write_UICR_CUSTOMER(x, long_value);
}

/*
// write a 32-bit unsigned long to the UICR
// x = 0..31
*/
void write_UICR_CUSTOMER(int x, unsigned long value)
{
#if defined(ARDUINO_NRF52_ADAFRUIT)
  app_mwu_disable();

  if ((uint32_t)(NVMC_START_ADDRESS + (x * 4)) != 0xFFFFFFFF) 
  {
/*
    // Set config to Erase
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    NRF_NVMC->ERASEUICR = 1;

    // Clear Erase mode
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    NRF_NVMC->ERASEUICR = 0; // Redundant?

    // Set Read mode
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
*/
    uint32_t ret=sd_flash_page_erase((uint32_t)(NVMC_START_ADDRESS / (NVMC_PAGE_SIZE)));    
    delay(1500);
    Serial.printf("Erased page %d: %d\n",(uint32_t)(NVMC_START_ADDRESS / (NVMC_PAGE_SIZE)),ret);
  }

//  sd_flash_write((uint32_t*)(NVMC_START_ADDRESS + (x * 4)), &value, 1);

/*
  // Set config to Write
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
  NRF_UICR->CUSTOMER[x] = value;

  // Set config to Read
  NRF_NVMC->ERASEUICR = 0;
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
*/
  app_mwu_enable();

#else
  printfLog("NVMC not supported on this platform\n");
#endif
}


/*
{
        // Storage buffers and variables to hold the UICR register content
        static uint32_t uicr_buffer[59]    = {0x00000000};
        static uint32_t pselreset_0        = 0x00000000;
        static uint32_t pselreset_1        = 0x00000000;
        static uint32_t approtect          = 0x00000000;
        static uint32_t nfcpins            = 0x00000000;

        CRITICAL_REGION_ENTER();
      
        // Read and buffer UICR register content prior to erase
        uint32_t uicr_address = 0x10001014;
 
        for(int i = 0; i<sizeof(uicr_buffer); i++)
        {
            uicr_buffer[i] = *(uint32_t *)uicr_address; 
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            // Set UICR address to the next register
            uicr_address += 0x04;
        }
      
        pselreset_0 = NRF_UICR->PSELRESET[0];
        pselreset_1 = NRF_UICR->PSELRESET[1];
        approtect   = NRF_UICR->APPROTECT;
        nfcpins     = NRF_UICR->NFCPINS;
        
        //Modify the Bootloader start address  to correspond to the new bootloader
        uicr_buffer[0] = 0x00078000;
       
        // Enable Erase mode
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos; //0x02; 
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        
        // Erase the UICR registers
        NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase << NVMC_ERASEUICR_ERASEUICR_Pos; //0x00000001;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
       
        // Enable WRITE mode
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos; //0x01;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
   
        // Write the modified UICR content back to the UICR registers 
        uicr_address = 0x10001014;
        for(int j = 0; j<sizeof(uicr_buffer); j++)
        {
            // Skip writing to registers that were 0xFFFFFFFF before the UICR register were erased. 
            if(uicr_buffer[j] != 0xFFFFFFFF)
            {
                *(uint32_t *)uicr_address = uicr_buffer[j];
                // Wait untill the NVMC peripheral has finished writting to the UICR register
                while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}                
            }
            // Set UICR address to the next register
            uicr_address += 0x04;  
        }

        NRF_UICR->PSELRESET[0]  = pselreset_0;
        NRF_UICR->PSELRESET[1]  = pselreset_1;
        NRF_UICR->APPROTECT     = approtect;
        NRF_UICR->NFCPINS       = nfcpins;

        CRITICAL_REGION_EXIT();
}
*/

/*
// read a float from the UICR
// x = 0..31
*/
bool read_UICR_CUSTOMER_float(int x, float *value)
{
  unsigned long long_value;

  if(read_UICR_CUSTOMER(x, &long_value))
  {
    *value = *((float*)&long_value);
    return true;
  }
  else
  {
    return false;
  }
}


/*
// read a 32-bit unsigned long from the UICR
// x = 0..31
*/
bool read_UICR_CUSTOMER(int x, unsigned long *value)
{
#if defined(ARDUINO_NRF52_ADAFRUIT)
  unsigned long long_value=(uint32_t)(NVMC_START_ADDRESS + (x * 4));

  if(long_value != 0xFFFFFFFF)
  {
    *value = long_value;
    return(true);
  }
  else
  {
    *value = 0;
    return(false);
  }
#else
  printfLog("NVMC not supported on this platform\n");
  return 0;
#endif
}
