/**
 * This file keeps the BLE helpers, to send the data over bluetooth
 * to the bike computer. Or any other receiver, if dev/debug.
 *
 * For the Adafruit BLE lib, see:
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/bd0747473242d5d7c58ebc67ab0aa5098db56547/libraries/Bluefruit52Lib
 */

#include <stdarg.h>
#include <BLECharacteristic.h>

// For battery-level calculations
#define VBAT_MV_PER_LSB (0.7324F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER (0.7128F) // 0.806M and 2M voltage divider factor

// Service and character constants at:
// https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/bd0747473242d5d7c58ebc67ab0aa5098db56547/libraries/Bluefruit52Lib/src/BLEUuid.h
/* Pwr Service Definitions
 * Cycling Power Service:      0x1818
 * Power Measurement Char:     0x2A63
 * Cycling Power Feature Char: 0x2A65
 * Sensor Location Char:       0x2A5D
 */
BLEService        pwrService  = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic pwrMeasChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic pwrFeatChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic pwrLocChar  = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);
BLECharacteristic pwrVector   = BLECharacteristic(UUID16_CHR_CYCLING_POWER_VECTOR);


BLEDfu bledfu;    // OTA DFU service
BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance
BLEUart bleuart;  // UART over BLE 

#define MAX_PRPH_CONNECTION   2

void bleSetup() {

  // off Blue LED for lowest power consumption
  Bluefruit.autoConnLed(false);

  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  Bluefruit.setName(DEV_NAME);

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();
  
  // Configure and Start the Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start the BLE Battery Service
  blebas.begin();
  pinMode(BOARD_VBATPIN, INPUT);

  // Configure and Start BLE DFU OTA service
  bledfu.begin();

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  setupPwr();

  // Setup the advertising packet(s)
  startAdv();
  Serial.println("Bluetooth initialized. Please use Adafruit's Bluefruit LE app to connect in UART mode");
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(-8);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(pwrService);
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/*
 * Set up the power service
 */
void setupPwr(void) {
  // Configure supported characteristics:
  pwrService.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Has to have notify enabled.
  // Power measurement. This is the characteristic that really matters. See:
  // https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.cycling_power_measurement.xml
  pwrMeasChar.setProperties(CHR_PROPS_NOTIFY);
  // First param is the read permission, second is write.
  pwrMeasChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // 8 total bytes, 4 16-bit values
  pwrMeasChar.setFixedLen(PWR_MEAS_CHAR_LEN);
  // Optionally capture Client Characteristic Config Descriptor updates
  pwrMeasChar.setCccdWriteCallback(cccdCallback);
  pwrMeasChar.begin();

  /*
   * The other two characterstics aren't updated over time, they're static info
   * relaying what's available in our service and characteristics.
   */

  // Characteristic for power feature. Has to be readable, but not necessarily
  // notify. 32 bit value of what's supported, see
  // org.bluetooth.characteristic.cycling_power_feature.xml
  // https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_feature.xml
  pwrFeatChar.setProperties(CHR_PROPS_READ);
  pwrFeatChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // 1 32-bit value
  pwrFeatChar.setFixedLen(4);
  pwrFeatChar.begin();
  // No extras for now, write 0.
  pwrFeatChar.write32(0);

  // Characteristic for sensor location. Has to be readable, but not necessarily
  // notify. See:
  // https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.sensor_location.xml
  pwrLocChar.setProperties(CHR_PROPS_READ);
  pwrLocChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrLocChar.setFixedLen(1);
  pwrLocChar.begin();
  // Set location to "left crank"
  pwrLocChar.write8(5);


  //
  // https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_control_point.xml
  // org.bluetooth.characteristic.cycling_power_control_point.xml
  //


  //
  //
  // https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_vector.xml
  // org.bluetooth.characteristic.cycling_power_vector.xml

}

void bleuart_data_transfer() {
  // Forward data from HW Serial to BLEUART
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write( buf, count );
  }

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }
}

// Get user input from bluetooth or serial
void GetUserInput(char* buf) {
  buf[0] = 0;

  // Get serial input
  if (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(20);

    Serial.readBytes(buf, sizeof(buf));
  }

  // Get bluetooth input
  int pos=0;
  while (bleuart.available()) {
    buf[pos++] = (char)bleuart.read();
  }

  if (pos > 1) buf[pos-1] = '\0';

//  if (buf[0] != '\0') {
//    Serial.printf("Buffer: (%s)(0x%x)(0x%x)\n",buf,buf[0],buf[1]); Serial.println("");
//  }
}

/*
 * Publish the instantaneous power measurement.
 */
void blePublishPower(int16_t instantPwr, uint16_t crankRevs, long millisLast) {
  // Power measure characteristic
  /**
   * Fields
   *
   * Flags (16 bits):
   *   b0 pedal power balance present
   *   b1 pedal power balance reference
   *   b2 accumulated torque present
   *   b3 accumulated torque source
   *   b4 wheel revolution data present
   *   b5 crank revolution data present
   *   b6 extreme force magnitudes present
   *   b7 extreme torque magnitudes present
   *   b8 extreme angles present
   *   b9 top dead spot angle present
   *   b10 bottom dead spot angle present
   *   b11 accumulated energy present
   *   b12 offset compenstation indicator
   *   b13 reserved
   *
   *   https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_measurement.xml
   * 
   * Instananous Power:
   *   16 bits signed int
   *   
   * Cumulative Crank Revolutions:
   *   16 bits signed int
   *
   * Last Crank Event Time
   *   16 bits signed int
   */
  // Flag cadence.
  uint16_t flag = 0b0000000000100000;

  // All data in characteristics goes least-significant octet first.
  // Split them up into 8-bit ints. LSO ends up first in array.
  uint8_t flags[2];
  uint16ToLso(flag, flags);
  uint8_t pwr[2];
  uint16ToLso(instantPwr, pwr);

  // Cadence last event time is time of last event, in 1/1024 second resolution
  uint16_t lastEventTime = uint16_t(millisLast / 1000.f * 1024.f) % 65536;
//  Serial.printf("Timestamp: %d\n",lastEventTime);

  // Split the 16-bit ints into 8 bits, LSO is first in array.
  uint8_t cranks[2];
  uint16ToLso(crankRevs, cranks);
  uint8_t lastTime[2];
  uint16ToLso(lastEventTime, lastTime);

  // All fields are 16-bit values, split into two 8-bit values.
  uint8_t pwrdata[PWR_MEAS_CHAR_LEN] = { flags[0], flags[1],  
                                         pwr[0], pwr[1], 
                                         cranks[0], cranks[1],
                                         lastTime[0], lastTime[1] };
  //uint8_t pwrdata[4] = { flags[0], flags[1], pwr[0], pwr[1] };

  if (connection_count > 0) {
    if (!pwrMeasChar.notify(pwrdata, sizeof(pwrdata))) {
  //    printfLog("ERROR: Power notify not set in the CCCD or not connected!\n");
    }
  }
}

void blePublishBatt() {
  float vbat_mv = readVBAT();


  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_mv);

  if (connection_count > 0) blebas.write(vbat_per);

  printfLog("      Battery level: %d%%  (%.3fV)\n\n", vbat_per, vbat_mv/1000.0);
}

void connectCallback(uint16_t connHandle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(connHandle);  
  char centralName[32] = { 0 };
  
  connection->getPeerName(centralName, sizeof(centralName));

  connection_count++;

  Serial.printf("Connected to %s", centralName);
  Serial.printf("Connection count: %d", connection_count);

  blePublishBatt(); // Publish battery level to newly connected device

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
 */
void disconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void) connHandle;
  (void) reason;

  connection_count--;
  
  Serial.println("Disconnected");
}

/*
 * Given a 16-bit uint16_t, convert it to 2 8-bit ints, and set
 * them in the provided array. Assume the array is of correct
 * size, allocated by caller. Least-significant octet is place
 * in output array first.
 */
void uint16ToLso(uint16_t val, uint8_t* out) {
  uint8_t lso = val & 0xff;
  uint8_t mso = (val >> 8) & 0xff;
  out[0] = lso;
  out[1] = mso;
}

/*
 * Publish logging to bluetooth
 */
void blePublishLog(char *msg, int numBytes) {
  static const short MAX = 20;  // 19 chars plus the null terminator
  int bytesToDo = numBytes;

  while (bytesToDo > 0) {

    if (bytesToDo > MAX) {
      bleuart.write(msg, MAX);
      bytesToDo -= MAX;
      msg = strcpy(msg, &msg[MAX]);
    } else {
      bleuart.write(msg, bytesToDo);
      bytesToDo = 0;
    }
    delay(40);
  }
}

void printfLog(const char* fmt, ...) {
  
  static const short MAX = 20;  // 19 chars plus the null terminator
  static char msg[256];

  va_list args;
  va_start(args, fmt);
  int numBytes = vsnprintf(msg, 255, fmt, args);
  va_end(args);

  if ((numBytes > 0) && (numBytes < 255)) {
    Serial.print(msg); 
    if (connection_count > 0) blePublishLog(msg, numBytes);
  } 
}

void cccdCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccdValue) {
  // Display the raw request packet
    Serial.printf("CCCD Updated: %d\n", cccdValue);

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr->uuid == pwrMeasChar.uuid) {
    if (chr->notifyEnabled()) {
      Serial.println("Pwr Measurement 'Notify' enabled");
    } else {
      Serial.println("Pwr Measurement 'Notify' disabled");
    }
  }
}
