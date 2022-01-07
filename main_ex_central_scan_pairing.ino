/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>

#if defined(ARDUINO_NRF52840_CIRCUITPLAY) || defined(ARDUINO_NRF52840_CLUE)
  #define USE_ARCADA
#endif

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#ifdef USE_ARCADA
  #include <Adafruit_Arcada.h>

  Adafruit_Arcada arcada;
  Adafruit_SPITFT* tft;

#else
  // Use built-in buttons if available, else use A0, A1
  #ifdef PIN_BUTTON1
    #define BUTTON_YES    PIN_BUTTON1
  #else
    #define BUTTON_YES    A0
  #endif

  #ifdef PIN_BUTTON2
    #define BUTTON_NO   PIN_BUTTON2
  #else
    #define BUTTON_NO   A1
  #endif
#endif

BLEClientUart clientUart; // bleuart client

/* For a list of EIR data types see:
 *    https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
 *    Matching enum: cores/nRF5/SDK/components/softdevice/s132/headers/ble_gap.h */

// Convert to string 
String converter(uint8_t *str){
    return String((char *)str);
}

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central ADV Scan Example");
  Serial.println("------------------------------------\n");

#ifdef USE_ARCADA
  arcada.arcadaBegin();
  arcada.displayBegin();
  arcada.setBacklight(255);

  tft = arcada.display;
  tft->setCursor(0, 0);
  tft->setTextWrap(true);
  tft->setTextSize(2);
#else
  pinMode(BUTTON_YES, INPUT_PULLUP);
  pinMode(BUTTON_NO, INPUT_PULLUP);
#endif

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  /* Set the device name */
  Bluefruit.setName("Bluefruit52");

  /////////
  Bluefruit.Security.setIOCaps(true, true, false); // display = true, yes/no = true, keyboard = false
  Bluefruit.Security.setPairPasskeyCallback(pairing_passkey_callback);

  // Set complete callback to print the pairing result
  Bluefruit.Security.setPairCompleteCallback(pairing_complete_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);

  /// Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  /// Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->println("Scanning ...");
#endif

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Filter out packet with a min rssi
   * - Interval = 100 ms, window = 50 ms
   * - Use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);
  //Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE); // only invoke callback if detect bleuart service
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  Serial.println("Scanning ...");

  // char response[] = "SleepU 3635";
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  PRINT_LOCATION();
  uint8_t len = 0;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  
  /* Display the timestamp and device address */
  if (report->type.scan_response)
  {
    Serial.printf("[SR%10d] Packet received from ", millis());
  }
  else
  {
    Serial.printf("[ADV%9d] Packet received from ", millis());
  }
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.print("\n");

  /* Raw buffer contents */
  Serial.printf("%14s %d bytes\n", "PAYLOAD", report->data.len);
  if (report->data.len)
  {
    Serial.printf("%15s", " ");
    Serial.printBuffer(report->data.p_data, report->data.len, '-');
    Serial.println();
  }

  /* RSSI value */
  Serial.printf("%14s %d dBm\n", "RSSI", report->rssi);

  /* Adv Type */
  Serial.printf("%14s ", "ADV TYPE");
  if ( report->type.connectable ) 
  {
    Serial.print("Connectable ");
  }else
  {
    Serial.print("Non-connectable ");
  }
  
  if ( report->type.directed )
  {
    Serial.println("directed");
  }else
  {
    Serial.println("undirected");
  }

  /* Shortened Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %s\n", "SHORT NAME", buffer);
    memset(buffer, 0, sizeof(buffer));
    Serial.println("-------------------------1");
    Serial.println(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)));
    Serial.println("--------------------------");
  }

  /* Complete Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.println("-------------------------2");
    Serial.printf("%s",buffer);
    Serial.println("--------------------------");
    String checkbuffer = converter(buffer);
    if(checkbuffer == "SleepU 3635")
    {
      Bluefruit.Central.connect(report);
      Serial.println("*************************************");
    }
    else{
      Bluefruit.Scanner.resume();
    }
    Serial.printf("%14s %s\n", "COMPLETE NAME", buffer);
    memset(buffer, 0, sizeof(buffer));

//    Serial.println("-------------------------3");
//    Serial.println(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)));
//    //Serial.printBuffer(report);
//    Serial.print(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
//    //Serial.printBuffer(buffer);
//    Serial.println("--------------------------");

  
  }

  /* TX Power Level */
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %i\n", "TX PWR LEVEL", buffer[0]);
    memset(buffer, 0, sizeof(buffer));
  }

  /* Check for UUID16 Complete List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid16List(buffer, len);
  }

  /* Check for UUID16 More Available List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid16List(buffer, len);
  }

  /* Check for UUID128 Complete List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid128List(buffer, len);
  }

  /* Check for UUID128 More Available List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid128List(buffer, len);
  }  

  /* Check for BLE UART UUID */
  if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
  {
    Serial.printf("%14s %s\n", "BLE UART", "UUID Found!");
  }

  /* Check for DIS UUID */
  if ( Bluefruit.Scanner.checkReportForUuid(report, UUID16_SVC_DEVICE_INFORMATION) )
  {
    Serial.printf("%14s %s\n", "DIS", "UUID Found!");
  }

  /* Check for Manufacturer Specific Data */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (len)
  {
    Serial.printf("%14s ", "MAN SPEC DATA");
    Serial.printBuffer(buffer, len, '-');
    Serial.println();
    memset(buffer, 0, sizeof(buffer));
  }  

  Serial.println();

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

void printUuid16List(uint8_t* buffer, uint8_t len)
{
  Serial.printf("%14s %s", "16-Bit UUID");
  for(int i=0; i<len; i+=2)
  {
    uint16_t uuid16;
    memcpy(&uuid16, buffer+i, 2);
    Serial.printf("%04X ", uuid16);
  }
  Serial.println();
}

void printUuid128List(uint8_t* buffer, uint8_t len)
{
  (void) len;
  Serial.printf("%14s %s", "128-Bit UUID");

  // Print reversed order
  for(int i=0; i<16; i++)
  {
    const char* fm = (i==4 || i==6 || i==8 || i==10) ? "-%02X" : "%02X";
    Serial.printf(fm, buffer[15-i]);
  }

  Serial.println();  
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  Serial.println("Connected");

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->println("Connected");
#endif

  // If we are not bonded with peer previously -> send pairing request
  // Else wait for the connection secured callback
  if ( !conn->bonded() )
  {
    conn->requestPairing();
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

#ifdef USE_ARCADA
  tft->println("Scanning ...");
#endif
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  while ( uart_svc.available() )
  {
    Serial.print( (char) uart_svc.read() );
  }
}

bool pairing_passkey_callback(uint16_t conn_handle, uint8_t const passkey[6], bool match_request)
{
  Serial.println("Pairing Passkey");
  Serial.printf("    %.3s %.3s\n\n", passkey, passkey+3);

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->println("Pairing Passkey\n");
  tft->setTextColor(ARCADA_YELLOW);
  tft->setTextSize(4);
  tft->printf("  %.3s %.3s\n", passkey, passkey+3);

  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
#endif

  // match_request means peer wait for our approval (return true)
  if (match_request)
  {
    Serial.println("Do you want to pair");
    Serial.println("Press Button Left to decline, Button Right to Accept");

    // timeout for pressing button
    uint32_t start_time = millis();

#ifdef USE_ARCADA
    tft->println("\nDo you accept ?\n\n");
    tft->setTextSize(3);

    // Yes <-> No on CPB is reversed since GIZMO TFT is on the back of CPB
    #if ARDUINO_NRF52840_CIRCUITPLAY
      tft->setTextColor(ARCADA_GREEN);
      tft->print("< Yes");
      tft->setTextColor(ARCADA_RED);
      tft->println("    No >");
    #else
      tft->setTextColor(ARCADA_RED);
      tft->print("< No");
      tft->setTextColor(ARCADA_GREEN);
      tft->println("    Yes >");
    #endif

    tft->setTextColor(ARCADA_WHITE);
    tft->setTextSize(2);
    tft->println();

    // wait until either button is pressed (30 seconds timeout)
    uint32_t justReleased = 0;
    do
    {
      if ( millis() > start_time + 30000 ) break;

      arcada.readButtons();
      justReleased = arcada.justReleasedButtons();
    } while ( !(justReleased & (ARCADA_BUTTONMASK_LEFT | ARCADA_BUTTONMASK_RIGHT) ) );

    // Right = accept
    if (justReleased & ARCADA_BUTTONMASK_RIGHT) return true;

    // Left = decline
    if (justReleased & ARCADA_BUTTONMASK_LEFT) return false;

#else
    // wait until either button is pressed (30 seconds timeout)
    while( digitalRead(BUTTON_YES) && digitalRead(BUTTON_NO) )
    {
      if ( millis() > start_time + 30000 ) break;
    }

    if ( 0 == digitalRead(BUTTON_YES) ) return true;

    if ( 0 == digitalRead(BUTTON_NO) ) return false;
#endif

    return false;
  }

  return true;
}

void pairing_complete_callback(uint16_t conn_handle, uint8_t auth_status)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    Serial.println("Succeeded");
  }else
  {
    Serial.println("Failed");

    // disconnect
    conn->disconnect();
  }

#ifdef USE_ARCADA
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    tft->setTextColor(ARCADA_GREEN);
    tft->print("Succeeded ");
  }else
  {
    tft->setTextColor(ARCADA_RED);
    tft->print("Failed ");
  }

  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
#endif
}

void connection_secured_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if ( !conn->secured() )
  {
    // It is possible that connection is still not secured by this time.
    // This happens (central only) when we try to encrypt connection using stored bond keys
    // but peer reject it (probably it remove its stored key).
    // Therefore we will request an pairing again --> callback again when encrypted
    conn->requestPairing();
  }
  else
  {
    Serial.println("Secured");

    #ifdef USE_ARCADA
    tft->setTextColor(ARCADA_YELLOW);
    tft->println("secured");
    tft->setTextColor(ARCADA_WHITE);
    #endif

    Serial.print("Discovering BLE Uart Service ... ");
    if ( clientUart.discover(conn_handle) )
    {
      Serial.println("Found it");

      Serial.println("Enable TXD's notify");
      clientUart.enableTXD();

      Serial.println("Ready to receive from peripheral");
    }else
    {
      Serial.println("Found NONE");

      // disconnect since we couldn't find bleuart service
      conn->disconnect();
    }
  }
}

void loop() 
{
  // nothing to do
}

