
// /*********************************************************************
// CODE in Central for sending data 
// *********************************************************************/

// /*
//  * This sketch demonstrate the central API(). A additional bluefruit
//  * that has bleuart as peripheral is required for the demo.
//  */
#include <bluefruit.h>

BLEClientUart clientUart; // bleuart client

//////////
BLEClientBas  clientBas;  // battery client
BLEClientDis  clientDis;  // device information client
//////////

/////////
const char content[10] ={0xAA,0x17,0xE8,0x00,0xFE,0x00,0x00,0x5B,0x0D,0x0A};
const char content1[10] ={0xAA,0x17,0xE8,0x00,0xFE,0x00,0x00,0x5B};
const char content_int[8] ={170,23,232,0,254,0,0,91};
/////////


uint32_t rx_count = 0;

void scan_callback(ble_gap_evt_adv_report_t* report);
void bleuart_rx_callback(BLEClientUart& uart_svc);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void printHexList(uint8_t* buffer, uint8_t len);


void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central BLEUART Example");
  Serial.println("-----------------------------------\n");

  // Config the connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);  
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  ///////////
  // Configure Battyer client
  clientBas.begin();  

  // Configure DIS client
  clientDis.begin();
  //////////


  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);  

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  String checkbuffer;
  /* Complete Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.println("-------------------------2");
    Serial.printf("%s",buffer);
    Serial.println("--------------------------");
    checkbuffer = String((char *)buffer);
    // if(checkbuffer == "SleepU 3635")
    if(checkbuffer == "TestBle")
    // if(checkbuffer == "ElcomEcg")
    {
      Serial.printf("%14s %s\n", "COMPLETE NAME", buffer);
      Bluefruit.Central.connect(report);
      Serial.println("*************************************");
    }
    else{
      Bluefruit.Scanner.resume();
    }
    
    memset(buffer, 0, sizeof(buffer));

  }
  else{
    Bluefruit.Scanner.resume();
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");

  /////////
  // Serial.print("Dicovering Device Information ... ");
  if ( clientDis.discover(conn_handle) )
  {
    Serial.println("Found it");
    char buffer[32+1];
    
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }
  }

  Serial.println();
  /////////


  Serial.print("Discovering BLE Uart Service ... ");
  if ( clientUart.discover(conn_handle) )
  {
    uint8_t buffer[32+1];
    memset(buffer, 0, sizeof(buffer));

    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();
    Serial.println("Ready to receive from peripheral");

  }else
  {
    Serial.println("Found NONE");
    
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
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
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */ 
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  Serial.println("bleuart_rx_callback");
  int count = uart_svc.available();
  (void)count;
  // Serial.println("bleuart_rx_callback");
  while ( uart_svc.available() )
  {
    Serial.print( (char) uart_svc.read() );
  }
  Serial.println();
  uart_svc.flush();
}

void printHexList(uint8_t* buffer, uint8_t len)
{  
  // print forward order
  for(int i=0; i<len; i++)
  { 
    Serial.printf("%02X-", buffer[i]);
  } 
  Serial.println();  
}

uint32_t long_time;
void loop()
{

  if(millis() - long_time > 2000){
    long_time = millis();
    // Serial.print("Send data: ");
    // Serial.println (sizeof(content_int));
    
    // clientUart.write(content_int, sizeof(content_int) - 1);
    // clientUart.write(content_int, 8);
    // clientUart.write(content_int, 8);
    // clientUart.print(content1);
    // clientUart.print(0xAA);
    // clientUart.print(0x17);
    // clientUart.print(0xE8);
    // clientUart.print(0x00);
    // clientUart.print(0xFE);
    // clientUart.print(0x00);
    // clientUart.print(0x00);
    // clientUart.println(0x5B);
  }

  if ( Bluefruit.Central.connected() )
  {
  // Not discovered yet
    if ( clientUart.discovered() )
    {
      // Discovered means in working state
      // Get Serial input and send to Peripheral
      if ( Serial.available() )
      {
        delay(2); // delay a bit for all characters to arrive
        Serial.println("Send data.");
        //////////
        // uint8_t content_send;
        // String b = AA 17 E8 00 FE 00 00 5B);
        // for(int i = 0; i < sizeof(content); i++)
        // {
          // content_send = content_send + content[i];
        //   // clientUart.write(content[i]);
        //   // Serial.print(content[i],HEX);
        //   // Serial.println();
        // }
        // clientUart.write(content_send);
        // Serial.print(content_send);

        // clientUart.write(content, sizeof(content));
        // Serial.println(String((char *)content));
        
        
        // clientUart.read8();

        // clientUart.read(buffer, sizeof(buffer));
        // Serial.print(String((char *)buffer));

        // Serial.print(clientUart.read(), HEX);

        // for(int i = 0; i < 20; i++)
        // {
        //   Serial.print(clientUart.read(buffer, sizeof(buffer)));
        //   Serial.println();
        // }
        
        // char str[20+1] = { 0 };
        // Serial.readBytes(str, 20);
        
        // clientUart.print( str );
      }
    }
  }
}

//--------------------------------------------------------------------------------------//
/*********************************************************************
Code in Peripherial to recieve and after that send again the piece of information to Central
*********************************************************************/
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

BLEClientUart clientUart;

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
// void prph_bleuart_rx_callback(uint16_t conn_handle);
void startAdv(void);

void setup()
{
  Serial.begin(115200);

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial ) yield();
#endif
  
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  // Bluefruit.begin(1,1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("TestBle"); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
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

void prph_bleuart_rx_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  
  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(str);  

  // if ( clientUart.discovered() )
  // {
  //   clientUart.print(str);
  // }else
  // {
  //   bleuart.println("[Prph] Central role not connected");
  // }
}

uint32_t long_time;
void loop()
{
  // after 2 second, the Pherial print value to Central
  if(millis() - long_time > 2000){
    long_time = millis();
    Serial.println("Send data: ");
    bleuart.print("abc");

  }
  // // Forward data from HW Serial to BLEUART
  // while (Serial.available())
  // {
  //   // Delay to wait for enough input, since we have a limited transmission buffer
  //   delay(2);

  //   uint8_t buf[64];
  //   int count = Serial.readBytes(buf, sizeof(buf));
  //   bleuart.write( buf, count );
  // }

  // // Forward from BLEUART to HW Serial
  // while ( bleuart.available() )
  // {
  //   uint8_t ch;
  //   ch = (uint8_t) bleuart.read();
  //   Serial.print(ch, HEX);
  //   bleuart.print(ch);
  // }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
