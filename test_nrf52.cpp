
#include <bluefruit.h>

BLEClientUart clientUart; // bleuart client

//////////
// BLEClientBas  clientBas;  // battery client
// BLEClientDis  clientDis;  // device information client
//////////

/////////
// const uint8_t content_uint[10] ={0xAA,0x17,0xE8,0x00,0xFE,0x00,0x00,0x5B,0x0D,0x0A};
// const char content_hex[10] ={0xAA,0x17,0xE8,0x00,0xFE,0x00,0x00,0x5B,0x0D,0x0A};
uint8_t content_hex[8] ={0xAA,0x17,0xE8,0x00,0xFE,0x00,0x00,0x5B};
uint8_t content_int[8] ={170,23,232,0,254,0,0,91};
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
  
  Bluefruit.setName("Bluefruit52");

  ///////////
  // Configure Battyer client
  // clientBas.begin();  

  // Configure DIS client
  // clientDis.begin();
  //////////


  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);  

  // Increase Blink rate to different from PrPh advertising mode
  // Bluefruit.setConnLedInterval(250);

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
  // if ( clientDis.discover(conn_handle) )
  // {
  //   Serial.println("Found it");
  //   char buffer[32+1];
    
  //   // read and print out Manufacturer
  //   memset(buffer, 0, sizeof(buffer));
  //   if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
  //   {
  //     Serial.print("Manufacturer: ");
  //     Serial.println(buffer);
  //   }

  //   // read and print out Model Number
  //   memset(buffer, 0, sizeof(buffer));
  //   if ( clientDis.getModel(buffer, sizeof(buffer)) )
  //   {
  //     Serial.print("Model: ");
  //     Serial.println(buffer);
  //   }
  // }

  // Serial.println();
  /////////


  Serial.print("Discovering BLE Uart Service ... ");
  if ( clientUart.discover(conn_handle) )
  {
    uint8_t buffer[32+1];
    memset(buffer, 0, sizeof(buffer));

    Serial.println("Found it");

    
    if(clientUart.enableTXD()){
      Serial.println("Enable TXD's notify");
    }
    else{
      Serial.println("Can't Enable TXD's notify");
    }
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
  // Serial.println("bleuart_rx_callback");
  int count = uart_svc.available();
  (void)count;
  //  Serial.println("bleuart_rx_callback");
  while ( uart_svc.available() )
  {
    Serial.print( (char) uart_svc.read(),HEX );
    Serial.print( "-" );
    //  Serial.println("bleuart_rx_callback_l");
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

  if( Bluefruit.Central.connected() && millis() - long_time > 2000){
    long_time = millis();
    //////////////
    Serial.print("Status: ");
    Serial.println(clientUart.write(content_int,8));
    // clientUart.write((uint8_t)170);
    // clientUart.write((uint8_t)23);
    // clientUart.write((uint8_t)232);
    // clientUart.write((uint8_t)0);
    // clientUart.write((uint8_t)254);
    // clientUart.write((uint8_t)0);
    // clientUart.write((uint8_t)0);
    // clientUart.write((uint8_t)91);
    
    // clientUart.print((char)170, HEX);
    // clientUart.print((char)23, HEX);
    // clientUart.print((char)232, HEX);
    // clientUart.print((char)0, HEX);
    // clientUart.print((char)254, HEX);
    // clientUart.print((char)0, HEX);
    // clientUart.print((char)0, HEX);
    // clientUart.print((char)91, HEX);
    // clientUart.write(content_hex,8);
  }
  //////////////////
  if ( Bluefruit.Central.connected() )
  {
    // Serial.println("first connected.");
  // Not discovered yet
    if ( clientUart.discovered() )
    {
      // Serial.println("first data.");
      // Discovered means in working state
      // Get Serial input and send to Peripheral
      if ( clientUart.available() )
      {
        char c = clientUart.read();
        Serial.println(c);
      }
      if ( Serial.available() )
      {
        delay(2); // delay a bit for all characters to arrive
        Serial.println("Send data.");
        //////////
        // uint8_t content_send;

        // for(int i = 0; i < sizeof(content_uint); i++)
        // {
        //   content_send = content_send + content_uint[i];
        //   // clientUart.write(content[i]);
        //   // Serial.print(content[i],HEX);
        //   // Serial.println();
        // }
        // clientUart.write(content_send);
        // Serial.print(content_send);

        // clientUart.write(content, sizeof(content));
        // Serial.println(String((char *)content));
        
        
        // clientUart.print();
          // clientUart.print(0xAA);
          // clientUart.print(0x17);
          // clientUart.print(0xE8);
          // clientUart.print(0x00);
          // clientUart.print(0xFE);
          // clientUart.print(0x00);
          // clientUart.print(0x00);
          // clientUart.println(0x5B);

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


