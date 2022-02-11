
// Import the library 
#include <Wire.h>
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "heartRate.h"


#include <filters.h>

const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

// Low-pass filter
Filter filterEcg(cutoff_freq, sampling_time, order);


MAX30105 particleSensor; // initialize MAX30102 with I2C

BLEClientUart clientUart; // bleuart client

// BLE Service
BLEUart bleuart; //uart over ble
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEBas  blebas;  // battery

uint32_t rx_count = 0;

String deviceName = "ElcomEcg";

int start_send_data_ecg             = 's'
            , stop_send_data_ecg        = 't'
            , start_send_data_spo2      = 'n'
            , stop_send_data_spo2       = 'm'
            , send_real_time            = 'u'
            , send_not_real_time        = 'v'
            , read_battery              = 'b';

int k = 0;

// setting up for the heart Rate calculate
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;


void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void startAdv(void);

void copyBigEndianArray(uint16_t *inArray, size_t inItems, uint8_t *outArray);

void setup()
{
  Serial.begin(115200);
  while(!Serial); //We must wait for Teensy to come online
  delay(100);

  // set pin for the ECG read data
  pinMode(17, INPUT); // Setup for leads off detection LO +
  pinMode(18, INPUT); // Setup for leads off detection LO - 
  
#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial ) yield();
#endif

  Serial.println("Bluefruit52 Peripheral BLEUART");
  Serial.println("-----------------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);  

  Bluefruit.begin();
  
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("ElcomEcg"); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  // bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  // bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");

  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  // byte ledBrightness = 70; //Options: 0=Off to 255=50mA
  // byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  // byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  // int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  // int pulseWidth = 69; //Options: 69, 118, 215, 411
  // int adcRange = 16384; //Options: 2048, 4096, 8192, 16384

  // // particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

  analogReadResolution(10);
  
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


uint32_t long_time;
bool flag_start_read_ecg = false;
bool flag_start_read_spo2 = false;

//////////
const unsigned int numReadings = 9;
unsigned int dataECG[numReadings];
unsigned int dataSPO2[numReadings];
unsigned int i = 0;

uint16_t bufferIn[10];
// float filteredval[10];
uint8_t bufferOut[20]; 
//////////

void loop()
{
  /////////////
  ///// READ TEMPERATURE
  float temperature = particleSensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);

  Serial.println();

  /////////////
  ///// READ BPM
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }

    // bufferIn[i+1] = beatAvg;
    // i++;
    // if (i>=numReadings)
    // {
    //   i=0; //reset to beginning of array, so you don't try to save readings outside of the bounds of the array
    //   bufferIn[0] = 3;
    //   copyBigEndianArray(bufferIn, 10, bufferOut);
    //   bleuart.write(bufferOut,20);
    // }


  }

  // Serial.print("IR=");
  // Serial.print(irValue);
  // Serial.print(", BPM=");
  // Serial.print(beatsPerMinute);
  // Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
  {
    Serial.print(" No finger?");
  }
  Serial.println();
  /////////////////////
  ///// ECG GRAPH
  if(flag_start_read_ecg){

    // int dataEcg = analogRead(A2);
    // float filteredval = filterEcg.filterIn(dataEcg);
    // Serial.print(dataEcg);
    // Serial.print(",");
    // Serial.println(filteredval);
    // delay(8);

    
    dataECG[i] = analogRead(A2);
    float filteredval = filterEcg.filterIn(dataECG[i]);
    Serial.print(dataECG[i]);
    Serial.print(",");
    Serial.println(filteredval);
    delay(8);

    // bufferIn[i+1] = dataECG[i];
    bufferIn[i+1] = filteredval;
    i++;
    if (i>=numReadings)
    {
      i=0; //reset to beginning of array, so you don't try to save readings outside of the bounds of the array
      bufferIn[0] = 1;
      copyBigEndianArray(bufferIn, 10, bufferOut);
      bleuart.write(bufferOut,20);
    }
    // delay(8);
    
  }
  ///////////////
  ///// SPO2 GRAPH
  if(flag_start_read_spo2)
  {
    // particleSensor.check(); //Check the sensor
    // // read stored red
    Serial.println(particleSensor.getRed());
    // // read next set of samples
    // particleSensor.nextSample(); 

    dataSPO2[i] = particleSensor.getRed();
    bufferIn[i+1] = dataSPO2[i];
    i++;
    if (i>=numReadings)
    {
      i=0; //reset to beginning of array, so you don't try to save readings outside of the bounds of the array
      bufferIn[0] = 2;
      copyBigEndianArray(bufferIn, 10, bufferOut);
      bleuart.write(bufferOut,20);
    }
    delay(10); 
  }

  
  

  if(millis() - long_time > 1000)
  {
    long_time = millis();

    while(Serial.available())
    {
      // Delay to wait for enough input, since we have a limited transmission buffer
      delay(2);

      uint8_t buf[64];
      int count = Serial.readBytes(buf, sizeof(buf));
      bleuart.write( buf, count );
      Serial.print(count);
    }

    // Forward from BLEUART to HW Serial
    while ( bleuart.available() )
    {
      int send;
      send = bleuart.read();
      Serial.println(send);
      bleuart.print(send);
      if (send == start_send_data_spo2)
      {
        flag_start_read_spo2 = true; 
      }
      if(send == stop_send_data_spo2)
      {
        flag_start_read_spo2 = false;
      }

      if(send == start_send_data_ecg)
      {
        flag_start_read_ecg = true;
      }
      if(send == stop_send_data_ecg)
      {
        flag_start_read_ecg = false;
      }    
    }
  }

}
/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
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
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

/* copy inItems UINT16 values from inArray to outArray in
 * MSB first (big-endian) order 
 */
void copyBigEndianArray(uint16_t *inArray, size_t inItems, uint8_t *outArray)
{
    for (int i = 0; i < inItems; i++)
    {
        // shift one byte right: AAFF -> 00AA
        outArray[2*i]     = inArray[i] >> 8; 

        // cut off left byte in conversion: AAFF -> FF
        outArray[2*i + 1] = inArray[i];       
    }
}

