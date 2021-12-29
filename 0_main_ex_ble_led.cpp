#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>

BLEPeripheral ledPeripheral = BLEPeripheral();

BLEService ledService = BLEService("0111");
BLECharCharacteristic ledCharacteristic = BLECharCharacteristic("0111", BLERead | BLEWrite);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  ledPeripheral.setAdvertisedServiceUuid(ledService.uuid());
  ledPeripheral.addAttribute(ledService);
  ledPeripheral.addAttribute(ledCharacteristic);
  ledPeripheral.setLocalName("Nordic NRF52 DK");
  ledPeripheral.begin();
}

void loop()
{
  BLECentral central = ledPeripheral.central();

  if (central) {
    while (central.connected()) {
      if (ledCharacteristic.written()) {
        if (ledCharacteristic.value()) {
          digitalWrite(LED_BUILTIN, HIGH);
        }
        else{
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
    }
  }
}
