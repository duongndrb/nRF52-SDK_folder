#include <Arduino.h>

#define MY_ENABLE_SERIAL
#define MY_BAUD_RATE     115200

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
#ifdef MY_ENABLE_SERIAL
  Serial.begin(MY_BAUD_RATE);
  while(!Serial);
  Serial.println("Pro Mini nRF52 - Blink Onboard LEDs");
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_LED1, HIGH);
#ifdef MY_ENABLE_SERIAL
  Serial.println("Blink!");
#endif
  delay(1000);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, HIGH);
#ifdef MY_ENABLE_SERIAL
  Serial.println("Blink!");
#endif
  delay(1000);
  digitalWrite(PIN_LED2, LOW);
}
