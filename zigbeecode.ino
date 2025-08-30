#include <SoftwareSerial.h>

SoftwareSerial Zigbee(0, 1); // RX, TX

void setup() {
  Serial.begin(9600);
  Zigbee.begin(9600);
  Serial.println("Zigbee Ready");
}

void loop() {
  Zigbee.println("Hello from Arduino!");
  Serial.println("ok");  //ok 신호보냄
  delay(2000);
}
