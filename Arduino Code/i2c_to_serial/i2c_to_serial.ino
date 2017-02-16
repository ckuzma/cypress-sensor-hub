#include <Wire.h>

/*  Wiring for Arduino Uno/101 to the
 *  Cypress PSoC Analog Coprocessor
 *  Pioneer Kit (CY8CKIT-048):
 *
 *  Arduino A5 ---> 4.0 Pioneer
 *  Arduino A4 ---> 4.1 Pioneer
 *  Arduino 5V ---> VIN Pioneer
 *  Arduino GND --> GND Pioneer
 */

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  Wire.requestFrom(0x08, 32);    // request 32 bytes from slave device 0x08

  while (Wire.available()) {
    float singleByte = Wire.read(); // receive a byte as a float
    Serial.print(singleByte);
    Serial.print(",");
  }
  Serial.println();
  
  delay(20);
}
