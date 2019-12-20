#include "PMS7003.h"
#include <SoftwareSerial.h>

SoftwareSerial softwareSerial(5, 6);
pms::PMS7003<SoftwareSerial> pms7003(softwareSerial);

void setup() {
    Serial.begin(57600);
    softwareSerial.begin(9600);
}

void loop() {
    Serial.println("trying to read from sensor");

    auto measurements = pms7003.read();

    if (measurements.is_ok) {
        Serial.print("PM 1: ");
        Serial.println(measurements.PM1_0);
        Serial.print("PM 2.5: ");
        Serial.println(measurements.PM2_5);
        Serial.print("PM 10.0: ");
        Serial.println(measurements.PM10_0);
    }
}