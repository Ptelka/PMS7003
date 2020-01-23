#include "PMS7003.h"
#include <SoftwareSerial.h>

SoftwareSerial softwareSerial(5, 6);
pms::Sensor<SoftwareSerial> pms7003(softwareSerial);

void setup() {
    Serial.begin(57600);
}

void loop() {
    Serial.println("trying to read from sensor");

    auto measurements = pms7003.read();

    if (measurements.is_ok) {
        Serial.print("PM 1: ");
        Serial.println(measurements.atmospheric.PM1_0);
        Serial.print("PM 2.5: ");
        Serial.println(measurements.atmospheric.PM2_5);
        Serial.print("PM 10.0: ");
        Serial.println(measurements.atmospheric.PM10_0);
    }
}
