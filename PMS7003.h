#ifndef PMS7003_H
#define PMS7003_H

/*
  PMS7003.h - Library for reading measurements from PMS7003 sensor
  Created by Piotr K. Telka, December 20, 2019.
  Released into the public domain.
*/

namespace pms {

struct Measurements {
    unsigned int PM1_0;
    unsigned int PM2_5;
    unsigned int PM10_0;
    unsigned int is_ok = false;
};

template<typename SerialInput>
class PMS7003 {
public:
    PMS7003(SerialInput &serial);

    Measurements read();
    void read(Measurements& measurements);

private:
    static constexpr int MAX_FRAME_LEN = 32;

    SerialInput &serial;

    unsigned int bytes = 0;
    unsigned char currentByte = 0;
    unsigned char previousByte = 0;
    unsigned int currentFrameLength = MAX_FRAME_LEN;
    unsigned int checksum = 0;

    void drain();
    void readNext();
    bool sync();
    void fill(Measurements &measurements);
    void reset();
};

template<typename SerialInput>
PMS7003<SerialInput>::PMS7003(SerialInput &serial)
: serial(serial) { }

template<typename SerialInput>
Measurements PMS7003<SerialInput>::read() { 
    Measurements measurements;
    read(measurements);
    return measurements;
}

template<typename SerialInput>
void PMS7003<SerialInput>::read(Measurements& measurements) {
    bool synced = false;

    do {
        if (serial.available() > MAX_FRAME_LEN) {
            drain();
        }

        if (serial.available() < 1) {
            continue;
        }

        readNext();

        if (!synced) {
            synced = sync();
        } else {
            fill(measurements);
        }
    } while (bytes < currentFrameLength);

    reset();
}

template<typename SerialInput>
void PMS7003<SerialInput>::drain() {
    Serial.print("PMS7003: draining: ");
    Serial.println(serial.available());
    while(serial.available()) {
        serial.read();
    }
}

template<typename SerialInput>
void PMS7003<SerialInput>::readNext() {
    previousByte = currentByte;
    currentByte = serial.read();
    checksum += currentByte;
    ++bytes;
#ifdef Debug
    Serial.print("PMS7003: got ");
    Serial.println(currentByte, HEX);
#endif
}

template<typename SerialInput>
bool PMS7003<SerialInput>::sync() {
    if (bytes == 2 && currentByte == 0x4D && previousByte == 0x42) {
#ifdef Debug
      Serial.println("PMS7003: Synced");
#endif
      return true;
    }
    
    if(bytes < 2) {
      return false;
    }
    
    bytes = 0;
    return false;
}

template<typename SerialInput>
void PMS7003<SerialInput>::fill(Measurements& measurements) {
    unsigned int val = (currentByte & 0xff) + (previousByte << 8);
    switch (bytes) {
        case 4:
            currentFrameLength = val + bytes;
            break;
        case 6:
            measurements.PM1_0 = val;
            break;
        case 8:
            measurements.PM2_5 = val;
            break;
        case 10:
            measurements.PM10_0 = val;
            break;
        case 32:
            checksum -= ((val >> 8) + (val & 0xFF));
            measurements.is_ok = val == checksum;
            break;
        default:
            break;
    }
}

template<typename SerialInput>
void PMS7003<SerialInput>::reset() {
    bytes = 0;
    currentFrameLength = MAX_FRAME_LEN;
    checksum = 0;
}

}   // namespace pms

#endif
