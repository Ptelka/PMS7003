#ifndef PMS7003_H
#define PMS7003_H

/*
PMS7003.h - Library for reading measurements from PMS7003 sensor
Created by Piotr K. Telka, December 20, 2019.
Released into the public domain.
*/

#include <array>
#include <functional>

namespace pms {

struct Measurements {
    unsigned int PM1_0;
    unsigned int PM2_5;
    unsigned int PM10_0;
    bool is_ok = false;
};

namespace mode {
struct Passive {} passive;
struct Active {} active;
}

namespace command {
static std::array<unsigned char, 7> wakeup {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
static std::array<unsigned char, 7> sleep {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
static std::array<unsigned char, 7> request {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
static std::array<unsigned char, 7> set_passive {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
static std::array<unsigned char, 7> set_active {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
}

template<typename SerialInput>
class Sensor {
public:
    Sensor(SerialInput &serial, mode::Passive);
    Sensor(SerialInput &serial, mode::Active);

    Measurements read();
    void read(Measurements& measurements);
    void sleep();
    void wakeup();

private:
    static constexpr int MAX_FRAME_LEN = 32;

    SerialInput &serial;

    unsigned int bytes = 0;
    unsigned char currentByte = 0;
    unsigned char previousByte = 0;
    unsigned int currentFrameLength = MAX_FRAME_LEN;
    unsigned int checksum = 0;

    void request_data();
    void do_nothing() {}

    void (Sensor::*prepare)();
    void drain();
    void read_next();
    bool sync();
    void fill(Measurements &measurements);
    void reset();
};

template<typename SerialInput>
Sensor<SerialInput>::Sensor(SerialInput &serial, mode::Active)
: serial(serial) {
    serial.write(command::set_active.data(), command::set_active.size());
    prepare = &Sensor::do_nothing;
}

template<typename SerialInput>
Sensor<SerialInput>::Sensor(SerialInput &serial, mode::Passive)
: serial(serial) {
    serial.write(command::set_passive.data(), command::set_passive.size());
    prepare = &Sensor::request_data;
}

template<typename SerialInput>
Measurements Sensor<SerialInput>::read() {
    Measurements measurements;
    read(measurements);
    return measurements;
}

template<typename SerialInput>
void Sensor<SerialInput>::read(Measurements& measurements) {
    bool synced = false;

    if (serial.available() > MAX_FRAME_LEN) {
        drain();
    }

    (this->*(prepare))();

    do {
        if (serial.available() < 1) {
            continue;
        }

        read_next();

        if (!synced) {
            synced = sync();
        } else {
            fill(measurements);
        }
    } while (bytes < currentFrameLength);

    reset();
}

template<typename SerialInput>
void Sensor<SerialInput>::drain() {
#ifdef Debug
    Serial.print("PMS7003: draining: ");
    Serial.println(serial.available());
#endif
    while(serial.available()) {
        serial.read();
    }
}

template<typename SerialInput>
void Sensor<SerialInput>::read_next() {
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
bool Sensor<SerialInput>::sync() {
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
void Sensor<SerialInput>::fill(Measurements& measurements) {
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
void Sensor<SerialInput>::reset() {
    bytes = 0;
    currentFrameLength = MAX_FRAME_LEN;
    checksum = 0;
}

template<typename SerialInput>
void Sensor<SerialInput>::sleep() {
    serial.write(command::sleep.data(), command::sleep.size());
}

template<typename SerialInput>
void Sensor<SerialInput>::wakeup() {
    serial.write(command::wakeup.data(), command::wakeup.size());
}

template<typename SerialInput>
void Sensor<SerialInput>::request_data() {
    serial.write(command::request.data(), command::request.size());
}

}   // namespace pms

#endif
