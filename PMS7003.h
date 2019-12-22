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

struct Concentration {
    unsigned int PM1_0;
    unsigned int PM2_5;
    unsigned int PM10_0;
};

struct Measurements {
    Concentration factory;
    Concentration atmospheric;
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

    void print_response();
    void write(const std::array<unsigned char, 7>& cmd);
};

template<typename SerialInput>
Sensor<SerialInput>::Sensor(SerialInput &serial, mode::Active)
: serial(serial) {
#ifdef Debug
    Serial.println("PMS7003: Setting sensor to active mode");
#endif
    serial.begin(9600);
    write(command::set_active);
    prepare = &Sensor::do_nothing;
}

template<typename SerialInput>
Sensor<SerialInput>::Sensor(SerialInput &serial, mode::Passive)
: serial(serial) {
#ifdef Debug
    Serial.println("PMS7003: Setting sensor to passive mode");
#endif
    serial.begin(9600);
    write(command::set_passive);
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
     // TODO: read everything into the buffer
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
     // TODO: read all data (number of particles in 0.1l of air)
    switch (bytes) {
        case 4:
            currentFrameLength = val + bytes;
            break;
        case 6:
            measurements.factory.PM1_0 = val;
            break;
        case 8:
            measurements.factory.PM2_5 = val;
            break;
        case 10:
            measurements.factory.PM10_0 = val;
            break;
        case 12:
            measurements.atmospheric.PM1_0 = val;
            break;
        case 14:
            measurements.atmospheric.PM2_5 = val;
            break;
        case 16:
            measurements.atmospheric.PM10_0 = val;
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
#ifdef Debug
    Serial.println("PMS7003: sleep");
#endif
   write(command::sleep);
}

template<typename SerialInput>
void Sensor<SerialInput>::wakeup() {
#ifdef Debug
    Serial.println("PMS7003: wakeup");
#endif
    write(command::wakeup);
}

template<typename SerialInput>
void Sensor<SerialInput>::request_data() {
#ifdef Debug
    Serial.println("PMS7003: requesting data");
#endif
    write(command::request);
}

template<typename SerialInput>
void Sensor<SerialInput>::write(const std::array<unsigned char, 7>& cmd) {
    serial.write(cmd.data(), cmd.size());
#ifdef Debug
    print_response();
#endif
}

template<typename SerialInput>
void Sensor<SerialInput>::print_response() {
    Serial.print("PMS7003: response: ");
    while(serial.available()) {
        Serial.print("0x");
        Serial.print((unsigned char)serial.read() & 0xFF, HEX);
        Serial.print(" ");
    }
    Serial.println();
}

}   // namespace pms

#endif
