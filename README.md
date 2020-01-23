# PMS7003

Library for reading measurements from PMS7003 sensor.

PMS7003 sensor can be used in two modes:
- active (default) in which sensor writes data through serial automatically
- passive in which request command must be send to retrieve data

You can specify mode in the Sensor class constructor:
```cpp
pms::Sensor<SoftwareSerial> pms7003(softwareSerial, pms::mode::active);
```

Or use the `set_mode()` method
```cpp
pms::Sensor<SoftwareSerial> pms7003(softwareSerial);

pms7003.set_mode(pms::mode::active);
```

In the passive mode Sensor automatically sends request for data after invoking read method.