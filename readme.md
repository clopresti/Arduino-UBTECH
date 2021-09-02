## Arduino UBTECH/JIMU Library

This library enables you to control the UBTECH servos, motors, and sensor that come
with popular JIMU robotic sets directly from an Arduino. You can chain any number of
these devices together on a single serial bus which uses only two data pins joined
together on the Arduino board. The library will handle the serial communication
protocol via a half-duplex SoftwareSerial interface. No other hardware or circuitry
is needed to handle the serial communication.

![XL-320 Servo](/images/devices.jpg)

## Library Installation
To use this library install the UBTECH-Arduino.zip archive from github releases to the 
Arduino library folder.
```
Arduino -> Sketch -> Include Library -> Add Zip Library
```

## Configuration

![XL-320 Servo](/images/servo.jpg)

The UBTECH devices work with 3V logic from the main control unit of JIMU robot kits
so I did all my testing using a bi-directional logic level shifter to convert the 
5V logic from my Arduino Uno to the 3V logic for the UBTECH devices. I have not yet
tested wether the UBTECH/JIMU servo/motor/sensors are 5V tolerant.

The UBTECH devices will work powered from the Arduino's Vin pin when the Arduino (Uno) is
connected to a USB power source. This will only provide 5V to the bus and the servos and
motors will not have full torque/speed but is good enough for testing. A better approach
is to power the Arduino from a DC 7-8V source or power the UBTECH devices directly from
an external 7-8V power source and connect common grounds with the Arduino.

Connect one of the Arduino ground pins to the ground wire of the first UBTECH device.
Connect the Vin pin on the Arduino to the power wire of the first UBTECH device.

Join together two digital pins on the Arduino (I used pins 2 and 3 on Arduino Uno)
and connect the joined wire to the 5V side of bi-directional logic shifter. Connect
the 3V side of the logic shifter to the data wire of the first UBTECH device. Connect 
5V and 3.3V pins on Arduino to the respective sides on the logic shifter along with grounds.

Make sure the data pins you use on the Arduino support change interrupts for SoftwareSerial. Refer to: https://www.arduino.cc/en/Reference/SoftwareSerial


## Example Code
```c
#include <Arduino-UBTECH.h>

#define MAX_DEVICES 32

typedef struct {
  UbtechBus::DeviceType type;
  uint8_t id;
} DeviceInfo;

//UBTECH serial bus instance
//Connected pins 2 and 3 (Arduino Uno) to data line
//  of first UBTECH device (servo/motor/sensor)
UbtechBus _ubtBus(2, 3);

//nNumber of devices connected
uint16_t deviceCount = 0;

//Store info for each connected device
DeviceInfo deviceInfo[MAX_DEVICES];

//Callback to store each discovered device
void deviceCallback(UbtechBus::DeviceType type, uint8_t id) {
  if (deviceCount < MAX_DEVICES) {
    deviceInfo[deviceCount++] = { type, id };
  }
}

//Print info for each connected device
void printDevices()
{
  if (deviceCount < 1) {
    Serial.println("No Devices Found.");
  } else {
    char msg[30]; const char* types[] = { 
      "UNKNOWN", "SERVO_H04", "MOTOR", "INFRARED", "ULTRASONIC", "TOUCH", "COLOR", "EYE_LIGHT" };
    for (int i = 0; i < deviceCount; i++) {
      DeviceInfo& d = deviceInfo[i];
      sprintf(msg, "Device: %s, Id: %d", types[d.type], d.id);
      Serial.println(msg);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial) { };

  _ubtBus.begin();

  Serial.println("Querying Devices..");
  
  //query all connected devices on the bus
  _ubtBus.queryDevices(deviceCallback);

  printDevices();
}

void loop()
{
}
```
