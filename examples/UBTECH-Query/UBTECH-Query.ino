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
      "UNKNOWN", "SERVO_H04", "MOTOR", "INFRARED", "ULTRASONIC", "TOUCH", "COLOR", "LED_EYE", "SPEAKER" };
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
