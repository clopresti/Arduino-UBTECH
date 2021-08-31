#include <Arduino.h>
#include <SoftwareSerial.h>

/*
This library enables you to control the UBTECH servos, motors, and sensor that come
with popular JIMU robotic sets directly from an Arduino. You can chain any number of
these devices together on a single serial bus which uses only two data pins joined
together on the Arduino board. The library will handle the serial communication
protocol via a half-duplex SoftwareSerial interface. No other hardware or circuitry
is needed to handle the serial communication.

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

Make sure the data pins you use on the Arduino support change interrupts for SoftwareSerial.
Refer to: https://www.arduino.cc/en/Reference/SoftwareSerial
*/


#ifndef __UbtechBus__
#define __UbtechBus__

class UbtechBus
{
public:

  enum DeviceType : uint8_t {
    UNKNOWN = 0,
    SERVO_H04 = 1,
    MOTOR = 2,
    INFRARED = 3,
    ULTRASONIC = 4,
    TOUCH = 5,
    COLOR = 6,
    LED_EYE = 7,
    SPEAKER = 8,
  };

  typedef void (*DeviceCallback)(DeviceType type, uint8_t id);

  // constructor
  UbtechBus(uint8_t rxPin, uint8_t txPin);

  void      begin();
  uint16_t  setTimeout(uint16_t timeout);
  void      queryDevices(DeviceCallback callback);

  bool      queryServo(uint8_t id);
  void      queryServo(uint8_t minId, uint8_t maxId, DeviceCallback callback);
  bool      setServoId(uint8_t oldId, uint8_t newId);
  int8_t    getServoPosition(uint8_t id);  
  bool      setServoPosition(uint8_t id, int8_t position, uint16_t time = 400);
  bool      setServoTurn(uint8_t id, uint16_t speed, bool clockwise = true);
  bool      stopServo(uint8_t id);

  bool      queryMotor(uint8_t id);
  void      queryMotor(uint8_t minId, uint8_t maxId, DeviceCallback callback);
  bool      setMotorId(uint8_t oldId, uint8_t newId);
  bool      setMotorTurn(uint8_t id, int16_t speed, uint16_t time = 0xFFFF);
  bool      stopMotor(uint8_t id);
  int16_t   getMotorSpeed(uint8_t id);

  bool      queryInfrared(uint8_t id);
  void      queryInfrared(uint8_t minId, uint8_t maxId, DeviceCallback callback);
  bool      setInfraredId(uint8_t oldId, uint8_t newId);
  bool      enableInfrared(uint8_t id, bool enable);
  uint16_t  getInfraredDistance(uint8_t id);

  bool      queryUltrasonic(uint8_t id);
  void      queryUltrasonic(uint8_t minId, uint8_t maxId, DeviceCallback callback);
  bool      setUltrasonicId(uint8_t oldId, uint8_t newId);
  bool      enableUltrasonic(uint8_t id, bool enable);
  uint16_t  getUltrasonicDistance(uint8_t id);
  bool      setUltrasonicLed(uint8_t id, uint8_t red, uint8_t green, uint8_t blue);
  bool      setUltrasonicLedOff(uint8_t id);

private:

  typedef bool (*QueryFunction)(uint8_t id, UbtechBus* pBus);

  bool sendAndReceive(uint8_t *buffer, uint16_t sendSize, uint16_t recvSize, bool printRecv = false);
  void queryDevice(DeviceType type, QueryFunction query, uint8_t minId, uint8_t maxId, DeviceCallback callback, uint16_t timeout);
  
  static uint8_t checksum(uint8_t *buf, uint8_t len);
  static uint8_t _crc8(unsigned short data);
  static uint32_t crc8(uint32_t crc, const uint8_t *vptr, uint32_t len);
  static uint32_t crc8_itu(const uint8_t *pBuf, uint32_t len);
  static void printBuffer(uint8_t* buffer, int size);
  
  class HalfDuplexSoftwareSerial : public SoftwareSerial
  {
    public:
      HalfDuplexSoftwareSerial(uint8_t rxPin, uint8_t txPin):
        SoftwareSerial(rxPin, txPin), _txPin(txPin) {
      }
      void enableTx() { pinMode(_txPin, OUTPUT); }
      void disableTx() { pinMode(_txPin, INPUT); }
    private:
      uint8_t _txPin;
  };

  uint16_t _timeout;
  HalfDuplexSoftwareSerial _serial;
};

#endif
