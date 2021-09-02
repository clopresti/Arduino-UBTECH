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
    EYE_LIGHT = 7,
  };

  typedef struct _Color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    _Color() : red(0), green(0), blue(0) {}
    _Color(uint8_t red, uint8_t green, uint8_t blue)
      : red(red), green(green), blue(blue) {}
  } Color;

  enum LED : uint8_t {
    L1 = 1,
    L2 = 2,
    L3 = 4,
    L4 = 8,
    L5 = 16,
    L6 = 32,
    L7 = 64,
    L8 = 128,
    ALL = 255
  };

  enum Scene : uint8_t {
    COLORED = 0,
    DISCO = 1,
    PRIMARY = 2,
    STACKING = 3
  };

  enum Face : uint8_t {
    BLINK = 0,
    SHY = 1,
    TEARS = 2,
    TEARS_FLASH = 3,
    CRY = 4,
    DIZZY = 5,
    HAPPY = 6,
    SURPRISED = 7,
    BREATH = 8,
    FLASH = 9,
    FAN = 10,
    WIPERS = 11
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
  bool      setUltrasonicLed(uint8_t id, Color color);
  bool      setUltrasonicLedOff(uint8_t id);

  bool      queryEyeLight(uint8_t id);
  void      queryEyeLight(uint8_t minId, uint8_t maxId, DeviceCallback callback);
  bool      setEyeLightId(uint8_t oldId, uint8_t newId);
  bool      enableEyeLight(uint8_t id, bool enable);
  bool      setEyeLightColor(uint8_t id, Color color, LED led = LED::ALL, float nSeconds = -1);
  bool      setEyeLightColor(uint8_t id, Color c1, Color c2, Color c3, Color c4, Color c5, Color c6, Color c7, Color c8, float nSeconds = -1);
  bool      setEyeLightScene(uint8_t id, Scene scene, uint8_t nTimes = 1);
  bool      setEyeLightFace(uint8_t id, Face face, Color color, uint8_t nTimes = 1);

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

static UbtechBus::LED operator |(const UbtechBus::LED& L1, const UbtechBus::LED& L2) {
  return static_cast<UbtechBus::LED>(static_cast<int>(L1) | static_cast<int>(L2));
}

#endif
