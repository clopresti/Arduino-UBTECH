#include "Arduino-UBTECH.h"

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



UbtechBus::UbtechBus(uint8_t rxPin, uint8_t txPin)
    : _serial(rxPin, txPin), _timeout(50)
{
}


void UbtechBus::begin()
{
    _serial.setTimeout(_timeout);
    _serial.begin(115200);
}

uint16_t UbtechBus::setTimeout(uint16_t timeout)
{
    uint16_t prevTimeout = _timeout;
    _serial.setTimeout(_timeout = timeout);
    return prevTimeout;
}

void UbtechBus::queryDevices(DeviceCallback callback) 
{
    queryServo(1, 32, callback);
    queryMotor(1, 8, callback);
    queryInfrared(1, 8, callback);
    queryUltrasonic(1, 8, callback);
    queryEyeLight(1, 8, callback);
}


bool UbtechBus::queryServo(uint8_t id)
{
    uint8_t buffer[] = { 0xFC, 0xCF, id, 0x01/*cmd*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0xED };
    buffer[8] = checksum(buffer+2, 6);
    if (!sendAndReceive(buffer, 10, 10))
        return false;
    return (buffer[0] == 0xFC && buffer[1] == 0xCF && buffer[2] == id && buffer[3] == 0xAA);
}

void UbtechBus::queryServo(uint8_t minId, uint8_t maxId, DeviceCallback callback) {
    queryDevice(DeviceType::SERVO_H04, [](uint8_t id, UbtechBus* pBus) {
        return pBus->queryServo(id);
    }, minId, maxId, callback, 10);
}

bool UbtechBus::setServoId(uint8_t oldId, uint8_t newId)
{
    uint8_t buffer[] = { 0xFA, 0xAF, oldId, 0xCD/*cmd*/, 0x00, newId, 0x00, 0x00, 0x00, 0xED };
    buffer[8] = checksum(buffer+2, 6);
    if (!sendAndReceive(buffer, 10, 10))
        return false;
    return (buffer[0] == 0xFA && buffer[1] == 0xAF && buffer[2] == oldId && buffer[3] == 0xAA);
}

int8_t UbtechBus::getServoPosition(uint8_t id)
{
    uint8_t buffer[] = { 0xFA, 0xAF, id, 0x03/*cmd*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0xED };
    buffer[8] = checksum(buffer+2, 6);
    return sendAndReceive(buffer, 10, 10) ? buffer[7] - 120 : 0x80;
}

bool UbtechBus::setServoPosition(uint8_t id, int8_t position, uint16_t time)
{
    position = (position < -118 ? 2 : position > 118 ? 138 : position + 120);
    time = (time < 300 ? 300 : time > 5000 ? 5000 : time) / 20;
    uint8_t buffer[] = { 0xFA, 0xAF, id, 0x01/*cmd*/, (uint8_t)position, (uint8_t)time, 0x00, 0x00, 0x00, 0xED };
    buffer[8] = checksum(buffer+2, 6);
    return sendAndReceive(buffer, 10, 1) && buffer[0] - 0xAA == id;
}

bool UbtechBus::setServoTurn(uint8_t id, uint16_t speed, bool clockwise)
{
    speed = speed == 1 ? 128 : speed == 2 ? 234 : speed == 3 ? 340 : speed == 4 ? 526 : speed == 5 ? 658 : 0;
    uint8_t dir = (speed == 0 ? 0xFF : clockwise ? 0xFD : 0xFE);
    uint8_t speedHi = (speed & 0xFF00) >> 8;
    uint8_t speedLo = speed & 0x00FF;
    uint8_t buffer[] = { 0xFA, 0xAF, id, 0x01/*cmd*/, dir, 0x00, speedHi, speedLo, 0x00, 0xED };
    buffer[8] = checksum(buffer+2, 6);
    return sendAndReceive(buffer, 10, 1) && buffer[0] - 0xAA == id;
}

bool UbtechBus::stopServo(uint8_t id) {
    return setServoTurn(id, 0);
}


bool UbtechBus::queryMotor(uint8_t id)
{
    uint8_t buffer[] = { 0xFB, 0x03, 0x06, 0x05/*cmd*/, id, 0x00, 0x09, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    buffer[9] = crc8_itu(buffer+1, 8);
    return sendAndReceive(buffer, 10, 17) && buffer[4] == id;
}

void UbtechBus::queryMotor(uint8_t minId, uint8_t maxId, DeviceCallback callback) {
    queryDevice(DeviceType::MOTOR, [](uint8_t id, UbtechBus* pBus) {
        return pBus->queryMotor(id);
    }, minId, maxId, callback, 10);
}

bool UbtechBus::setMotorId(uint8_t oldId, uint8_t newId)
{
    uint8_t buffer[] = { 0xFB, 0x03, 0x08/*len*/, 0x06/*cmd*/, oldId, 0x00, 0x03, 0x00, 0x01, 0x00, newId, 0x00 };
    buffer[11] = crc8_itu(buffer+1, 10);
    if (!sendAndReceive(buffer, 12, 7))
        return false;
    return (buffer[0] == 0xAC && buffer[4] == oldId);
}

bool UbtechBus::setMotorTurn(uint8_t id, int16_t speed, uint16_t time)
{
    uint8_t speedHi = speed < 0 ? 0xFF : 0x00;
    uint8_t speedLo = speed < -140 ? 116 : speed > 140 ? 140 : speed < 0 ? 256 + speed : speed;
    uint8_t timeHi = (time & 0xFF00) >> 8;
    uint8_t timeLo = time & 0x00FF;
    uint8_t buffer[] = { 0xFB, 0x03, 0x0C/*len*/, 0x06/*cmd*/, id, 0x00, 0x04, 0x00, 0x03, speedHi, speedLo, timeHi, timeLo, 0x00, 0x01, 0x00 };
    buffer[15] = crc8_itu(buffer+1, 14);
    return sendAndReceive(buffer, 16, 7) && buffer[4] == id;
}

bool UbtechBus::stopMotor(uint8_t id)
{
    uint8_t buffer[] = { 0xFB, 0x03, 0x08/*len*/, 0x06/*cmd*/, id, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00 };
    buffer[11] = crc8_itu(buffer+1, 10);
    return sendAndReceive(buffer, 12, 7) && buffer[4] == id;
}

int16_t UbtechBus::getMotorSpeed(uint8_t id)
{
    uint8_t buffer[] = { 0xFB, 0x03, 0x06/*len*/, 0x05/*cmd*/, id, 0x00, 0x07, 0x00, 0x01, 0x00 };
    buffer[9] = crc8_itu(buffer+1, 8);
    if (!sendAndReceive(buffer, 10, 9) || buffer[4] != id)
        return 0xFF;
    return buffer[6] == 0xFF ? -(256 - buffer[7]) : buffer[7];
}


bool UbtechBus::queryInfrared(uint8_t id)
{
    uint8_t buffer[] = { 0xF8, 0x8F, 0x06, 0x07/*cmd*/, id, 0x00, 0xED, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 15))
        return false;
    return buffer[0] == 0xF8 && buffer[1] == 0x8F && buffer[4] - 0xAA == id;
}

void UbtechBus::queryInfrared(uint8_t minId, uint8_t maxId, DeviceCallback callback) {
    queryDevice(DeviceType::INFRARED, [](uint8_t id, UbtechBus* pBus) {
        return pBus->queryInfrared(id);
    }, minId, maxId, callback, 10);
}

bool UbtechBus::setInfraredId(uint8_t oldId, uint8_t newId)
{
    uint8_t buffer[] = { 0xF8, 0x8F, 0x07/*len*/, 0x06/*cmd*/, oldId, newId, 0x00, 0xED };
    buffer[6] = checksum(buffer+2, 4);
    if (!sendAndReceive(buffer, 8, 7))
        return false;
    return ((buffer[0] == 0xF8) && (buffer[1] == 0x8F) && (buffer[4] - 0xAA == oldId));
}

bool UbtechBus::enableInfrared(uint8_t id, bool enable)
{
    uint8_t buffer[] = { 0xF8, 0x8F, 0x06/*len*/, (uint8_t)(enable ? 0x02 : 0x03)/*cmd*/, id, 0x00, 0xED };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 7))
        return false;
    return ((buffer[0] == 0xF8) && (buffer[1] == 0x8F) && (buffer[4] - 0xAA == id));
}

uint16_t UbtechBus::getInfraredDistance(uint8_t id)
{
    uint8_t buffer[] = { 0xF8, 0x8F, 0x06/*len*/, 0x04/*cmd*/, id, 0x00, 0xED, 0x00, 0x00 };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 9))
        return 0x0000;
    if (buffer[0] != 0xF8 || buffer[1] != 0x8F || buffer[4] - 0xAA != id)
        return 0x0000;
    return ((buffer[5] << 8) + buffer[6]);
}


bool UbtechBus::queryUltrasonic(uint8_t id)
{
    uint8_t buffer[] = { 0xF5, 0x5F, 0x06/*len*/, 0x07/*cmd*/, id, 0x00, 0xED, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 15))
        return false;
    return (buffer[0] == 0xF5 && buffer[1] == 0x5F && buffer[4] - 0xAA == id);
}

void UbtechBus::queryUltrasonic(uint8_t minId, uint8_t maxId, DeviceCallback callback) {
    queryDevice(DeviceType::ULTRASONIC, [](uint8_t id, UbtechBus* pBus) {
        return pBus->queryUltrasonic(id);
    }, minId, maxId, callback, 10);
}

bool UbtechBus::setUltrasonicId(uint8_t oldId, uint8_t newId)
{
    uint8_t buffer[] = { 0xF5, 0x5F, 0x07/*len*/, 0x06/*cmd*/, oldId, newId, 0x00, 0xED };
    buffer[6] = checksum(buffer+2, 4);
    if (!sendAndReceive(buffer, 8, 7))
        return false;
    return ((buffer[0] == 0xF5) && (buffer[1] == 0x5F) && (buffer[4] - 0xAA == oldId));
}  

bool UbtechBus::enableUltrasonic(uint8_t id, bool enable)
{
    uint8_t buffer[] = { 0xF5, 0x5F, 0x06/*len*/, (uint8_t)(enable ? 0x02 : 0x03)/*cmd*/, id, 0x00, 0xED };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 7))
        return false;
    return (buffer[0] == 0xF5 && buffer[1] == 0x5F && buffer[4] - 0xAA == id);
}

uint16_t UbtechBus::getUltrasonicDistance(uint8_t id)
{
    uint8_t buffer[] = { 0xF5, 0x5F, 0x06/*len*/, 0x04/*cmd*/, id, 0x00, 0xED, 0x00, 0x00 };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 9))
        return 0x0000;
    if (buffer[0] != 0xF5 || buffer[1] != 0x5F || buffer[4] - 0xAA != id)
        return 0x0000;
    return ((buffer[5] << 8) + buffer[6]);
}

bool UbtechBus::setUltrasonicLed(uint8_t id, Color color)
{
    bool off = (color.red == 0x00 && color.green == 0x00 && color.blue == 0x00);
    uint8_t buffer[] = { 0xF5, 0x5F, 0x0D/*len*/, 0x08/*cmd*/, id, color.red, color.green, color.blue, (uint8_t)(off ? 0x00 : 0x01), 0x00, (uint8_t)(off ? 0x00 : 0xFF), (uint8_t)(off ? 0x00 : 0xFF), 0x00, 0xED };
    buffer[12] = checksum(buffer+2, 10);
    if (!sendAndReceive(buffer, 14, 7))
        return false;
    return (buffer[0] == 0xF5 && buffer[1] == 0x5F && buffer[4] - 0xAA == id);
}

bool UbtechBus::setUltrasonicLedOff(uint8_t id) {
    return setUltrasonicLed(id, Color(0x00, 0x00, 0x00));
}


bool UbtechBus::queryEyeLight(uint8_t id)
{
    uint8_t buffer[] = { 0xF4, 0x4F, 0x06/*len*/, 0x07/*cmd*/, id, 0x00, 0xED, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 15))
        return false;
    return (buffer[0] == 0xF4 && buffer[1] == 0x4F && buffer[4] - 0xAA == id);
}

void UbtechBus::queryEyeLight(uint8_t minId, uint8_t maxId, DeviceCallback callback) {
    queryDevice(DeviceType::EYE_LIGHT, [](uint8_t id, UbtechBus* pBus) {
        return pBus->queryEyeLight(id);
    }, minId, maxId, callback, 10);
}

bool UbtechBus::setEyeLightId(uint8_t oldId, uint8_t newId)
{
    uint8_t buffer[] = { 0xF4, 0x4F, 0x07/*len*/, 0x06/*cmd*/, oldId, newId, 0x00, 0xED };
    buffer[6] = checksum(buffer+2, 4);
    if (!sendAndReceive(buffer, 8, 7))
        return false;
    return ((buffer[0] == 0xF4) && (buffer[1] == 0x4F) && (buffer[4] - 0xAA == oldId));
}

bool UbtechBus::enableEyeLight(uint8_t id, bool enable)
{
    uint8_t buffer[] = { 0xF4, 0x4F, 0x06/*len*/, (uint8_t)(enable ? 0x02 : 0x03)/*cmd*/, id, 0x00, 0xED };
    buffer[5] = checksum(buffer+2, 3);
    if (!sendAndReceive(buffer, 7, 7))
        return false;
    return ((buffer[0] == 0xF4) && (buffer[1] == 0x4F) && (buffer[4] - 0xAA == id));
}

bool UbtechBus::setEyeLightColor(uint8_t id, Color color, LED led, float nSeconds)
{
    uint8_t duration = nSeconds < 0.1 || nSeconds >= 25.5 ? 255 : (uint8_t)(nSeconds * 10.0);
    uint8_t buffer[] = { 0xF4, 0x4F, 0x0C/*len*/, 0x0B/*cmd*/, id, duration, 0x01, led, color.red, color.green, color.blue, 0x00, 0xED };
    buffer[11] = checksum(buffer+2, 9);
    if (!sendAndReceive(buffer, 13, 7))
        return false;
    return ((buffer[0] == 0xF4) && (buffer[1] == 0x4F) && (buffer[4] - 0xAA == id));
}

bool UbtechBus::setEyeLightColor(uint8_t id, Color c1, Color c2, Color c3, Color c4, Color c5, Color c6, Color c7, Color c8, float nSeconds)
{
    uint8_t duration = nSeconds < 0.1 || nSeconds >= 25.5 ? 255 : (uint8_t)(nSeconds * 10.0);  
    uint8_t idx = 7, buffer[] = { 0xF4, 0x4F, 0x28/*len*/, 0x0B/*cmd*/, id, duration, 0x08, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0x00, 0xED };
    buffer[idx++] = 0x01; buffer[idx++] = c1.red; buffer[idx++] = c1.green; buffer[idx++] = c1.blue;
    buffer[idx++] = 0x02; buffer[idx++] = c2.red; buffer[idx++] = c2.green; buffer[idx++] = c2.blue;
    buffer[idx++] = 0x04; buffer[idx++] = c3.red; buffer[idx++] = c3.green; buffer[idx++] = c3.blue;
    buffer[idx++] = 0x08; buffer[idx++] = c4.red; buffer[idx++] = c4.green; buffer[idx++] = c4.blue;
    buffer[idx++] = 0x10; buffer[idx++] = c5.red; buffer[idx++] = c5.green; buffer[idx++] = c5.blue;
    buffer[idx++] = 0x20; buffer[idx++] = c6.red; buffer[idx++] = c6.green; buffer[idx++] = c6.blue;
    buffer[idx++] = 0x40; buffer[idx++] = c7.red; buffer[idx++] = c7.green; buffer[idx++] = c7.blue;
    buffer[idx++] = 0x80; buffer[idx++] = c8.red; buffer[idx++] = c8.green; buffer[idx++] = c8.blue;
    buffer[39] = checksum(buffer+2, 37);
    if (!sendAndReceive(buffer, 41, 7))
        return false;
    return ((buffer[0] == 0xF4) && (buffer[1] == 0x4F) && (buffer[4] - 0xAA == id));
}

bool UbtechBus::setEyeLightScene(uint8_t id, Scene scene, uint8_t nTimes)
{
    uint8_t buffer[] = { 0xF4, 0x4F, 0x0C/*len*/, 0x0A/*cmd*/, id, (uint8_t)(scene + 12), 0x00, nTimes, 0x00, 0x00, 0x00, 0x00, 0xED };
    buffer[11] = checksum(buffer+2, 9);
    if (!sendAndReceive(buffer, 13, 7))
        return false;
    return ((buffer[0] == 0xF4) && (buffer[1] == 0x4F) && (buffer[4] - 0xAA == id));  
}

bool UbtechBus::setEyeLightFace(uint8_t id, Face face, Color color, uint8_t nTimes)
{
    uint8_t buffer[] = { 0xF4, 0x4F, 0x0C/*len*/, 0x0A/*cmd*/, id, face, 0x00, nTimes, color.red, color.green, color.blue, 0x00, 0xED };
    buffer[11] = checksum(buffer+2, 9);
    if (!sendAndReceive(buffer, 13, 7))
        return false;
    return ((buffer[0] == 0xF4) && (buffer[1] == 0x4F) && (buffer[4] - 0xAA == id));  
}


//Private Members

bool UbtechBus::sendAndReceive(uint8_t *buffer, uint16_t sendSize, uint16_t recvSize, bool printRecv)
{
    _serial.stopListening();
    _serial.enableTx();
    //prevent framing errors
    _serial.write((uint8_t)0);
    if (_serial.write(buffer, sendSize) != sendSize)
        return false;
    if (recvSize > 0) {
        _serial.listen();
        _serial.disableTx();
        int recvCnt = _serial.readBytes(buffer, recvSize);
        if (printRecv) printBuffer(buffer, recvCnt);
        return recvCnt == recvSize;
    }
    return true;
}

void UbtechBus::queryDevice(DeviceType type, QueryFunction query, uint8_t minId, uint8_t maxId, DeviceCallback callback, uint16_t timeout)
{
    uint16_t prevTimeout = setTimeout(timeout);
    for (uint8_t i = minId; i <= maxId; i++) {
        if (query(i, this)) {
            callback(type, i);
        } else if (query(i, this)) {
            callback(type, i);
        }
    }
    setTimeout(prevTimeout);
}  

uint8_t UbtechBus::checksum(uint8_t *buf, uint8_t len) {
    uint32_t i, sum = 0;
    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return  (uint8_t)(sum % 256);
}

uint8_t UbtechBus::_crc8(unsigned short data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x8000) {
        data = data ^ (0x1070U << 3);
        }
        data = data << 1;
    }
    return (uint8_t)(data >> 8);
}

uint32_t UbtechBus::crc8(uint32_t crc, const uint8_t *vptr, uint32_t len) {
    for (int i = 0; i < len; i++) {
        crc = _crc8((crc ^ vptr[i]) << 8);
    }
    return crc;
}

uint32_t UbtechBus::crc8_itu(const uint8_t *pBuf, uint32_t len) {
    uint32_t crc;
    crc = crc8(0, pBuf, len);
    crc ^= 0x55;
    return crc;
}

void UbtechBus::printBuffer(uint8_t* buffer, int size) {
    for (int i = 0; i < size; i++) {
        if (i > 0) Serial.print(" ");
        if (buffer[i] < 16) Serial.print("0");
        Serial.print(buffer[i], HEX);
    }
    Serial.println(";");
}
