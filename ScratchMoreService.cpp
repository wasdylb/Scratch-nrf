/**
  * Class definition for the Scratch MicroBit More Service.
  * Provides a BLE service to remotely controll Micro:bit from Scratch3.
  */
#include "ScratchMoreService.h"

int gpio[] = {14, 2, 12, 4, 8, 3, 13, 1};
int analogIn[] = {2, 4, 3, 1};

/**
  * Constructor.
  * Create a representation of the ScratchMoreService
  * @param _uBit The instance of a MicroBit runtime.
  */
ScratchMoreService::ScratchMoreService(MicroBit &_uBit)
    : uBit(_uBit)
{
  uBit.display.disable();

  // Initialize pin configuration.
  for (size_t i = 0; i < sizeof(gpio) / sizeof(gpio[0]); i++)
  {
    setInputMode(gpio[i]);
  }
  for (size_t i = 0; i < sizeof(analogIn) / sizeof(analogIn[0]); i++)
  {
    setInputMode(analogIn[i]);
  }
  
  servoInit();
  motorInit();
  buzzerInit();

  // Create the data structures that represent each of our characteristics in Soft Device.
  GattCharacteristic txCharacteristic(
      ScratchMoreServiceTxUUID,
      (uint8_t *)&txData,
      0,
      sizeof(txData),
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

  GattCharacteristic rxCharacteristic(
      ScratchMoreServiceRxUUID,
      (uint8_t *)&rxBuffer,
      0,
      sizeof(rxBuffer),
      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);

  // Set default security requirements
  txCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);
  rxCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

  GattCharacteristic *characteristics[] = {&txCharacteristic, &rxCharacteristic};
  GattService service(
      ScratchMoreServiceUUID, characteristics,
      sizeof(characteristics) / sizeof(GattCharacteristic *));

  uBit.ble->addService(service);

  txCharacteristicHandle = txCharacteristic.getValueHandle();
  rxCharacteristicHandle = rxCharacteristic.getValueHandle();

  // Write initial value.
  uBit.ble->gattServer().write(
      txCharacteristicHandle,
      (uint8_t *)&txData,
      sizeof(txData));

  // Advertise this service.
  const uint16_t uuid16_list[] = {ScratchMoreServiceUUID};
  uBit.ble->gap().accumulateAdvertisingPayload(GapAdvertisingData::INCOMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));

  // Setup callbacks for events.
  if (EventModel::defaultEventBus)
  {
    EventModel::defaultEventBus->listen(MICROBIT_ID_BUTTON_A, MICROBIT_EVT_ANY, this, &ScratchMoreService::onButtonChanged, MESSAGE_BUS_LISTENER_IMMEDIATE);
    EventModel::defaultEventBus->listen(MICROBIT_ID_BUTTON_B, MICROBIT_EVT_ANY, this, &ScratchMoreService::onButtonChanged, MESSAGE_BUS_LISTENER_IMMEDIATE);
    EventModel::defaultEventBus->listen(MICROBIT_ID_ACCELEROMETER, MICROBIT_ACCELEROMETER_EVT_DATA_UPDATE, this, &ScratchMoreService::onAccelerometerChanged, MESSAGE_BUS_LISTENER_IMMEDIATE);
  }

  uBit.ble->onDataWritten(this, &ScratchMoreService::onDataWritten);

  uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_CONNECTED, this, &ScratchMoreService::onBLEConnected, MESSAGE_BUS_LISTENER_IMMEDIATE);
}

/**
  * Callback. Invoked when any of our attributes are written via BLE.
  */
void ScratchMoreService::onDataWritten(const GattWriteCallbackParams *params)
{
  uint8_t *data = (uint8_t *)params->data;

  if (params->handle == rxCharacteristicHandle && params->len > 0)
  {
    if (data[0] == ScratchBLECommand::CMD_PIN_INPUT)
    {
      setInputMode(data[1]);
    }
    else if (data[0] == ScratchBLECommand::CMD_PIN_OUTPUT)
    {
      setDigitalValue(data[1], data[2]);
    }
    else if (data[0] == ScratchBLECommand::CMD_PIN_PWM)
    {
      int value;
      memcpy(&value, &(data[2]), 2);
      setAnalogValue(data[1], value);
    }
    else if (data[0] == ScratchBLECommand::CMD_PIN_SERVO)
    {
      uint16_t angle, range, center;
      memcpy(&angle, &(data[2]), 2);
      memcpy(&range, &(data[4]), 2);
      memcpy(&center, &(data[6]), 2);
      setServoValue((int)data[1], (int)angle, (int)range, (int)center);
    }
    else if (data[0] == ScratchBLECommand::CMD_PIN_MOTOR) {
      int16_t leftspeed, rightspeed;
      memcpy(&leftspeed, &(data[1]), 2);
      memcpy(&rightspeed, &(data[3]), 2);
      setMotorValue((int)leftspeed, (int)rightspeed);
    }
    else if (data[0] == ScratchBLECommand::CMD_PIN_BUZZER) {
      int16_t tonevalue;
      memcpy(&tonevalue, &(data[2]), 2);
      setBuzzerValue(data[1], tonevalue);
    }
  }
}

/**
  * Button update callback
  */
void ScratchMoreService::onButtonChanged(MicroBitEvent e)
{
  int state;
  if (e.value == MICROBIT_BUTTON_EVT_UP)
  {
    state = 0;
  }
  if (e.value == MICROBIT_BUTTON_EVT_DOWN)
  {
    state = 1;
  }
  if (e.value == MICROBIT_BUTTON_EVT_HOLD)
  {
    state = 2;
  }
  if (e.source == MICROBIT_ID_BUTTON_A)
  {
    buttonAState = state;
  }
  if (e.source == MICROBIT_ID_BUTTON_B)
  {
    buttonBState = state;
  }
}

void ScratchMoreService::onAccelerometerChanged(MicroBitEvent)
{
  if (uBit.accelerometer.getGesture() == MICROBIT_ACCELEROMETER_EVT_SHAKE)
  {
    gesture = gesture | 1;
  }
  if (uBit.accelerometer.getGesture() == MICROBIT_ACCELEROMETER_EVT_FREEFALL)
  {
    gesture = gesture | 1 << 1;
  }
}

/**
 * Normalize angle in upside down.
 */
int ScratchMoreService::normalizeCompassHeading(int heading)
{
  if (uBit.accelerometer.getZ() > 0)
  {
    if (heading <= 180)
    {
      heading = 180 - heading;
    }
    else
    {
      heading = 360 - (heading - 180);
    }
  }
  return heading;
}

/**
 * Convert roll/pitch radians to Scratch extension value (-1000 to 1000).
 */
int ScratchMoreService::convertToTilt(float radians)
{
  float degrees = (360.0f * radians) / (2.0f * PI);
  float tilt = degrees * 1.0f / 90.0f;
  if (degrees > 0)
  {
    if (tilt > 1.0f)
      tilt = 2.0f - tilt;
  }
  else
  {
    if (tilt < -1.0f)
      tilt = -2.0f - tilt;
  }
  return (int)(tilt * 1000.0f);
}

void ScratchMoreService::updateGesture()
{
  int old[] = {lastAcc[0], lastAcc[1], lastAcc[2]};
  lastAcc[0] = uBit.accelerometer.getX();
  lastAcc[1] = uBit.accelerometer.getY();
  lastAcc[2] = uBit.accelerometer.getZ();
  int threshold = 100;
  if ((abs(lastAcc[0] - old[0]) > threshold) || (abs(lastAcc[1] - old[1]) > threshold) || (abs(lastAcc[2] - old[2]) > threshold))
  {
    // Moved
    gesture = gesture | (1 << 2);
  }
}

void ScratchMoreService::resetGesture()
{
  gesture = 0;
}

void ScratchMoreService::updateDigitalValues()
{
  digitalValues = 0;
  for (size_t i = 0; i < sizeof(gpio) / sizeof(gpio[0]); i++)
  {
    if (uBit.io.pin[gpio[i]].isInput())
    {
      digitalValues |= ((uBit.io.pin[gpio[i]].getDigitalValue(PullUp)) << i);
    }
  }
}

void ScratchMoreService::updateAnalogValues()
{
  for (size_t i = 0; i < sizeof(analogIn) / sizeof(analogIn[0]); i++)
  {
    if (uBit.io.pin[analogIn[i]].isInput())
    {
      uBit.io.pin[analogIn[i]].setPull(PullNone);
      analogValues[i] = (uint16_t)uBit.io.pin[analogIn[i]].getAnalogValue();
    }
  }
}

void ScratchMoreService::servoInit()
{
    uBit.io.pin[SEVROPIN].setServoValue(15);
}

void ScratchMoreService::motorInit()
{
    uBit.io.pin[MOTOR_L_A].setAnalogValue(0);
    uBit.io.pin[MOTOR_L_B].setAnalogValue(0);
    uBit.io.pin[MOTOR_R_A].setAnalogValue(0);
    uBit.io.pin[MOTOR_R_B].setAnalogValue(0);
}

void ScratchMoreService::buzzerInit()
{
    uBit.io.pin[BUZZERPIN].setAnalogValue(0);
}

void ScratchMoreService::setInputMode(int pinIndex)
{
  uBit.io.pin[pinIndex].getDigitalValue(); // Configure the pin as input, but the value is not used.
}

void ScratchMoreService::setDigitalValue(int pinIndex, int value)
{
  uBit.io.pin[pinIndex].setDigitalValue(value);
}

void ScratchMoreService::setAnalogValue(int pinIndex, int value)
{
  uBit.io.pin[pinIndex].setAnalogValue(value);
}

void ScratchMoreService::setServoValue(int pinIndex, int angle, int range, int center)
{
  if (range == 0)
  {
    uBit.io.pin[pinIndex].setServoValue(angle);
  }
  else if (center == 0)
  {
    uBit.io.pin[pinIndex].setServoValue(angle, range);
  }
  else
  {
    uBit.io.pin[pinIndex].setServoValue(angle, range, center);
  }
}

void ScratchMoreService::setMotorValue(int valueL, int valueR)
{
    if(valueL<0) {
        uBit.io.pin[MOTOR_L_A].setAnalogValue(0);
        uBit.io.pin[MOTOR_L_B].setAnalogValue(-valueL);
    } else {
        uBit.io.pin[MOTOR_L_A].setAnalogValue(valueL);
        uBit.io.pin[MOTOR_L_B].setAnalogValue(0);
    }
    if(valueR<0) {
        uBit.io.pin[MOTOR_R_A].setAnalogValue(0);
        uBit.io.pin[MOTOR_R_B].setAnalogValue(-valueR);
    } else {
        uBit.io.pin[MOTOR_R_A].setAnalogValue(valueR);
        uBit.io.pin[MOTOR_R_B].setAnalogValue(0);
    }
}

void ScratchMoreService::setBuzzerValue(int pinIndex, int value)
{
    if (value == 0) {
        uBit.io.pin[pinIndex].setAnalogValue(0);
    } else {
        uBit.io.pin[pinIndex].setAnalogValue(128);
        uBit.io.pin[pinIndex].setAnalogPeriodUs(int(1000000/value)); //T = 1/f
    }
}

void ScratchMoreService::composeDefaultData(uint8_t *buff)
{
  updateDigitalValues();
  updateAnalogValues();

  // Tilt value is sent as int16_t big-endian.
  int16_t tiltX = (int16_t)convertToTilt(uBit.accelerometer.getRollRadians());
  buff[0] = (tiltX >> 8) & 0xFF;
  buff[1] = tiltX & 0xFF;
  int16_t tiltY = (int16_t)convertToTilt(uBit.accelerometer.getPitchRadians());
  buff[2] = (tiltY >> 8) & 0xFF;
  buff[3] = tiltY & 0xFF;
  buff[4] = (uint8_t)gesture;
  buff[5] = (uint8_t)buttonBState;
  buff[6] = (uint8_t)buttonAState;
  buff[7] = (uint8_t)digitalValues;
  buff[8] = (analogValues[0] >> 8) & 0xFF;
  buff[9] = analogValues[0] & 0xFF;
  buff[10] = (analogValues[1] >> 8) & 0xFF;
  buff[11] = analogValues[1] & 0xFF;
  buff[12] = (analogValues[2] >> 8) & 0xFF;
  buff[13] = analogValues[2] & 0xFF;
  buff[14] = (analogValues[3] >> 8) & 0xFF;
  buff[15] = analogValues[3] & 0xFF;
}

/**
 * Notify default micro:bit data to Scratch.
 */
void ScratchMoreService::notifyDefaultData()
{
  composeDefaultData(txData);
  uBit.ble->gattServer().notify(
      txCharacteristicHandle,
      (uint8_t *)&txData,
      sizeof(txData) / sizeof(txData[0]));
}

/**
  * Notify data to Scratch3
  */
void ScratchMoreService::notify()
{
  if (uBit.ble->gap().getState().connected)
  {
    updateGesture();
    notifyDefaultData();
    resetGesture();
  }
  else
  {
    disConnected();
  }
}

void ScratchMoreService::onBLEConnected(MicroBitEvent e)
{
  uBit.io.pin[BLECONNECT].setDigitalValue(0);
}

void ScratchMoreService::disConnected()
{
  if (uBit.systemTime() / 500 % 2)
  {
    uBit.io.pin[BLECONNECT].setDigitalValue(0);
  }
  else
  {
    uBit.io.pin[BLECONNECT].setDigitalValue(1);
  }
}

const uint16_t ScratchMoreServiceUUID = 0xf005;

const uint8_t ScratchMoreServiceTxUUID[] = {
    0x52, 0x61, 0xda, 0x01, 0xfa, 0x7e, 0x42, 0xab, 0x85, 0x0b, 0x7c, 0x80, 0x22, 0x00, 0x97, 0xcc};

const uint8_t ScratchMoreServiceRxUUID[] = {
    0x52, 0x61, 0xda, 0x02, 0xfa, 0x7e, 0x42, 0xab, 0x85, 0x0b, 0x7c, 0x80, 0x22, 0x00, 0x97, 0xcc};
