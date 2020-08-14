#ifndef SCRATCH_MORE_SERVICE_H
#define SCRATCH_MORE_SERVICE_H

#include "MicroBit.h"

#define SCRATCH_MORE_ID 2000

#define SCRATCH_MORE_EVT_NOTIFY 1

#define MOTOR_L_A   6
#define MOTOR_L_B   16
#define MOTOR_R_A   9
#define MOTOR_R_B   7

#define SEVROPIN    10
#define BUZZERPIN   0
#define BLECONNECT  15

// UUIDs for our service and characteristics
extern const uint16_t ScratchMoreServiceUUID;
extern const uint8_t ScratchMoreServiceTxUUID[];
extern const uint8_t ScratchMoreServiceRxUUID[];

/**
  * Class definition for a MicroBitMore Service.
  * Provides a BLE service to remotely read the state of sensors from Scratch3.
  */
class ScratchMoreService
{
public:
  /**
    * Constructor.
    * Create a representation of the ScratchMoreService
    * @param _uBit The instance of a MicroBit runtime.
    */
  ScratchMoreService(MicroBit &_uBit);

  /**
    * Notify data to Scratch3.
    */
  void notify();
  void notifyDefaultData();

  /**
   * Set value to Slots.
   */
  // void setSlot(int slot, int value);

  /**
   * Get value to Slots.
   */
  // int getSlot(int slot);

  /**
    * Callback. Invoked when any of our attributes are written via BLE.
    */
  void onDataWritten(const GattWriteCallbackParams *params);

  /**
   * Invocked when the bluetooth connected.
   */
  void onBLEConnected(MicroBitEvent e);

  void updateDigitalValues();
  void updateAnalogValues();

private:

  // Data format [1,2,3] to send.
  uint8_t txDataFormat;

  // Sending data to Scratch3.
  uint8_t txData[16];

  // Recieving buffer from Scratch3.
  uint8_t rxBuffer[8];

  /**
   * Button state.
   */
  int buttonAState;
  int buttonBState;

  /**
   * Hold gesture state until next nofification.
   */
  int gesture;

  /**
   * Save the last accelerometer values to conpaire current for detecting moving.
   */
  int lastAcc[3];

  /**
   * Heading angle of compass.
   */
  int compassHeading;

  int8_t digitalValues;
  uint16_t analogValues[4];

  void servoInit();
  void motorInit();
  void buzzerInit();
  void setInputMode(int pinIndex);
  void setDigitalValue(int pinIndex, int value);
  void setAnalogValue(int pinIndex, int value);
  void setServoValue(int pinIndex, int angle, int range, int center);
  void setMotorValue(int valueL, int valueR);
  void setBuzzerValue(int pinIndex, int value);

  void onButtonChanged(MicroBitEvent);
  void onAccelerometerChanged(MicroBitEvent);

  void updateGesture(void);
  void resetGesture(void);

  int normalizeCompassHeading(int heading);
  int convertToTilt(float radians);

  void composeDefaultData(uint8_t *buff);

  void disConnected();

  // microbit runtime instance
  MicroBit &uBit;

  // Handles to access each characteristic when they are held by Soft Device.
  GattAttribute::Handle_t txCharacteristicHandle;
  GattAttribute::Handle_t rxCharacteristicHandle;

  enum ScratchBLECommand
  {
    CMD_PIN_INPUT = 0x79,
    CMD_PIN_MOTOR = 0x7A,
    CMD_PIN_SERVO = 0x7B,
    CMD_PIN_BUZZER = 0x7C,
    CMD_PIN_OUTPUT = 0x7D,
    CMD_PIN_PWM = 0x7E,
  };
};

#endif
