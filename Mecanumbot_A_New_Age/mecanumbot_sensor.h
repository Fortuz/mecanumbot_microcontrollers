#ifndef MECANUMBOT_SENSOR_H
#define MECANUMBOT_SENSOR_H

#include <IMU.h>
#include "OLLO.h"

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8

typedef struct LED_PIN_ARRAY
{
  int front_left;
  int front_right;
  int back_left;
  int back_right;
}LedPinArray;

class MecanumbotSensor
{
 public:
  MecanumbotSensor();
  ~MecanumbotSensor();

  bool init(void);

  // IMU
  void initIMU(void);
  float* getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);

  float* getImuAngularVelocity(void);
  float* getImuLinearAcc(void);
  float* getImuMagnetic(void);
  float* getOrientation(void);
  
  // Battery
  float checkVoltage(void);

  // Sound
  void onMelody();
  void makeMelody(uint8_t index);  

  // led pattern
  void initLED(void);
  void setLedPattern(double linear_vel, double angular_vel);
 private:
  cIMU imu_;
  OLLO ollo_;

  LedPinArray led_pin_array_; 

  bool is_melody_play_complete_;
  uint16_t melody_note_[8];
  uint8_t melody_duration_[8];
};

#endif