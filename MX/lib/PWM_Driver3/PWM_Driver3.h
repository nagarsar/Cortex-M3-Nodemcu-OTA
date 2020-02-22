/***************************************************
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _PWM_Driver_H
#define _PWM_Driver_H


#include <Wire.h>

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif


#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define register_per_pca 16
#define pca_per_block 4
#define block_per_projet 5  //[0-16]

class PWM_Driver {
 public:
  //PWM_Driver(uint8_t addr = 0x40);
  PWM_Driver(uint8_t bloc=1);
  void begin(void);
  void reset(void);
  void setPWMFreq(float freq);
  void setPWM(uint8_t num, uint16_t off);
  void setRGBLPWM(uint8_t num,uint16_t r,uint16_t g, uint16_t b, uint16_t l);
  void setPin(uint8_t num, uint16_t val, bool invert=false);



 private:
  uint8_t _i2caddr[pca_per_block];


  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);
};






#endif
