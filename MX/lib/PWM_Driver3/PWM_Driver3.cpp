

#include <PWM_Driver3.h>



#define WIRE Wire

// Set to true to print some debug messages, or false to disable them.
#define ENABLE_DEBUG_OUTPUT false

/*PWM_Driver::PWM_Driver(uint8_t addr) {
  _i2caddr = addr;
}*/


PWM_Driver::PWM_Driver(uint8_t bloc) {

  _i2caddr[0] = 0x40 + 0 + (bloc-1) * pca_per_block;
  _i2caddr[1] = 0x40 + 1 + (bloc-1) * pca_per_block;
  _i2caddr[2] = 0x40 + 2 + (bloc-1) * pca_per_block;
  _i2caddr[3] = 0x40 + 3 + (bloc-1) * pca_per_block;
}


void PWM_Driver::begin(void) {
 WIRE.begin();
 reset();
}


void PWM_Driver::reset(void) {
 write8(0xFA, 0x0);
 write8(0xFB, 0x0);
 write8(0xFC, 0x0);
 write8(0xFD, 0x0);
 write8(PCA9685_MODE1, 0x0);
}

void PWM_Driver::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  if (ENABLE_DEBUG_OUTPUT) {
    //Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
  }
  uint8_t prescale = floor(prescaleval + 0.5);
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Final pre-scale: "); Serial.println(prescale);
  }

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void PWM_Driver::setRGBLPWM(uint8_t num,uint16_t r,uint16_t g, uint16_t b, uint16_t l){
	setPWM(num*4  ,l);
	setPWM(num*4-1,b);
	setPWM(num*4-2,g);
	setPWM(num*4-3,r);
}


/**
num: led de 1 a 64
*/
void PWM_Driver::setPWM(uint8_t i, uint16_t off) {

  static uint16_t off_prev=0;
  static uint8_t i_prev=0;

  if ( i!=i_prev || off!=off_prev ){
    off_prev=off;
    i_prev=i;

    uint8_t pca_num, id_led;

    pca_num = (i-1) / register_per_pca;  //donne l'id du pca d'un bloc [0 a 3]
    id_led = (i-1) % register_per_pca;

    WIRE.beginTransmission(_i2caddr[pca_num]);
    WIRE.write(LED0_ON_L + 4 * id_led);
    WIRE.write(0);
    WIRE.write(0>>8);
    WIRE.write(off);
    WIRE.write(off>>8);
    WIRE.endTransmission();
  }
}



uint8_t PWM_Driver::read8(uint8_t addr) {

  WIRE.beginTransmission(_i2caddr[0]);
  WIRE.write(addr);
  WIRE.endTransmission();
  WIRE.requestFrom((uint8_t)_i2caddr[0], (uint8_t)1);

  WIRE.beginTransmission(_i2caddr[1]);
  WIRE.write(addr);
  WIRE.endTransmission();
  WIRE.requestFrom((uint8_t)_i2caddr[1], (uint8_t)1);

  WIRE.beginTransmission(_i2caddr[2]);
  WIRE.write(addr);
  WIRE.endTransmission();
  WIRE.requestFrom((uint8_t)_i2caddr[2], (uint8_t)1);

  WIRE.beginTransmission(_i2caddr[3]);
  WIRE.write(addr);
  WIRE.endTransmission();
  WIRE.requestFrom((uint8_t)_i2caddr[3], (uint8_t)1);


  return WIRE.read();

}

void PWM_Driver::write8(uint8_t addr, uint8_t d) {
  WIRE.beginTransmission(_i2caddr[0]);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();

  WIRE.beginTransmission(_i2caddr[1]);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();

  WIRE.beginTransmission(_i2caddr[2]);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();

  WIRE.beginTransmission(_i2caddr[3]);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();

}
