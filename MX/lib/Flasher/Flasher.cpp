#include "Flasher.h"
#include <stdio.h>
#include <string.h>
#include <Arduino.h>


#define Serial_Available()  Serial1.available()
#define Serial_Read()       Serial1.read()
#define Serial_Printf       Serial.printf

#define PIN_RGB_R		61
#define PIN_RGB_G		62
#define PIN_RGB_B		63
#define PIN_RGB_LUM	64

PWM_Driver pwm;

Metro blink = Metro(1000);// milliseconds
Metro display = Metro(100);// milliseconds


const int ledPin = 13;
const uint8_t flashing_code=0x1;
static CAN_message_t msg;
char line[45];
int count = 0;






void Flasher::upgrade_firmware(const char *type){

  Serial_Printf("%s flash size = %dK in %dK sectors\n", FLASH_ID, FLASH_SIZE / 1024, FLASH_SECTOR_SIZE / 1024);

  flash_erase_upper();   // erase upper half of flash

  if ((uint32_t)flash_word < FLASH_SIZE || (uint32_t)flash_erase_sector < FLASH_SIZE || (uint32_t)flash_move < FLASH_SIZE) {
    Serial_Printf("routines not in ram\n");
    return;
  }

  // what is currently used?
  int32_t addr = FLASH_SIZE / 2;
  while (addr > 0 && *((uint32_t *)addr) == 0xFFFFFFFF)
    addr -= 4;
  Serial_Printf("current firmware 0:%x\n", addr + 4);

  if (addr > FLASH_SIZE / 2 - RESERVE_FLASH) {
    Serial_Printf("firmware is too large\n");
    return;
  }

  Serial_Printf("WARNING: this can ruin your device\n");
  Serial_Printf("waiting for intel hex lines\n");

  // read in hex lines
  Serial_Printf( "%s\n",type );

  if(strcmp("SERIAL", type) == 0) {
      waiting_for_serial();
  }
  else if(strcmp("CAN", type) == 0) {
      waiting_for_can();
  }

} // upgrade_firmware()



// Waiting for Serial
//***************************
void Flasher::waiting_for_serial(){

  // Pass the boolean to TRUE !
  _flashing_state = true;


  char line[45];
  uint8_t count = 0;

  for (;;)  {
    while (!Serial_Available()) {}

    byte c = Serial_Read();
    if (count % 10 == 1) digitalWrite(ledPin, !digitalRead(ledPin));
    if (c == '\n' || c == '\r') {
      line[count] = 0;          // terminate string
      flash_hex_line(line);
      count = 0;
    } else
      line[count++] = c;        // add to string*

  }
}


// Waiting for Can
//***************************
void Flasher::waiting_for_can(){

  _flashing_state = true;

  //byte c;
  char line[45];
  uint8_t count = 0;


  //Serial_Printf( "entree dans les limbes du can\n");
  //setRGB(255,0,255);

  //On eteind toutes les LEDS
  /*digitalWrite(ledPin, 0);
  for (uint8_t i=0;i<61;i++){
    pwm.setPWM(i,0);
  }*/

  for (;;)  {

    while (p_CANbus->read(msg) ){

      for (uint8_t i=0;i<8;i++){

        byte c = msg.buf[i];
        //Serial_Printf( "%c",(char) c );

        if (global_line_i % 10 == 1) digitalWrite(ledPin, !digitalRead(ledPin));

        if (c == '\n' || c == '\r') {
          line[count] = 0;          // terminate string
          flash_hex_line(line);
          count = 0;
          //yield();
        } else
          line[count++] = c;        // add to string

      }//for loop
    }//while loop
  }//for
}




// check that the uploaded firmware contains a string that indicates that it will run on this MCU
int Flasher::check_compatible(uint32_t min, uint32_t max)
{
  uint32_t i;

  // look for FLASH_ID in the new firmware
  for (i = min; i < max - strlen(FLASH_ID); ++i) {
    if (strncmp((char *)i, FLASH_ID, strlen(FLASH_ID)) == 0)
      return 1;
  }
  return 0;
} // check_compatible()

// *****************************************************************************

// WARNING:  you can destroy your MCU with flash erase or write!
// This code may or may not protect you from that.
// 2nd Modifications for teensy 3.5/3/6 by Deb Hollenback at GiftCoder
//    This code is released into the public domain.
// Extensive modifications for OTA updates by Jon Zeeff
// Original by Niels A. Moseley, 2015.
// This code is released into the public domain.

// https://namoseley.wordpress.com/2015/02/04/freescale-kinetis-mk20dx-series-flash-erasing/


static int leave_interrupts_disabled = 0;


// *********************************
// actual flash operation occurs here - must run from ram
// flash a 4 byte word

RAMFUNC int Flasher::flash_word (uint32_t address, uint32_t word_value)
{
  //Serial_Printf("befor: %X\n", *(volatile uint32_t *) address);
  //Serial_Printf("writing: %X\n", word_value);
  if (address >= FLASH_SIZE || (address & 0B11) != 0) // basic checks
    return 1;

  // correct value in FTFL_FSEC no matter what
  if (address == 0x40C) {
    word_value = 0xFFFFFFFE;
  }

  // check if already done - not an error
  if (*(volatile uint32_t *) address == word_value)
    return 0;

  // check if not erased
  if (*(volatile uint32_t *) address != 0xFFFFFFFF)  // TODO this fails
    return 4;

  __disable_irq ();

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)  // wait for ready
  {
  };

  // clear error flags
  FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0;

  // program long word!
  FTFL_FCCOB0 = 0x06;    // PGM
  FTFL_FCCOB1 = address >> 16;
  FTFL_FCCOB2 = address >> 8;
  FTFL_FCCOB3 = address;
  FTFL_FCCOB4 = word_value >> 24;
  FTFL_FCCOB5 = word_value >> 16;
  FTFL_FCCOB6 = word_value >> 8;
  FTFL_FCCOB7 = word_value;

  FTFL_FSTAT = FTFL_FSTAT_CCIF;  // execute!

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)  // wait for ready
  {
  };

  FMC_PFB0CR |= 0xF << 20;  // flush cache

  if (!leave_interrupts_disabled)
    __enable_irq ();

  //Serial_Printf("after: %X\n", *(volatile uint32_t *) address);
  // check if done OK
  if (*(volatile uint32_t *) address != word_value) {
    //Serial_Printf("should be: %X but is: %X\n", word_value, (volatile uint32_t *) address);
    //Serial_Printf("FTFL_FSTAT: %X\n", FTFL_FSTAT);
    return 8;
  }

  return FTFL_FSTAT & (FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0);
}


int erase_count = 0;

// *************************************

RAMFUNC int Flasher::flash_erase_sector (uint32_t address, int unsafe)
{
  if (address > FLASH_SIZE || (address & (FLASH_SECTOR_SIZE - 1)) != 0) // basic checks
    return 1;

  if (address == (0x40C & ~(FLASH_SECTOR_SIZE - 1)) && unsafe != 54321)    // 0x40C is dangerous, don't erase it without override
    return 2;

  __disable_irq ();

  // wait for flash to be ready!
  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)
  {
  };

  // clear error flags
  FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0;

  // erase sector
  FTFL_FCCOB0 = 0x09;
  FTFL_FCCOB1 = address >> 16;
  FTFL_FCCOB2 = address >> 8;
  FTFL_FCCOB3 = address;

  FTFL_FSTAT = FTFL_FSTAT_CCIF;  // execute!

  while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) != FTFL_FSTAT_CCIF)  // wait for ready
  {
  };

  FMC_PFB0CR = 0xF << 20;  // flush cache

  if (!leave_interrupts_disabled)
    __enable_irq ();

  return FTFL_FSTAT & (FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL | FTFL_FSTAT_MGSTAT0);

}  // flash_erase_sector()

// ***********************************************
// move upper half down to lower half
// DANGER: if this is interrupted, the teensy could be permanently destroyed

RAMFUNC void Flasher::flash_move (uint32_t min_address, uint32_t max_address)
{
  leave_interrupts_disabled = 1;

  min_address &= ~(FLASH_SECTOR_SIZE - 1);      // round down

  uint32_t address;
  int error = 0;

  // below here is critical

  // copy upper to lower, always erasing as we go up
  for (address = min_address; address <= max_address; address += 4) {

    if ((address & (FLASH_SECTOR_SIZE - 1)) == 0) {   // new sector?
      error |= flash_erase_sector(address, 54321);

      if (address == (0x40C & ~(FLASH_SECTOR_SIZE - 1)))  // critical sector
        error |= flash_word(0x40C, 0xFFFFFFFE);                  // fix it immediately
    }

    error |= flash_word(address, *(uint32_t *)(address + FLASH_SIZE / 2));

  } // for

  // hint - can use LED here for debugging.  Or disable erase and load the same program as is running.

  if (error) {
    //digitalWrite(ledPin, HIGH);    // set the LED on and halt
    for (;;) {}
  }

  // restart to run new program
#define CPU_RESTART_ADDR ((uint32_t *)0xE000ED0C)
#define CPU_RESTART_VAL 0x5FA0004
  *CPU_RESTART_ADDR = CPU_RESTART_VAL;

}  // flash_move()


// ***********************************

// Given an Intel hex format string, write it to upper flash memory (normal location + 128K)
// When finished, move it to lower flash
//
// Note:  hex records must be 32 bit word aligned!
// TODO: use a CRC value instead of line count

int Flasher::flash_hex_line (const char *line)
{
  // hex records info
  unsigned int byte_count;
  static uint32_t address;
  unsigned int code;
  char data[128];   // assume no hex line will have more than this.  Alignment?

  static uint32_t base_address = 0;
  static int line_count = 0;
  static int error = 0;
  static int done = 0;
  static uint32_t max_address = 0;
  static uint32_t min_address = ~0;
  static char name[20];

  if (line[0] != ':'){	// a non hex line is ignored
    return 0;
  }

  if (error) {      // a single error and nothing more happens
    return -1;
  }

  // check for final flash execute command
  int lines;
  if (sscanf(line, ":flash %d", &lines) == 1) {
    if (lines == line_count && done) {
      //Serial.println(name);

      // switch off the leds
      for (int i=0;i<65;i++){
        pwm.setPWM(i,0);
      }

      global_total_lines=lines;
      Serial_Printf("flash %x:%x begins with name:%s\n", min_address, max_address, name);
      delay(100);
      flash_move (min_address, max_address);
      // should never get here
    } else {
      Serial_Printf ("bad line count %d vs %d or not done\n", lines, line_count);
      return -7;
    }
  } // if

  global_line_i=line_count;
  ++line_count;

  //remplacer par variable plus tard
  pwm.setPWM(global_line_i*26/2300,500);
  pwm.setPWM(32+global_line_i*26/2300,500);


  //int parse_hex_line(const char *theline, char bytes, unsigned int *addr, unsigned int *num, unsigned int *code);

  // must be a hex data line
  if (! flasher.parse_hex_line((const char *)line, (char *)data, (unsigned int *) &address, (unsigned int*) &byte_count, (unsigned int*) &code))
  {
    Serial_Printf ("bad hex line %s\n", line);
    error = 1;
    return -1;
  }

  // address sanity check
  if (base_address + address + byte_count > FLASH_SIZE / 2 - RESERVE_FLASH) {
    Serial_Printf("address too large\n");
    error = 2;
    return -4;
  }

  // process line
  switch (code)
  {
    case 0:             // normal data
      break;
    case 1:             // EOF
      delay(1000);
      Serial_Printf ("done, %d hex lines, address range %x:%x, waiting for :flash %d\n", line_count, min_address, max_address, line_count);
      //if (check_compatible(min_address + FLASH_SIZE / 2, max_address + FLASH_SIZE / 2)) //avant
      if (check_compatible (min_address, max_address + FLASH_SIZE / 2))
        done = 1;
      else
        Serial_Printf ("new firmware not compatible\n");
      return 0;
    case 2:
      base_address = ((data[0] << 8) | data[1]) << 4;   // extended segment address
      return 0;
    case 4:
      base_address = ((data[0] << 8) | data[1]) << 16;  // extended linear address
      return 0;
    default:
      Serial_Printf ("err code = %d, line = %s\n", code, line);
      error = 3;
      return -3;
  }				// switch

  // write hex line to upper flash - note, cast assumes little endian and alignment
  if (flash_block (base_address + address + (FLASH_SIZE / 2), (uint32_t *)data, byte_count))  // offset to upper 128K
  {
    Serial_Printf ("can't flash %d bytes to %x\n", byte_count, address);
    error = 4;
    return -2;
  }

  // track size of modifications
  if (base_address + address + byte_count > max_address)
    max_address = base_address + address + byte_count;
  if (base_address + address < min_address)
    min_address = base_address + address;

  return 0;
}				// flash_hex_line()

// ****************************
// check if sector is all 0xFF
int Flasher::flash_sector_erased(uint32_t address)
{
  uint32_t *ptr;

  for (ptr = (uint32_t *)address; ptr < (uint32_t *)(address + FLASH_SECTOR_SIZE); ++ptr) {
    if (*ptr != 0xFFFFFFFF)
      return 0;
  }
  return 1;
} // flash_sector_erased()

// ***************************
// erase the entire upper half
// Note: highest sectors of flash are used for other things - don't erase them
void Flasher::flash_erase_upper()
{
  uint32_t address;
  int ret;

  // erase each block
  for (address = FLASH_SIZE / 2; address < (FLASH_SIZE - RESERVE_FLASH); address += FLASH_SECTOR_SIZE) {
    if (!flash_sector_erased(address)) {
      //Serial.printf("erase sector %x\n", address);
      if ((ret = flash_erase_sector(address, 0)) != 0)
        Serial_Printf("flash erase error %d\n", ret);
    }
  } // for
} // flash_erase_upper()


// **************************
// take a word aligned array of words and write it to upper memory flash
int Flasher::flash_block (uint32_t address, uint32_t * bytes, int count)
{
  int ret;

  if ((address % 4) != 0 || (count % 4 != 0))  // sanity checks
  {
    Serial_Printf ("flash_block align error\n");
    return -1;
  }

  while (count > 0)
  {
    if ((ret = flash_word(address, *bytes)) != 0)
    {
      Serial_Printf ("flash_block write error %d\n", ret);
      return -2;
    }
    address += 4;
    ++bytes;
    count -= 4;
  }				// while

  return 0;
}				// flash_block()





// **********************************************************

/* Intel Hex records:

  Start code, one character, an ASCII colon ':'.
  Byte count, two hex digits, indicating the number of bytes (hex digit pairs) in the data field.
  Address, four hex digits
  Record type (see record types below), two hex digits, 00 to 05, defining the meaning of the data field.
  Data, a sequence of n bytes of data, represented by 2n hex digits.
  Checksum, two hex digits, a computed value that can be used to verify the record has no errors.

  Example:
  :109D3000711F0000AD38000005390000F546000035
  :049D400001480000D6
  :00000001FF

*/


/* Intel HEX read/write functions, Paul Stoffregen, paul@ece.orst.edu */
/* This code is in the public domain.  Please retain my name and */
/* email address in distributed copies, and let me know about any bugs */

/* I, Paul Stoffregen, give no warranty, expressed or implied for */
/* this software and/or documentation provided, including, without */
/* limitation, warranty of merchantability and fitness for a */
/* particular purpose. */

// type modifications by Jon Zeeff


/* parses a line of intel hex code, stores the data in bytes[] */
/* and the beginning address in addr, and returns a 1 if the */
/* line was valid, or a 0 if an error occured.  The variable */
/* num gets the number of bytes that were stored into bytes[] */

int Flasher::parse_hex_line (const char *theline, char *bytes, unsigned int *addr, unsigned int *num, unsigned int *code)
{
  unsigned sum, len, cksum;
  const char *ptr;
  int temp;

  *num = 0;
  if (theline[0] != ':')
    return 0;
  if (strlen (theline) < 11)
    return 0;
  ptr = theline + 1;
  if (!sscanf (ptr, "%02x", &len))
    return 0;
  ptr += 2;
  if (strlen (theline) < (11 + (len * 2)))
    return 0;
  if (!sscanf (ptr, "%04x", (unsigned int *)addr))
    return 0;
  ptr += 4;
  //Serial_Printf("Line: length=%d Addr=%d\n", len, *addr);
  if (!sscanf (ptr, "%02x", code))
    return 0;
  ptr += 2;
  sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + (*code & 255);
  while (*num != len)
  {
    if (!sscanf (ptr, "%02x", &temp))
      return 0;
    bytes[*num] = temp;
    ptr += 2;
    sum += bytes[*num] & 255;
    (*num)++;
    if (*num >= 256)
      return 0;
  }
  if (!sscanf (ptr, "%02x", &cksum))
    return 0;

  if (((sum & 255) + (cksum & 255)) & 255)
    return 0;			/* checksum error */
  return 1;
}





// Compare the line with a TRIG signal
//************************************
uint8_t Flasher::try_trig(const char *line){
  if(strcmp("TRIG", line) == 0){
    Serial_Printf("TRIG -> ok\n");
    return 1;
  } else
    return 0;
}



// Compare the line with a MX signal
//************************************
bool Flasher::is_mx(const char *line){
  if (line[0] == '/'){
    if (line[1] == 'M'){
      Serial_Printf("M%c.hex firmware detected...\n",(char) line[2]);
      return true;
    }
    return false;
  }
  return false;
}

// Compare the line with a SX signal
//************************************
bool Flasher::is_sx(const char *line){
  if (line[0] == '/'){
    if (line[1] == 'S'){
      Serial_Printf("S%c.hex firmware detected...\n",(char) line[2]);
      return true;
    }
    return false;
  }
  return false;
}

// Compare the line with a SX signal
//************************************
int8_t Flasher::which_sx(const char *line){

    if (line[2] == '1'){
      return 1;
    } else if (line[2] == '2'){
      return 2;
    } else if (line[2] == '3'){
      return 3;
    } else if (line[2] == '4'){
      return 4;
    } else if (line[2] == 'X'){
      return 88; //88=X char to int
    } else {
      // no slave detected
      return 0;
    }
    return 0;
}


void Flasher::set_flashing_state(bool *pflashing_state, bool state)
{
  *pflashing_state = state;
}


// if Receiving THE signal
// stop every communications
// Launch the upgrade_firmware
//****************************
void Flasher::try_my_triggers(byte c){

  // ONLY if the platform isn't flashing  !!
  //****************************************
  if (! _flashing_state ){

    //Serial_Printf("%c",c);
    if (c == '\n' || c == '\r') {
      line[count] = 0;          // terminate string

      if( try_trig(line) ){
        _trig=true;
      }

      if ( is_mx(line) && _trig){
        _trig=false;
        Serial_Printf("MX serial upgrade will begin...\n");
        digitalWrite(ledPin,LOW);
        _flashing_state=true;
        flasher.upgrade_firmware("SERIAL");
      }

      if ( is_sx(line) && _trig){
        _trig=false;
        uint8_t code = which_sx(line);
        Serial_Printf("S%c transfert by can will begin...\n",(char) code);
        _flashing_state=true;
        pgm_going_to_sx(code);
        //! Do not put in a loop //!
        flasher.reboot();
      }

      count = 0;
    } else
      line[count++] = c;  // add to string

  } //currently_flashing
  //return true; //is flashing !
}


// Everything we receive now is going to the right sx
//***************************************************
void Flasher::pgm_going_to_sx(uint8_t platform_code){

  // platform_code is the number of the slave
  //********************************
  if (platform_code){

    uint8_t programming_platform;

    if      (platform_code == 1) programming_platform = 0x2f;
    else if (platform_code == 2) programming_platform = 0x30;
    else if (platform_code == 3) programming_platform = 0x2b;
    else if (platform_code == 4) programming_platform = 0x2c;
    else                         programming_platform = 0x58;

    msg.id  = (((flashing_code & 0xFFF) << 8) | ((programming_platform & 0xFF))) ;
    msg.len = 8;


    // Adding the TRIG signal
    //***********************
    const char * trig = "TRIG...\n";
    for (uint8_t i=0; i<8 ; i++) {
      msg.buf[i] = trig[i];
      delay(50);
    } Can_Send(msg);


    // Switch off the light before blinking... !
    //*****************************************
    digitalWrite(ledPin, 0);


    // Getting the hex program from Serial1
    //*************************************
    long can_sent_count=0;

    // To force repeating the serial during 600ms
    //*******************************************
    elapsedMillis waiting;
    while (waiting < 600) {

      if ( Serial1.available()  ) {

        uint8_t bufindex = 0;
        while ( bufindex < 8 && waiting < 500){
          uint8_t c = Serial_Read();

          if (isAscii(c)){
            msg.buf[bufindex++] = c;
            waiting = 0;
          }
        }

        Can_Send(msg);
        can_sent_count++;
        if (can_sent_count%20) digitalWrite(ledPin, !digitalRead(ledPin));

      } //serial_available
    } //is_com
  } // if code
}




// check the availability
//***********************
void Flasher::is_can(){
  // can non detecté CAN chaque 1s
	if(!p_CANbus->available()) {
		if (blink.check() ==1){
			Serial_Printf("CAN non disponible\n"); //displayBufs();
		}
	} //waiting for Canbus to be connected
}


// Can begin
//************************
void Flasher::Can_Begin(){
  p_CANbus->begin();
}




// Pwm Set RGB (255,255,255)
//**************************
void Flasher::setRGB(uint8_t r,uint8_t g,uint8_t b){
  static uint8_t r255=0,g255=0,b255=0;
  if (r!=r255 || g!=g255 || b!=b255){
    r255=r; g255=g; b255=b;
    pwm.setPWM(PIN_RGB_R,r*16);
    pwm.setPWM(PIN_RGB_G,g*16);
    pwm.setPWM(PIN_RGB_B,b*16);
    pwm.setPWM(PIN_RGB_LUM,4095);
  }
}



// 1 : Display
// 2 : Write on the bus
//*********************
void Flasher::Can_Send(const CAN_message_t &msg){
  Can_Printf(msg);
  p_CANbus->write(msg);
  //yield();
}


// Display the flow without formating
//***********************************
void Flasher::Can_Printf(const CAN_message_t &msg){
  //flashing_code
  if (((msg.id & 0xFFF)>>8) == 0x1) {
    for (uint8_t i=0;i<8;i++){
      Serial_Printf( "%c",msg.buf[i] );
    }
  }
}


// Display the flow without formating
//***********************************
void Flasher::Can_Read(){
  if (p_CANbus->read(msg) ){
    Can_Printf(msg);
  }
}




void Flasher::set_platform_code(uint8_t x){
  _platform_code=x;
}

// timers-counters stops FAIL_CODE:-1
// Serials stops         FAIL_CODE:-2
// I2C stops             FAIL_CODE:-3
// CAN stops             FAIL_CODE:-4
// SPI stops             FAIL_CODE:-5
//            GENERAL SUCCESS_CODE:1
//***********************************
void Flasher::stop_coms(){
  Serial.end();
  Serial1.end();
  p_CANbus->end();
}


void Flasher::start_coms(){
  Serial.begin(1000000);
  Serial1.begin(115200);
  p_CANbus->begin();
  pwm.begin();
}


void Flasher::reboot(){
    // restart to run new program
  #define CPU_RESTART_ADDR ((uint32_t *)0xE000ED0C)
  #define CPU_RESTART_VAL 0x5FA0004
    *CPU_RESTART_ADDR = CPU_RESTART_VAL;
}


void Flasher::set_Can(FlexCAN *can){
  p_CANbus = can;
}



Flasher flasher;
