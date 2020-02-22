#ifndef Flasher_h_
#define Flasher_h_

#include <stdint.h>
#include <kinetis.h>
#include <FlexCAN.h>
#include <Metro.h>
#include <PWM_Driver3.h>
//#include "serial.h"

#if defined(__MK20DX128__)
#define FLASH_SIZE              0x20000
#define FLASH_SECTOR_SIZE       0x400
#define FLASH_ID                "fw_teensy3.0"
#define RESERVE_FLASH (2 * FLASH_SECTOR_SIZE)
#elif defined(__MK20DX256__)
#define FLASH_SIZE              0x40000
#define FLASH_SECTOR_SIZE       0x800
#define FLASH_ID                "fw_teensy3.1"
#define RESERVE_FLASH (1 * FLASH_SECTOR_SIZE)
#else
#error NOT SUPPORTED
#endif

// apparently better - thanks to Frank Boesing
#define RAMFUNC  __attribute__ ((section(".fastrun"), noinline, noclone, optimize("Os") ))



class Flasher
{
public:
  void upgrade_firmware(const char *type);
  const int ledPin = 13;
  uint16_t global_total_lines=0;
  uint16_t global_line_i =0;

  void waiting_for_serial(void);
  void waiting_for_can(void);

  uint8_t try_trig(const char *line);
  bool is_mx(const char *line);
  bool is_sx(const char *line);
  void set_flashing_state(bool *pflashing_state, bool state);
  int8_t which_sx(const char *line);
  void is_can(void);
  void try_my_triggers(byte c);
  void pgm_going_to_sx(uint8_t code);
  void Can_Printf(const CAN_message_t &msg);
  void Can_Send(const CAN_message_t &msg);
  void Can_Read(void);
  void Can_Begin();
  void send_pgm(byte c);
  void set_Can(FlexCAN *can);
  void setRGB(uint8_t r,uint8_t g,uint8_t b);
  void stop_coms();
  void start_coms();
  void set_platform_code(uint8_t x);
  void reboot();

private:
  FlexCAN * p_CANbus;
  uint8_t _platform_code;
  int _number_of_lines=0;
  bool _trig=false;
  bool _flashing_state = false;
  int check_compatible(uint32_t min, uint32_t max);
  uint32_t  read_once(unsigned char address);
  void program_once(unsigned char address, uint32_t value);
  void flash_erase_upper();
  int flash_sector_erased(uint32_t address);
  RAMFUNC static int flash_word (uint32_t address, uint32_t word_value);
  RAMFUNC static int flash_erase_sector (uint32_t address, int unsafe);
  RAMFUNC static void flash_move (uint32_t min_address, uint32_t max_address);
  int flash_hex_line(const char *line);
  int parse_hex_line (const char *theline, char *bytes, unsigned int *addr, unsigned int *num, unsigned int *code);
  int flash_block(uint32_t address, uint32_t *bytes, int count);

};

extern Flasher flasher;


#endif
