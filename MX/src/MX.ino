#include <Arduino.h>
#include <Metro.h>
#include <FlexCAN.h>
#include <SPI.h>
#include <Flasher.h>
#include <PWM_Driver3.h>
#include "Constants.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+                      DEFINITIONS                     +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MX      1 // choose here BTW [MX]:1 or [SX]:0
#define OTA     1 // to activate the reprogramming tool
#define NODEMCU 1 // to activate the nathan shield_wifi (OTA compatible)
#define DEBUG   1 // to show some debugs in serial console

// IS the nodemcu plugged ?
#if MX
  #if NODEMCU
  #else
    String ReseauWifi = "flexlam";
    String MotDePasse = "flex1234";
    String adresse_IP = "192.168.1.10";
  #endif
#endif


const uint8_t execution_code = 0x0 ; // the concatenate id will be: 0x[0]XX
const uint8_t flashing_code  = 0x1 ; // the concatenate id will be: 0x[1]XX
uint8_t platform_code        = 0xff; // the concatenate id will be: 0xX[XX], XX is given in setup by getX() method


const uint8_t INTENSITE = 50; // % of intensity
uint8_t curseur         = 1;  // cursor is set to 1 because it is the first led in the row of leds

uint8_t ID_state1; // state of ID G (left)
uint8_t ID_state2; // state of ID D (right)

// Collect all data
#define FRAME_NB 23 // the number of lights you have {LANTERNE, ROUTE, ANTIBAV, ...}
#define BUF_NB   8  // the number of bufs you have in a CAN bus is 8


char eclairage[FRAME_NB][BUF_NB]; // this contains every lights with their settings it cost arround 183 octets
long txCount;                     // number of tx sent by MX
uint8_t bufindex = 0;             // an index for serial inputs methods
const uint8_t BUFFER_SIZE = 34;   // the number of characters a wifi request can contain
uint8_t buf[13];                  // the number of bufs a wifi request can contain
char request[BUFFER_SIZE];        // the wifi / serial request



#define Serial_Available()  Serial1.available() // for practical reasons
#define Serial_Read()       Serial1.read()      // for practical reasons
#define Serial_Printf       Serial.printf       // allow to replace Serial.println -> Serial_Printf("\n");

IntervalTimer canTimer;           // Timer counter object /! This can interrupt the main routine /!
IntervalTimer IDTimer;            // Timer counter object /! This can interrupt the main routine /!
Metro t_curseur   = Metro(15);    // frequency period (ms) of directionnals indicators
Metro t_execution = Metro(5);     // frequency period (ms) of MX, sending the can frames | officially it msut be 5ms


const int ledPin = 13;    // pin of the main led
FlexCAN CANbus(1000000);  // FlexRAY speed : 1000000 mo/s
static CAN_message_t msg; // the msg struct can be accessed everywhere in the program
PWM_Driver bloc;          // pwm object of 64 outputs



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+                      PROGRAMM                        +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Setup
// setting everyone !
//*******************
void setup() {
  pinMode(ledPin, OUTPUT); pinMode(A0,INPUT); pinMode(A1,INPUT); pinMode(A2,INPUT); pinMode(A3,INPUT);
  digitalWrite(ledPin, HIGH);
  #if MX
    MX_setup();
    #if NODEMCU
    #else
      setup_wifi8266_AT();
    #endif
  delay(5000);
  #endif
  #if OTA
    flasher.set_Can(&CANbus);
    flasher.start_coms();
    while (millis()<3000){
      platform_code = getX();
      flasher.set_platform_code( platform_code );
    } digitalWrite(ledPin, LOW);
  #endif
}


// Loop
// only the basics...
//*******************
void loop() {
  serial_communication(); // for MX: OTA come from Serial
  can_communication();    // for SX: OTA come from CAN
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+                      METHODS                         +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// SERIAL COMMUNICATION
//*********************
void serial_communication(){

  // [0]
  if ( Serial.available() ){
    byte c = Serial.read();
    if (MX) get_keyboard_frames( c );
  }

  // [1]
  if ( Serial1.available() ){
    byte c = Serial1.read();
    if (MX)  get_wifi_frames( c );
    if (OTA) flasher.try_my_triggers( c );
  }

}




// CAN_COMMUNICATION
//******************
void can_communication(){

  // if Current platform is [MX]
  // [===========] --> we send !
  //****************************
  if( MX ) {
    if ( t_execution.check() ){

      Serial.print( "\n- Send Trame CAN -\n");
      msg.id  = (((execution_code & 0xFFF) << 8) | ((platform_code & 0xFF))) ;
      msg.len = 8;

      for (uint8_t i=0; i<FRAME_NB; i++){
        for (uint8_t j=0; j<BUF_NB; j++){
          msg.buf[j]=eclairage[i][j];
        }
        CANbus.write(msg);
        if (DEBUG) read_can(&msg);
        txCount++;
        if(txCount%(2*FRAME_NB)==1) digitalWrite(ledPin, !digitalRead(ledPin) );
        delayMicroseconds(100);
      }
    }
  }
  // Current platform is [SX]
  // [===========] <-- we receive
  //*****************************
  else {
    if ( CANbus.read(msg) ){
      // this is the :
      // - "flasing_code" mode
      //! After this, there is NO RETURN !
      //! If it is stopped completely you must reboot the platform
      // - test on 0x[1]XX
      if (((msg.id & 0xFFF)>>8) == flashing_code) {
        if (OTA) {
          flasher.upgrade_firmware("CAN");
          //flasher.Can_Printf(msg);
        }
      }
      // this is the :
      // - "execution_code" mode
      // - test on 0x[0]XX
      if (((msg.id & 0xFFF)>>8) == execution_code) {
        read_can(&msg);
        do_can(&msg);
      }
    }
  }

}



// Display the following params
// [id] [len] [0] [1] [2] [3] [4] [5] [6] [7]
// *******************************************
void read_can(CAN_message_t * msg){
  Serial_Printf("0x%x %d ", msg->id, msg->len);
  for (uint8_t i=0;i<BUF_NB;i++){ Serial_Printf("%x ", msg->buf[i]); }
  Serial_Printf("\n");
}





#if MX // ............................ will compile only the MX's methods !

// Construit une trame wifi a
// partir de la com serial1
//***************************
void get_wifi_frames(byte c){

  //Serial_Printf("%c", c);
    if (!NODEMCU){
      if(Serial1.find("GET /") ) {

        delay(1); //1ms : Temps pour reconnecter le serveur
        while ( bufindex < BUFFER_SIZE ){
          request[bufindex++] = c;
          delayMicroseconds(70); // 70 us entre chaque enregistrement sinon caractère lu illisible //10us pour 14 caracts //20us : beug //50us : beug //100us : beug
        }
        parse_command(request);
        bufindex=0;
      }
      if (!NODEMCU) Serial1.println("AT+CIPCLOSE=5");
    }

    else {

      if (c == '\n' || c == '\r') {
        request[bufindex] = 0;          // terminate string
        String req;
        req = String(request);
        req.remove(0,5);                // to remove the [GET /] part
        parse_command( req.c_str() );
        bufindex = 0;
      } else
        request[bufindex++] = c;        // add to string*
    }
}

// Construit une trame keyboard
// a partir de la com serial
//***************************
void get_keyboard_frames(char c){

  if (c == '\n' || c == '\r') {
    request[bufindex] = 0;          // terminate string
    parse_command(request);
    bufindex = 0;
  } else
    request[bufindex++] = c;        // add to string*
}

// parse_command : parse
// une chaine de caractere et
// stocke dans des variables
//***************************
void parse_command(const char *com) {

  sscanf(com,"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",&buf[0],&buf[1],&buf[2],&buf[3],&buf[4],&buf[5],&buf[6],&buf[7],&buf[8],&buf[9],&buf[10],&buf[11]);
  do_mapping();

  #if DEBUG
  Serial.print("parse_command: ");
  for (uint8_t i=0;i<12;i++){
    Serial.print(buf[i]); Serial.print(" ");
    buf[i]=0; //reset the buf
  }
  Serial.println("");
  #endif

}//fin parse_command

// MX Setup to put every addresses
//********************************
void MX_setup(){

  ID_state1=0;
  ID_state2=0;

  //canTimer.begin( can_communication , 5000); //µs
  IDTimer.begin( ID , 500000); //µs

  eclairage[0][0]=ON_OFF;
  eclairage[1][0]=LANTERNE;
  eclairage[2][0]=CODE;
  eclairage[3][0]=ROUTE;
  eclairage[4][0]=ANTIBAV;
  eclairage[5][0]=ANTIBAR;
  eclairage[6][0]=STOP;
  eclairage[7][0]=RECUL;
  eclairage[8][0]=ID_G;
  eclairage[9][0]=WARNING_CHE;
  eclairage[10][0]=ID_D;
  eclairage[11][0]=FONCTION1;
  eclairage[12][0]=FONCTION2;
  eclairage[13][0]=FONCTION3;
  eclairage[14][0]=FONCTION4;
  eclairage[15][0]=FONCTION5;
  eclairage[16][0]=FONCTION6;
  eclairage[17][0]=FONCTION7;
  eclairage[18][0]=FONCTION8;
  eclairage[19][0]=ZONE1;
  eclairage[20][0]=ZONE2;
  eclairage[21][0]=ZONE3;
  eclairage[22][0]=ZONE4;
}

// Do_mapping
// Contains all the  rules
//                                              |collect DATA in eclairage[][]
//                                        ROLES |Start some routines :ID, RGB...
//                                              |Save states
//                                              |
//             +---------+   WIFI sent bufs    +--+          eclairage[][]     +--+
//   PUSH ---> |ID BUTTON| +-----------------> |MX| +------------------------> |SX|
//             +---------+                     +--+                            +--+
//                                              |
//                                              |eclairage[8][i]=buf[i]
//                                      ACTIONS |IDTimer ON
//                                              |ID_state=0
//
//*********************************************************************************
void do_mapping(){

  switch(buf[0]){

    case ON_OFF      : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[0][i]=buf[i]; }
                       if (buf[1]==0 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1]=0; eclairage[11][1]=0; eclairage[12][1]=0;
                                        eclairage[13][1]=0; eclairage[14][1]=0; eclairage[15][1]=0;
                                        eclairage[16][1]=0; eclairage[17][1]=0;
                                        ID_state1=0; ID_state2=0; }                     break;
    case LANTERNE    : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[1][i]=buf[i]; }      break;
    case CODE        : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[2][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[1][1] =1; eclairage[3][1] =0;  }      break;
    case ROUTE       : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[3][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[1][1] =1; eclairage[2][1] =1;  }      break;
    case ANTIBAV     : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[4][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[1][1] =1; eclairage[2][1] =1;  }      break;
    case ANTIBAR     : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[5][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[1][1] =1; eclairage[2][1] =1;  }      break;
    case STOP        : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[6][i]=buf[i]; }      break;
    case RECUL       : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[7][i]=buf[i]; }      break;
    case ID_G        : IDTimer.begin( ID , 500000); //µs
                       for (uint8_t i=0; i<BUF_NB; i++){ eclairage[8][i]=buf[i]; }
                       if (buf[1]==1 && ID_state1==0){ ID_state1=1; ID_state2=0; eclairage[10][1] =0;}
                       if (buf[1]==0 && ID_state1==1){ ID_state1=0; }                   break;
    case ID_FIXE     : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[9][i]=buf[i];  }     break;
    case ID_D        : IDTimer.begin( ID , 500000); //µs
                       for (uint8_t i=0; i<BUF_NB; i++){ eclairage[10][i]=buf[i]; eclairage[8][1] =0;}
                       if (buf[1]==1 && ID_state2==0){ ID_state2=1; ID_state1=0;}
                       if (buf[1]==0 && ID_state2==1){ ID_state2=0; }                   break;
    case FONCTION1   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[11][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[12][1] =0; eclairage[13][1] =0; eclairage[14][1] =0;
                                        eclairage[15][1] =0; eclairage[16][1] =0; eclairage[17][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION2   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[12][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[13][1] =0; eclairage[14][1] =0;
                                        eclairage[15][1] =0; eclairage[16][1] =0; eclairage[17][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION3   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[13][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[12][1] =0; eclairage[14][1] =0;
                                        eclairage[15][1] =0; eclairage[16][1] =0; eclairage[17][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION4   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[14][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[12][1] =0; eclairage[13][1] =0;
                                        eclairage[15][1] =0; eclairage[16][1] =0; eclairage[17][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION5   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[15][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[12][1] =0; eclairage[13][1] =0;
                                        eclairage[14][1] =0; eclairage[16][1] =0; eclairage[17][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION6   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[16][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[12][1] =0; eclairage[13][1] =0;
                                        eclairage[14][1] =0; eclairage[15][1] =0; eclairage[17][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION7   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[17][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[12][1] =0; eclairage[13][1] =0;
                                        eclairage[14][1] =0; eclairage[15][1] =0; eclairage[16][1] =0;
                                        eclairage[18][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case FONCTION8   : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[18][i]=buf[i]; }
                       if (buf[1]==1 ){ eclairage[11][1] =0; eclairage[12][1] =0; eclairage[13][1] =0;
                                        eclairage[14][1] =0; eclairage[15][1] =0; eclairage[16][1] =0;
                                        eclairage[17][1] =0; }
                       if (buf[1]==1 ){ eclairage[1][1] =0; eclairage[2][1] =0; eclairage[3][1] =0;
                                        eclairage[4][1] =0; eclairage[5][1] =0; eclairage[6][1] =0;
                                        eclairage[7][1] =0; eclairage[8][1] =0; eclairage[9][1] =0;
                                        eclairage[10][1] =0; ID_state1=0; ID_state2=0; } break;
    case ZONE1       : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[19][i]=buf[i]; }      break;
    case ZONE2       : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[20][i]=buf[i]; }      break;
    case ZONE3       : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[21][i]=buf[i]; }      break;
    case ZONE4       : for (uint8_t i=0; i<BUF_NB; i++){ eclairage[22][i]=buf[i]; }      break;
  }
}

// this is a switch
// it couldn't be both ID
// in the same time
//***********************
void ID(){
  if (ID_state1){ eclairage[8][1]  = !eclairage[8][1];  }
  if (ID_state2){ eclairage[10][1] = !eclairage[10][1]; }
}


  #if NODEMCU // ......... will compile only the Not NODEMCU methods !
  #else
  // REPONSE repond sur le terminal serie
  //*************************************
  void REPONSE()
  {
    String reponse = "";
    long int time = millis();
    while( (time+500) > millis()) {
      while(Serial1.available()) {
        char c = Serial1.read();
        reponse+=c;
      }
    }
    Serial.print(reponse);
  }

  // Permet de configurer la puce 8266 en most hotspot serveur
  //**********************************************************
  void setup_wifi8266_AT(){
    Serial1.println("AT+RST"); REPONSE();
    Serial1.println("AT+CIPAP=\""+ adresse_IP + "\""); REPONSE();
    Serial1.println("AT+CWSAP=\""+ ReseauWifi + "\",\"" + MotDePasse +"\",10,3"); REPONSE();
    Serial1.println("AT+CWMODE=2"); REPONSE();
    Serial1.println("AT+CIPMUX=1"); REPONSE();
    Serial1.println("AT+CIPSERVER=1,80"); REPONSE();
  }
  #endif


#else // ...................... will compile only the SX's methods !


// Clignotant défilant sur un interval de leds donné  [min - max]
// Sur cette version, la cadence est fixée statique a 15ms
// grace au timer "t_curseur"
// Les variable prev_ gardent en mémoire l'etat précedent
//**************************************************************
void do_ID(uint8_t id, uint8_t buf1, uint8_t min, uint8_t max){
  static uint8_t prev_buf[255];

  if (prev_buf[id] != buf1){
    prev_buf[id] = buf1;
    if (buf1==1){
      t_curseur.reset();
      curseur=1;
    } else {
      for (uint8_t j=min; j<=max ; j++){
        bloc.setPWM(j, 0 );
      }
    }
  }

  if (buf1==1){
    for (uint8_t j=min ; j<=curseur+min-1 ; j++){
      bloc.setPWM(j, INTENSITE * 4094 / 100 );
    }
  }

}//fin do_ID

// Defini l'etat de plusieurs leds sur un intervall [min - max]
// l'etat est lu depuis buf1
//*************************************************************
void do_leds(uint8_t id, uint8_t buf1, uint8_t min, uint8_t max){
  static uint8_t prev_buf[255];

  if (prev_buf[id] != buf1){
    prev_buf[id] = buf1;

    if (buf1==1){
      for (uint8_t j=min ; j<=max ; j++){
        bloc.setPWM(j   , INTENSITE * 4094 / 100 );
        bloc.setPWM(j+32, INTENSITE * 4094 / 100 );
      }
    } else {
      for (uint8_t j=min ; j<=max ; j++){
        bloc.setPWM(j   , 0 );
        bloc.setPWM(j+32, 0 );
      }
    }
  }
}//fin do_leds

// Do CAN
// here are, the choices we do
// in terms of lights comportments
//********************************
void do_can(CAN_message_t * msg){
  static long rxCount=0;
  rxCount++;

  if (rxCount%(2*FRAME_NB)==1) digitalWrite(ledPin, !digitalRead(ledPin) );
  switch(msg->buf[0]){
    case ID_G     :  do_ID  ( ID_G,     msg->buf[1],  1, 20);  break; // [EXAMPLE] : from led 1 to 20
    case ID_D     :  do_ID  ( ID_D,     msg->buf[1], 33, 53);  break;
    case STOP     :  do_leds( STOP,     msg->buf[1], 21, 25);  break;
    case RECUL    :  do_leds( RECUL,    msg->buf[1], 26, 27);  break;
    case ANTIBAR  :  do_leds( ANTIBAR,  msg->buf[1], 28, 28);  break;
    case LANTERNE :  do_leds( LANTERNE, msg->buf[1], 29, 32);  break;
  }
}

#endif //.......... then it's not depending to a proper platfrom anymore




// Return slave adress according to the shunt logic
// avg= 0x2f            //avd= 0x30
// arg= 0x2b            //ard= 0x2c
// tous= 0x58 (int)88
//*************************************************
uint8_t getX() {
  if      (analogRead(A0)<2) return 0x2f;
  else if (analogRead(A1)<2) return 0x30;
  else if (analogRead(A2)<2) return 0x2b;
  else if (analogRead(A3)<2) return 0x2c;
  else                       return 0x58;
}



// DEBUG
//see the INPUTS levels
// A0 ............................]
// A1 .........................................]
// A2 ...................]
// A3 .........................]
//******
void debug(){
  Serial.print ( "A0:" ); Serial.print ( analogRead(A0) );
  Serial.print ( " A1:" );Serial.print ( analogRead(A1) );
  Serial.print ( " A2:" );Serial.print ( analogRead(A2) );
  Serial.print ( " A3:" );Serial.print ( analogRead(A3) );
}
