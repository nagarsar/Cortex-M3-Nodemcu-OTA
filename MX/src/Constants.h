//*******************************************************************
// CONFIGURATION
//*******************************************************************

//pin des élements d'éclairage


//hex
//lighting_type_Front
#define ASSOCIATE_TIMER   0x1e //30 en decimal
#define TIMER_COUNTER			0x28
#define CLIGNOTANT_CHE		0x29
#define CLIGNOTANT_CLI		0x2a
#define POUMONAGE					0x2b
#define DRL_MODE_CODE			0x2c
#define ON_OFF					  0x2d
#define RESTART						0x2e
#define SAVE							0x2f
#define DAY_NIGHT					0x30
#define LANTERNE					0x31
#define CODE							0x46
#define ROUTE							0x47
#define DRL_HB_LB					0x48
#define ANTIBAV						0x49
#define ANTIBAR						0x4a
#define STOP							0x4b
#define RECUL							0x32
#define WARNING_CHE				0x33
#define ID_FIXE 			    0x34
#define BOUNCING					0x36
#define PROPORTIONNEL			0x37
#define HALO							0x38
#define MONOLED						0x39
#define CENTRER						0x3a
#define FONTAINE					0x3b
#define BOUNCE_COLOR			0x3c
#define RVB								0x3d
#define BOULIER						0x50
#define SPIRALE						0x51
#define FILANTE						0x52
#define SIGNATURE					0x53
#define VAGUE							0x54
#define EQUALIZER					0x55
#define OVERLAY						0x56
#define ADRESSAGE_PINS1		0x64
#define ADRESSAGE_PINS2		0x65
#define ADRESSAGE_PINS3		0x66
#define ADRESSAGE_PINS4		0x67
#define ADRESSAGE_PINS5		0x68
#define ADRESSAGE_PINS6		0x69
#define ADRESSAGE_PINS7		0x6a
#define SET_LOW_BEAM_COEF  0x70
#define ID_G  0x80
#define ID_D  0x81
#define FONCTION1  0x82
#define FONCTION2  0x83
#define FONCTION3  0x84
#define FONCTION4  0x85
#define FONCTION5  0x86
#define FONCTION6  0x87
#define FONCTION7  0x88
#define FONCTION8  0x89
#define ZONE1      0x8a
#define ZONE2      0x8b
#define ZONE3      0x8c
#define ZONE4      0x8d

//code zone
#define AVG 0x2f
#define AVD 0x30
#define ARG 0x2b
#define ARD 0x2c
#define TOUS 0x58


//struct trames
typedef struct WIFI_message_t {
  int id;
  uint8_t buf[16];
} Param_t;
