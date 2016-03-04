
#ifndef BS_CONFIG_H
#define BS_CONFIG_H

#define BLACKSHIP_BAUDRATE B19200//   B19200

#define BLACKSHIP_STX               ((unsigned char)0x02)
#define BLACKSHIP_ETX               ((unsigned char)0x03)

#define BLACKSHIP_CPUID_MOTOR_RIGHT ((unsigned char)0x01)
#define BLACKSHIP_CPUID_MOTOR_LEFT  ((unsigned char)0x02)
#define BLACKSHIP_CPUID_MASTER      ((unsigned char)0x99)

#define BLACKSHIP_MODE_MOTOR_TIMEOUT    ((unsigned char)0x08)
#define BLACKSHIP_MODE_MOTOR_STATUS     ((unsigned char)0x09)
#define BLACKSHIP_MODE_MOTOR_REQUEST    ((unsigned char)0xF9)

#define BLACKSHIP_MODE_MOTOR_SETSPEED   ((unsigned char)0x00)
#define BLACKSHIP_MODE_MOTOR_SETTIMEOUT ((unsigned char)0x07)

#define BLACKSHIP_MAX_MSG_LEN       ((unsigned char)1024)
#define BLACKSHIP_HEADER_LEN        ((unsigned char)3)
#define BLACKSHIP_FOOTER_LEN        ((unsigned char)1)
#define BLACKSHIP_MAX_DATA_LEN      ((unsigned char)(BLACKSHIP_MAX_MSG_LEN - (BLACKSHIP_HEADER_LEN + BLACKSHIP_FOOTER_LEN)))


namespace bs {

static const bool USE_SLIP = false;
static const double K_SPEED = 1.0;
static const double K_TURN = 1.6;
static const double WHEELWIDTH = 0.405;
static const double WHEELRADIUS_L = 0.1535;
static const double WHEELRADIUS_R = 0.1535;
static const double WHEELCOUNT_1ROT = 2294.6;
static const char* STR_PORT = "/dev/ttyUSB0";//"/dev/serial/by-path/pci-0000:03:00.0-usb-0:2:1.0-port0"; //"\\\\.\\COM25" //"COM2"  //"/dev/ttyUSB0"
static const double K_ODOXY = 1.0;
static const double K_ODOTHETA = 1.0;
static const double K_SLIP = 0.0;
static const double MPS2BS = 152.0;
static const double ODO_OUTPUT = 0;
static const double SIMULATOR = 0;
static const double ID_IIS = 250;
static const double STARTX_IIS = 0.0;
static const double STARTY_IIS = 0.0;
static const double STARTTHETA_IIS = 0.0;
static const double MAX_V = 0.2;
static const double MAX_W = 0.3;

}

#endif
