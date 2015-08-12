/************************************************************************************
Blackship RT-Component
Copyright (c) 2010, Segway-Japan, Ltd.
All rights reserved.

Contact us if you use this software for sell.
If you use this software not for sell, you can use this software under BSD lisence.
See the files LICENSE.TXT and LICENSE-BSD.TXT for more details.
************************************************************************************/


#include "ros_blackship/bs_if.h"

CBlackshipIF::CBlackshipIF() {
    prevr = prevl = 0;
    curr = curl = 0;
    cnt_setspeed = 0;
}

CBlackshipIF::~CBlackshipIF() {

}

bool CBlackshipIF::blackship_open(const char* serial_port) {
    char errmsg[255 + 1];
    std::cout << "#Opening Blackship I/F " << serial_port << std::endl;
    if (!serial.InitSerial((char*)serial_port, BLACKSHIP_BAUDRATE)) {
        //strerror_s(errmsg, 255, errno);
        strerror_r(errno, errmsg, 255);
        std::cout << "open(2) failed on " << serial_port << ": " << errmsg << std::endl;
        return false;
    }
    std::cout << "#Opening Blackship I/F done" << std::endl;
    return true;
}

bool CBlackshipIF::blackship_close() {
    char errmsg[255 + 1];

    std::cout << "#Closing connection." << std::endl;
    if (!serial.CloseSerial()) {
        strerror_r(errno, errmsg, 255);
        //strerror_s(errmsg, 255, errno);
        std::cout << "close(2) failed : " << errmsg << std::endl;
        return false;
    }
    std::cout << "#Closing connection done" << std::endl;
    return true;
}

bool CBlackshipIF::blackship_set_speed(int ls, int rs) {
    unsigned char smsgr[BLACKSHIP_MAX_DATA_LEN];
    unsigned char smsgl[BLACKSHIP_MAX_DATA_LEN];

    cnt_setspeed++;

    smsgr[0] = BLACKSHIP_STX;
    smsgr[1] = BLACKSHIP_CPUID_MOTOR_RIGHT;
    smsgr[2] = 0x07;
    smsgr[3] = 0x00;
    smsgr[4] = (unsigned char)rs;
    smsgr[5] = 0x00;
    smsgr[6] = BLACKSHIP_ETX;

    smsgl[0] = BLACKSHIP_STX;
    smsgl[1] = BLACKSHIP_CPUID_MOTOR_LEFT;
    smsgl[2] = 0x07;
    smsgl[3] = 0x00;
    smsgl[4] = (unsigned char)ls;
    smsgl[5] = 0x00;
    smsgl[6] = BLACKSHIP_ETX;

    if (cnt_setspeed % 2) {
        return serial.Write2(smsgl, 7) && serial.Write2(smsgr, 7);
    } else {
        return serial.Write2(smsgr, 7) && serial.Write2(smsgl, 7);
    }
}


bool CBlackshipIF::blackship_set_timeout(int tm) {
    unsigned char smsg[BLACKSHIP_MAX_DATA_LEN];

    smsg[0] = BLACKSHIP_STX;
    smsg[1] = BLACKSHIP_CPUID_MOTOR_LEFT;
    smsg[2] = 0x07;
    smsg[3] = 0x07;
    smsg[4] = (unsigned char)tm;
    smsg[5] = 0x00;
    smsg[6] = BLACKSHIP_ETX;
    if (!serial.Write2(smsg, 7)) {
        return false;
    }

    smsg[1] = BLACKSHIP_CPUID_MOTOR_RIGHT;
    if (!serial.Write2(smsg, 7)) {
        return false;
    }
    return true;
}

