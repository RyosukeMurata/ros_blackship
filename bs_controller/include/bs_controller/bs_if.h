/************************************************************************************
Blackship RT-Component
Copyright (c) 2010, Segway-Japan, Ltd.
All rights reserved.

Contact us if you use this software for sell.
If you use this software not for sell, you can use this software under BSD lisence.
See the files LICENSE.TXT and LICENSE-BSD.TXT for more details.
************************************************************************************/
#ifndef __BS_IF_H__
#define __BS_IF_H__


#include <string.h>
#include <iostream>
#include "bs_config.h"

#include <stdio.h>
#include <errno.h>
#include "serial.h"

class CBlackshipIF {
public:
    CBlackshipIF();
    ~CBlackshipIF();

    bool blackship_open(const char* serial_port);
    bool blackship_close();

    bool blackship_set_speed(int ls, int rs);
    bool blackship_set_timeout(int tm);

private:
    CSerial serial;
    unsigned short prevr, prevl;
    int curr, curl;
    int cnt_setspeed;
};

#endif

