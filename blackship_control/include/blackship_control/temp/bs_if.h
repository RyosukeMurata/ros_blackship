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

#pragma comment (lib, "winmm.lib") 


class bs_status
{
 public:

  //status
  int int_left_wheel;
  int int_right_wheel;

  float battery_voltage;
  float ui_battery_voltage;

  //for internal use
  int int_left_prev;
  int int_right_prev;


  //odometory

  double wheelwidth;
  double wheelradiusL;
  double wheelradiusR;
  double count1rot;
  
  bool odofirst;

  double odox;
  double odoy;
  double odotheta;

  double addx;
  double addy;
  double addtheta;

  double odotime;
  double odotime_prev;
  //coil::TimeValue odotime;
  //coil::TimeValue odotime_prev;

  bs_status()
  {
    memset(this, 0, sizeof(bs_status));
    odofirst = true;
  }
};



bool blackship_open(const char* serial_port);

int blackship_close();

void blackship_get_encoder(int& le, int& re);
double blackship_get_encoder2(int& le, int& re);

int blackship_set_speed(int ls, int rs);

int blackship_set_timeout(int tm);


#endif

