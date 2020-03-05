/*
 * Copyright (C) Team Wonder
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/circle/circle.c"
 * @author Team Wonder
 * Just pathing in circle
 */

#include "modules/circle/circle.h"
#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"


int CIRCLE_L = 1334;
int CIRCLE_D = 7;


void circle_init(void)
{

  int i=1;
  double x=0;
  double y=0;

}

float current_time = 0;

void circle_periodic(void)
{

int r = CIRCLE_L/2 - CIRCLE_D;

double dt = 0.001;
current_time += dt;
double e = 1;

 int32_t x = r * cos(current_time);
 int32_t y = e * r * sin(current_time);


// int V = 1;

// if(i==1){
//   x=r;
//   y+=dt*V;

//   if (y>r){
//     i=2;
//   }
// }

// if(i==2){
//   x-=dt*V;
//   y=r;

//   if (x<-r){
//     i=3;
//   }
// }

// if(i==3){
//   x=-r;
//   y-=dt*V;

//   if (y<-r){
//     i=4;
//   }
// }

// if(i==4){
//   x+=dt*V;
//   y=-r;

//   if (x>r){
//     i=1;
//   }
// }


waypoint_set_xy_i(WP_STDBY,x,y);

}