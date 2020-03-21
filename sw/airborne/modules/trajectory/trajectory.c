/*
 * CopTRAJECTORY_Yright (C) Team Wonder
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; TRAJECTORY_You can redistribute it and/or modifTRAJECTORY_Y
 * it under the terms of the GNU General Public License as published bTRAJECTORY_Y
 * the Free Software Foundation; either version 2, or (at TRAJECTORY_Your option)
 * anTRAJECTORY_Y later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANTRAJECTORY_Y WARRANTTRAJECTORY_Y; without even the implied warrantTRAJECTORY_Y of
 * MERCHANTABILITTRAJECTORY_Y or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * TRAJECTORY_You should have received a copTRAJECTORY_Y of the GNU General Public License
 * along with paparazzi; see the file COPTRAJECTORY_YING.  If not, see
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


int TRAJECTORY_L = 1434; //1434 for circle
int TRAJECTORY_D = 7;
float TRAJECTORY_X = 0;
float TRAJECTORY_Y = 0;
int AVOID_number_of_objects = 0;
float AVOID_h1,AVOID_h2;
float AVOID_d;
float AVOID_safety_angle=10;

void trajectory_init(void)
{}

float current_time = 0;

void trajectory_periodic(void)
{

nav_set_heading_towards_waypoint(WP_STDBY);

double dt = 0.001;
current_time += dt;
double e = 0.7;

int mode=1;

switch (mode)
{
case 1:
  circle();
  if (NavBlockTime() > 20)
    mode=2;
  break;

case 2:
  square();
  break;
}

float x_rotated=TRAJECTORY_X*0.5+TRAJECTORY_Y*0.866025;
float y_rotated=-TRAJECTORY_X*0.866025+TRAJECTORY_Y*0.5;

waypoint_set_xy_i(WP_STDBY,x_rotated,y_rotated);
}

void circle()
{
  
  int r = TRAJECTORY_L/2 - TRAJECTORY_D;   
  if(AVOID_number_of_objects!=0)
  {
    float r_reduced=0;
    float offset=asin(AVOID_d/(2*r))*180/M_PI; //offset in degrees
    if( AVOID_h1-offset<AVOID_safety_angle ||-1*AVOID_safety_angle<AVOID_h2-offset)
    {
      r_reduced=r*(AVOID_h2-offset)*M_PI/180;
    }
    r -= r_reduced;
  }

    int32_t TRAJECTORY_X = r * cos(current_time);
    int32_t TRAJECTORY_Y = e * r * sin(current_time);
  
  return;
}

void square()
{
int V = 70;
int square_mode;

if(square_mode==1){
  TRAJECTORY_X = r;
  TRAJECTORY_Y += dt*V;

  if (TRAJECTORY_Y > r){
    square_mode=2;
  }
}

if(square_mode==2){
  TRAJECTORY_X -= dt*V;
  TRAJECTORY_Y=r;

  if (TRAJECTORY_X<-r){
    square_mode=3;
  }
}

if(square_mode==3){
  TRAJECTORY_X=-r;
  TRAJECTORY_Y-=dt*V;

  if (TRAJECTORY_Y<-r){
    square_mode=4;
  }
}

if(square_mode==4){
  TRAJECTORY_X+=dt*V;
  TRAJECTORY_Y=-r;

  if (TRAJECTORY_X>r){
    square_mode=1;
  }
}
  return;
}