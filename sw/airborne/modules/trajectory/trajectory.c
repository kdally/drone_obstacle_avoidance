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

#include "modules/trajectory/trajectory.h"
#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"


int TRAJECTORY_L = 1434; //1434 for circle
int TRAJECTORY_D = 7;
int AVOID_number_of_objects = 0;
float AVOID_h1,AVOID_h2;
float AVOID_d;
float AVOID_safety_angle=10;
float TRAJECTORY_X;
float TRAJECTORY_Y;

double current_time = 0;
int square_mode = 1;
int mode=1;


void trajectory_init(void)
{
}



void trajectory_periodic(void)
{

nav_set_heading_towards_waypoint(WP_STDBY);

double dt = 0.0001;
current_time += dt;
int V = 70;
int r = TRAJECTORY_L/2 - TRAJECTORY_D;   

TRAJECTORY_X = r * cos(current_time);
TRAJECTORY_Y = r * sin(current_time);
  


// if (mode==1)
// {
//   //circle(current_time, &TRAJECTORY_X, &TRAJECTORY_Y);
 
//   if (current_time > 200)
//     mode=1;
// }

// else
// {
//   //square(dt, &TRAJECTORY_X, &TRAJECTORY_Y);
//   if(square_mode==1){
//     TRAJECTORY_X = r;
//     TRAJECTORY_Y += dt*V;

//     if (TRAJECTORY_Y > r){
//       square_mode=2;
//     }
//   }

//   if(square_mode==2){
//     TRAJECTORY_X -= dt*V;
//     TRAJECTORY_Y=r;

//     if (TRAJECTORY_X<-r){
//       square_mode=3;
//     }
//   }

//   if(square_mode==3){
//     TRAJECTORY_X=-r;
//     TRAJECTORY_Y-=dt*V;

//     if (TRAJECTORY_Y<-r){
//       square_mode=4;
//     }
//   }

//   if(square_mode==4){
//     TRAJECTORY_X+=dt*V;
//     TRAJECTORY_Y=-r;

//     if (TRAJECTORY_X>r){
//       square_mode=1;
//     }
//   }
 
// }

float x_rotated=TRAJECTORY_X*0.5+TRAJECTORY_Y*0.866025;
float y_rotated=-TRAJECTORY_X*0.866025+TRAJECTORY_Y*0.5;

waypoint_set_xy_i(WP_STDBY,x_rotated,y_rotated);
}

/*
void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  double e = 1;
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

    *TRAJECTORY_X = r * cos(current_time);
    *TRAJECTORY_Y = e * r * sin(current_time);
  
  return;
}*/

// void square(double dt, float *TRAJECTORY_X, float *TRAJECTORY_Y)
// {
//   int r = TRAJECTORY_L/2 - TRAJECTORY_D;   
//   int V = 70;


//   if(square_mode==1){
//     *TRAJECTORY_X = r;
//     *TRAJECTORY_Y += dt*V;

//     if (*TRAJECTORY_Y > r){
//       square_mode=2;
//     }
//   }

//   if(square_mode==2){
//     *TRAJECTORY_X -= dt*V;
//     *TRAJECTORY_Y=r;

//     if (*TRAJECTORY_X<-r){
//       square_mode=3;
//     }
//   }

//   if(square_mode==3){
//     *TRAJECTORY_X=-r;
//     *TRAJECTORY_Y-=dt*V;

//     if (*TRAJECTORY_Y<-r){
//       square_mode=4;
//     }
//   }

//   if(square_mode==4){
//     *TRAJECTORY_X+=dt*V;
//     *TRAJECTORY_Y=-r;

//     if (*TRAJECTORY_X>r){
//       square_mode=1;
//     }
//   }
//     return;
// }