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

enum trajectory_mode_t {
  CIRCLE,
  SQUARE,
  LACE,
  INVERTED_LACE
};

enum trajectory_mode_t trajectory_mode = CIRCLE;
int TRAJECTORY_L = 1750; //for a dt f 0.0011, razor thin margins
// int TRAJECTORY_L = 1550; //for a dt f 0.0004
int TRAJECTORY_D = 0;
float TRAJECTORY_X=0;
float TRAJECTORY_Y=0;
float TRAJECTORY_SWITCHING_TIME=20;

float current_time = 0;
int square_mode = 1;
int lace_mode = 1;
int mode=1;
// float dt=0.0004; // 0.6 m/s
float dt=0.0011; // up to 1.6 m/s
// float dt=0.0015; //very fast

int AVOID_number_of_objects = 0;
float AVOID_h1,AVOID_h2;
float AVOID_d;
float AVOID_safety_angle = 10;
float AVOID_FOV = 80;


void trajectory_init(void){}

void trajectory_periodic(void)
{

safety_check_optical_flow(GLOBAL_OF_VECTOR);

nav_set_heading_towards_waypoint(WP_STDBY);

/*
//array with the positions
int indecis[OF_NUMBER_ELEMENTS];
for(int i=0;i<OF_NUMBER_ELEMENTS;i++){
    indecis[i]=i;
}
//to call sorting uncoment next line
//quickSort(OF_array,indecis,0,OF_NUMBER_ELEMENTS-1);
*/

current_time += dt;
int r = TRAJECTORY_L/2 - TRAJECTORY_D;   

switch (trajectory_mode){
  case CIRCLE:

    circle(current_time, &TRAJECTORY_X, &TRAJECTORY_Y, r);
    
    if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        TRAJECTORY_Y=0;
        TRAJECTORY_X=0;  
        square_mode=1; 
        trajectory_mode = SQUARE;
      }

    break;
  case SQUARE:

    square(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);

    if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
      current_time=0;
      TRAJECTORY_Y=0;
      TRAJECTORY_X=0; 
      lace_mode=1;
      trajectory_mode = LACE;
    }

    break;
  case LACE:

    lace(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);

      if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        TRAJECTORY_Y=0;
        TRAJECTORY_X=0; 
        lace_mode=1;
        trajectory_mode = INVERTED_LACE;
      }

      break;
  case INVERTED_LACE:

    lace_inverted(dt, &TRAJECTORY_X, &TRAJECTORY_Y, r);

      if (current_time > TRAJECTORY_SWITCHING_TIME){ //move to another mode
        current_time=0;
        trajectory_mode = CIRCLE;
      }

      break;
  default:
    break;
  }

float x_rotated=TRAJECTORY_X*0.5+TRAJECTORY_Y*0.866025;
float y_rotated=-TRAJECTORY_X*0.866025+TRAJECTORY_Y*0.5;

waypoint_set_xy_i(WP_STDBY,x_rotated,y_rotated);

float *GLOBAL_OF_VECTOR = NULL;

}



void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  double e = 1;
  //   if(AVOID_number_of_objects!=0)
  // {
  //   float r_reduced=0;
  //   float offset=asin(AVOID_d/(2*r))*180/M_PI; //offset in degrees
  //   if( AVOID_h1-offset<AVOID_safety_angle ||-1*AVOID_safety_angle<AVOID_h2-offset)
  //   {
  //     r_reduced=r*(AVOID_h2-offset)*M_PI/180;
  //   }
  //   r -= r_reduced;
  // }  

    if(AVOID_number_of_objects!=0)
    {
      if(abs(AVOID_h1)<AVOID_safety_angle || abs(AVOID_h2)<AVOID_safety_angle)
     {
       r-=AVOID_h2*1; //adjust gain - we have tune this experimentaly!!
     }
    }

    *TRAJECTORY_X = r * cos(current_time);
    *TRAJECTORY_Y = e * r * sin(current_time);
  
  return;
}

void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  if(AVOID_number_of_objects!=0)
  {
    if(abs(AVOID_h1)<AVOID_safety_angle || abs(AVOID_h2)<AVOID_safety_angle)
    {
      r-=AVOID_h2*1; //adjust gain - we have tune this experimentaly!!
    }
  }

  if(square_mode==1){
    *TRAJECTORY_X = r;
    *TRAJECTORY_Y += dt*V;

    if (*TRAJECTORY_Y > r){
      square_mode=2;
    }
  }

  if(square_mode==2){
    *TRAJECTORY_X -= dt*V;
    *TRAJECTORY_Y=r;

    if (*TRAJECTORY_X<-r){
      square_mode=3;
    }
  }

  if(square_mode==3){
    *TRAJECTORY_X=-r;
    *TRAJECTORY_Y-=dt*V;

    if (*TRAJECTORY_Y<-r){
      square_mode=4;
    }
  }

  if(square_mode==4){
    *TRAJECTORY_X+=dt*V;
    *TRAJECTORY_Y=-r;

    if (*TRAJECTORY_X>r){
      square_mode=1;
    }
  }
    return;
}

void lace(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;
  
  if(AVOID_number_of_objects!=0)
  {
    if(abs(AVOID_h1)<AVOID_safety_angle || abs(AVOID_h2)<AVOID_safety_angle)
    {
      r-=AVOID_h2*1; //adjust gain - we have tune this experimentaly!!
    }
  }

  if(lace_mode==1){
    *TRAJECTORY_X = r;
    *TRAJECTORY_Y -= dt*V;

    if (*TRAJECTORY_Y < -r){
      lace_mode=2;
    }
  }

  if(lace_mode==2){
    *TRAJECTORY_X -= dt*V*0.70710678;
    *TRAJECTORY_Y=-1 * *TRAJECTORY_X;

    if (*TRAJECTORY_X<-r){
      lace_mode=3;
    }
  }

  if(lace_mode==3){
    *TRAJECTORY_X=-r;
    *TRAJECTORY_Y-=dt*V;

    if (*TRAJECTORY_Y<-r){
      lace_mode=4;
    }
  }

  if(lace_mode==4){
    *TRAJECTORY_X+=dt*V*0.70710678;
    *TRAJECTORY_Y=*TRAJECTORY_X;

    if (*TRAJECTORY_X>r){
      lace_mode=1;
    }
  }
    return;
}

void lace_inverted(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r)
{
  int V = 700;

  if(AVOID_number_of_objects!=0)
  {
    if(abs(AVOID_h1)<AVOID_safety_angle || abs(AVOID_h2)<AVOID_safety_angle)
    {
      r-=AVOID_h2*1; //adjust gain - we have tune this experimentaly!!
    }
  }

  if(lace_mode==1){
    *TRAJECTORY_Y = r;
    *TRAJECTORY_X -= dt*V;

    if (*TRAJECTORY_X < -r){
      lace_mode=2;
    }
  }

  if(lace_mode==2){
    *TRAJECTORY_Y -= dt*V*0.70710678;
    *TRAJECTORY_X=-1 * *TRAJECTORY_Y;

    if (*TRAJECTORY_Y<-r){
      lace_mode=3;
    }
  }

  if(lace_mode==3){
    *TRAJECTORY_Y=-r;
    *TRAJECTORY_X-=dt*V;

    if (*TRAJECTORY_X<-r){
      lace_mode=4;
    }
  }

  if(lace_mode==4){
    *TRAJECTORY_Y+=dt*V*0.70710678;
    *TRAJECTORY_X=*TRAJECTORY_Y;

    if (*TRAJECTORY_Y>r){

      lace_mode=1;
    }
  }
    return;
}

void avoidance_straight_path(float AVOID_h1, float AVOID_h2){

// h1 and h2 are the left and right headings in degrees of the obstable in a realtive 
// reference frame w/ origin in the center of the FOV 

if(AVOID_number_of_objects!=0){
  float heading_increment = 0;

  if(AVOID_h1 > 0 && AVOID_h1 < AVOID_safety_angle){
    heading_increment = AVOID_h1 - AVOID_safety_angle; // linear function to adapt heading change
  }

  if(AVOID_h2 < 0 && AVOID_h2 > -1*AVOID_safety_angle){
    heading_increment = AVOID_h1 + AVOID_safety_angle; // linear function to adapt heading change
  }

  if(AVOID_h2 > -1*AVOID_safety_angle && AVOID_h2 < 0){

    if(abs(AVOID_h1) > abs(AVOID_h2)){
      heading_increment = AVOID_safety_angle;
    }
    else{
      heading_increment = -AVOID_safety_angle;
    }
  }

  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(heading_increment); // Check thaatRadOfDeg works with Floats
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  }
}



void safety_check_optical_flow(float *AVOID_safety_optical_flow){


// for(int i=0; i< 173; i++){
//   printf("%.6f", *(AVOID_safety_optical_flow+i));
// }



// float indices[] = AVOID_safety_optical_flow[];

// float OF_values[] = AVOID_safety_optical_flow[];

// for (i = 0; i < 10; i++) {
//     sum += array[i];
// }
  
}



float convert_index_to_heading(int index, int N){
  float heading=2*index/(N-1)-1;  //normalize the index 
  heading=asin(heading); 
  return heading; //heading in radians
}

int convert_heading_to_index(float heading, int N){
  int index=(1+sin(heading))/(N-1)*2;
  return index; 
}

void quickSort(float array[],int indecis[], int low,int high)
{
    if (low < high)
    {
        /* pi is partitioning index, arr[pi] is now
           at right place */
        int pi = partition(array, indecis, low, high);

        quickSort(array, indecis, low, pi - 1);  // Before pi
        quickSort(array, indecis, pi + 1, high); // After pi
    }
    return;
}

int partition(float array[], int indecis[], int low, int high)
{
  int pivot = array[high];
  int i = (low - 1);

  for (int j = low; j < high; j++)
  {
    if (array[j] <= pivot)
    {
      i++;
      float aux=array[i];
      array[i]=array[j];
      array[j]=aux;
      int aux2=indecis[i];
      indecis[i]=indecis[j];
      indecis[j]=aux2;
    }
  }
  float aux = array[i+1];
  array[i+1]=array[high];
  array[high]=aux;
  int aux2 = indecis[i+1];
  indecis[i+1]=indecis[high];
  indecis[high]=aux2;
  
  return (i + 1);
}