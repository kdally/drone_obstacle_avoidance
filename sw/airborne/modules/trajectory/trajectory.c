/*
 * Copyright (C) Team Wonder
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
#include "state.h"
#include "modules/observer/observer.h"
#define NUMBER_OF_PARTITIONS 10

enum trajectory_mode_t {
  CIRCLE,
  SQUARE,
  TAKE_OFF
};

enum safety_mode_t {
  SAFE,
  THREAT,
  ESCAPE_IN_PROGRESS,
  HOLD,
  WAIT
};

// set initial trajectory and safety modes
enum trajectory_mode_t trajectory_mode = TAKE_OFF;
enum safety_mode_t safety_mode = SAFE;
void switch_path(enum trajectory_mode_t next_mode);

// define and initialise global variables
// ***** TRAJECTORY variables *****
float TRAJECTORY_X = 0.0;                       // x coodinate for trajectory in global reference system
float TRAJECTORY_Y = 0.0;                       // y coodinate for trajectory in global reference system
float TRAJECTORY_LOCAL_X = 0.0;                 // x coodinate for trajectory in cyberzoo reference system
float TRAJECTORY_LOCAL_Y = 0.0;                 // x coodinate for trajectory in cyberzoo reference system
int TRAJECTORY_L = 1800;                        // cyberzoo length in given coordinate system
float TRAJECTORY_SWITCHING_TIME=19;             // time after which trajectory switches mode
int TRAJECTORY_RADIUS;                          // radius of the trajectory  
int square_mode = 1;                            // indication of which part of the square trajectory to follow
float current_time;                             // elapsed time
int dt;                                         // time steo

// ***** COLOR-FILTERS BASED-AVOIDANCE variables *****
float AVOID_objects[100][3];                    // 2D array storing objects' relative heading and distance
int AVOID_number_of_objects = 0;                // count for the number of detected objects 
int AVOID_biggest_threat;                       // index of objetc representing the biggest threat
float AVOID_safety_angle = 15 * M_PI/180;       // angle within which objects are considered a threat
float AVOID_dist_threat = 1.2;                  // distance at which objects are considered a threat
int AVOID_keep_escape_count = 0;                // number of iterations since the avoidance trajectory has started
int AVOID_keep_escape_count_max = 2200;         // maximum number of iterations to keep following the avoidance trajectory
float AVOID_dist_stop_escape = 1.4;             // maximum travelled distance to keep following the avoidance trajectory
struct EnuCoor_i AVOID_start_avoid_coord;       // coordinate of the location where the avoidance trajectory has started
float AVOID_normal_dt = 0.0003;                 // time step when no threatening object is dectected
float AVOID_slow_dt = 0.00003;                  // time step when a threatening object is dectected
int AVOID_hold_position = 1;

// ***** OPTICAL FLOW-BASED AVOIDANCE variables *****
float AVOID_OF_angle = 3.5 * M_PI/180;          // Angle in front of the drone in which the OF is tested to test if it's safe to move forward
bool safe_mode_previous=false;                  // has the value of true if optical flow based safe mode was activated in the last iteration
int last_iteration_safe_heading=0;              // influence that the safest heading computed based on OF has on the trajectory  
float OF_NEXT_HEADING_INFLUENCE = 0.25;         // gain of escpae route from the optical flow-based avoidance
float OPTICAL_FLOW_THRESHOLD=0.6;               // optical flow above which it's dangerous to move forward

// ***** Distance Counter *****
float distance_travelled;
float last_x;
float last_y;

/*
 * Initialisation function, setting the initial time variables and distance tracker
 */
void trajectory_init(void){

  // initialize time variables
  current_time = 0;
  dt = AVOID_normal_dt;

  // initialise the distance tracker
  distance_travelled=0;
  last_x=stateGetPositionEnu_f()->x;
  last_y=stateGetPositionEnu_f()->y;
}

/*
 * Function that defines a global trajectory and adapts it such that obstacles are avoided
 */
void trajectory_periodic(void)
{

// process detected objects data from color filter algorithm  
AVOID_number_of_objects = 0;
count_objects();
unpack_object_list();

// update trajectory coordinates
switch (trajectory_mode){
  case TAKE_OFF:
    take_off(&TRAJECTORY_X, &TRAJECTORY_Y);
    break;
  case CIRCLE:
    circle(current_time, &TRAJECTORY_X, &TRAJECTORY_Y);
    switch_path(SQUARE);
    break;
  case SQUARE:
    square(dt, &TRAJECTORY_X, &TRAJECTORY_Y);
    switch_path(CIRCLE);
    break;
  default:
    break;
  }

// update goal waypoint based on trajectory
waypoint_set_xy_i(WP_GOAL,TRAJECTORY_X,TRAJECTORY_Y);

// DISABLED FOR COMPETITION BECAUSE OF OPTICAL-FLOW FUNCTIONHIGH RUN TIME
// use optical flow-based avoidance right after take-off and when the trajectory mode is changed 
// if(safety_mode!=ESCAPE_IN_PROGRESS){  
//   if(current_time<2.0){
//       bool change_heading = safety_check_optical_flow(GLOBAL_OF_VECTOR, TRAJECTORY_X, TRAJECTORY_Y);
//     if(change_heading){
//       moveWaypointForwardWithDirection(WP_GOAL,OF_NEXT_HEADING_INFLUENCE,safe_heading(GLOBAL_OF_VECTOR));
//       safe_mode_previous=true;
//     }
//     else{
//       safe_mode_previous=false;
//     }
//   }
// }

// align drone with goal
nav_set_heading_towards_waypoint(WP_GOAL);

// update the travelled distance counter (removed for the competition)
//distance_travelled+=distance_travelled_last_iteration();
//printf("\n Distance tavelled= %f \n", distance_travelled);

// update time
current_time += dt;

return;
}

// *****************************************  TRAJECTORY FUNCTIONS ********************************************
/*
 * Function that updates the trajectory to follow a circular trajectory
 */
void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y)
{
  // choose the ecentricty
  double e = 1;

  // define the default time step
  dt = AVOID_normal_dt;

  // check if threatening objects are present, or if one is currently being avoided
  determine_if_safe(AVOID_dist_stop_escape, AVOID_dist_threat);

  if(safety_mode==SAFE){

    // default radius to half of the cyberzoo
    TRAJECTORY_RADIUS = TRAJECTORY_L/2;
  }
  else if(safety_mode==THREAT){

    // reduce the speed in case of threat, implemented as a smaller time step 
    dt = AVOID_slow_dt;

    // high radius reduction if the right side has a relative heading smaller than 40 deg
    if(AVOID_objects[AVOID_biggest_threat][0]<-0.70){
      TRAJECTORY_RADIUS = TRAJECTORY_L/2 - 400;   
    }

    // smaller radius reduction otherwise
    else{
      TRAJECTORY_RADIUS = TRAJECTORY_L/2 - 300;   
    }

    // keep the escape trajectory by setting the safety mode and iteration count
    safety_mode = ESCAPE_IN_PROGRESS;
    AVOID_keep_escape_count += 1;
  }
  // if a threatening object is currently being avoided, keep the same radius
  //else if(safety_mode==ESCAPE_IN_PROGRESS){}

  // convert to cartesian coordinates, in the cyberzoo local coordinate system 
  TRAJECTORY_LOCAL_X = TRAJECTORY_RADIUS * cos(current_time);
  TRAJECTORY_LOCAL_Y = e * TRAJECTORY_RADIUS * sin(current_time);

  // rotate trajectory to match orientation of the ciberzoo
  *TRAJECTORY_X = TRAJECTORY_LOCAL_X*0.5+TRAJECTORY_LOCAL_Y*0.866025;
  *TRAJECTORY_Y = -TRAJECTORY_LOCAL_X*0.866025+TRAJECTORY_LOCAL_Y*0.5;

return;
}

/*
 * Function that updates the trajectory to follow a square trajectory
 */
void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y)
{
  // choose the speed factor
  int V = 700;

  // define the default time step and local coordinates
  dt = AVOID_normal_dt;

  // check if threatening objects are present, or if one is currently being avoided
  determine_if_safe(AVOID_dist_stop_escape, AVOID_dist_threat);

  if(safety_mode==SAFE){

    // default radius to half of the cyberzoo
    TRAJECTORY_RADIUS = TRAJECTORY_L/2;
  }
  else if(safety_mode==THREAT){

    // reduce the speed in case of threat, implemented as a smaller time step 
    dt = AVOID_slow_dt;

    // high radius reduction if the right side has a relative heading smaller than 40 deg
    if(AVOID_objects[AVOID_biggest_threat][0]<-0.70){
      TRAJECTORY_RADIUS = TRAJECTORY_L/2 - 400;   
    }

    // smaller radius reduction otherwise
    else{
      TRAJECTORY_RADIUS = TRAJECTORY_L/2 - 300;   
    }

    // keep the escape trajectory by setting the safety mode and iteration count
    AVOID_keep_escape_count += 1;
    safety_mode = ESCAPE_IN_PROGRESS;
  }
  // if a threatening object is currently being avoided, keep the same radius
  //else if(safety_mode==ESCAPE_IN_PROGRESS){}

  //update coordinates depending on which side of the circle the drone
  if(square_mode==1){
    TRAJECTORY_LOCAL_X = TRAJECTORY_RADIUS;
    TRAJECTORY_LOCAL_Y += dt*V;

    if (TRAJECTORY_LOCAL_Y > TRAJECTORY_RADIUS){
      square_mode=2;
    }
  }
  if(square_mode==2){
    TRAJECTORY_LOCAL_X -= dt*V;
    TRAJECTORY_LOCAL_Y = TRAJECTORY_RADIUS;

    if (TRAJECTORY_LOCAL_X<-TRAJECTORY_RADIUS){
      square_mode=3;
    }
  }
  if(square_mode==3){
    TRAJECTORY_LOCAL_X =-TRAJECTORY_RADIUS;
    TRAJECTORY_LOCAL_Y -= dt*V;

    if (TRAJECTORY_LOCAL_Y<-TRAJECTORY_RADIUS){
      square_mode=4;
    }
  }
  if(square_mode==4){
    TRAJECTORY_LOCAL_X += dt*V;
    TRAJECTORY_LOCAL_Y =- TRAJECTORY_RADIUS;

    if (TRAJECTORY_LOCAL_X>TRAJECTORY_RADIUS){
      square_mode=1;
    }
  }
  
  // rotate trajectory to match orientation of the ciberzoo
  *TRAJECTORY_X = TRAJECTORY_LOCAL_X*0.5+TRAJECTORY_LOCAL_Y*0.866025;
  *TRAJECTORY_Y = -TRAJECTORY_LOCAL_X*0.866025+TRAJECTORY_LOCAL_Y*0.5;

  return;
}

/*
 * Function that updates the trajectory to safely take-off and join the next trajectory
 */
void take_off(float *TRAJECTORY_X, float *TRAJECTORY_Y){

  // distance to travel to complete take-off procedure
  float distance_forward = 2.0;

  // use a small time step during take-off the remain slow
  dt = AVOID_slow_dt;

  // check if threatening objects are present, or if one is currently being avoided. 
  // Leave the take-off mode after having travelled 1.7m, and consider obstacles up to a 5m distance a threat (entire cyberzoo)
  AVOID_safety_angle = 40*M_PI/180;
  
  determine_if_safe(1.5, 4.0);

  if(safety_mode==HOLD){
    float new_heading = 100*M_PI/180;
    FLOAT_ANGLE_NORMALIZE(new_heading);
    nav_heading = ANGLE_BFP_OF_REAL(new_heading);
    *TRAJECTORY_X = 0.0;
    *TRAJECTORY_Y = 0.0;
  }
  //printf("\n dist %f \n", final_objs[AVOID_biggest_threat][2]);
 
  if(safety_mode==THREAT){
    
    // choose next heading 45 deg to the right of the right edge of the closest obstacle
    float new_heading = stateGetNedToBodyEulers_f()->psi + AVOID_objects[AVOID_biggest_threat][0] - 15*M_PI/180;
    FLOAT_ANGLE_NORMALIZE(new_heading);
    nav_heading = ANGLE_BFP_OF_REAL(new_heading);
    //printf("\n %f \n", new_heading*180/M_PI);

    // update trajectory coordinates based on the new heading and distance forward
    *TRAJECTORY_X = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(new_heading) * (distance_forward));
    *TRAJECTORY_Y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(new_heading) * (distance_forward));
    
    // keep the escape trajectory by setting the safety mode and iteration count
    safety_mode = ESCAPE_IN_PROGRESS;
    AVOID_keep_escape_count += 1;
  }
  else if(safety_mode==SAFE){
    
    // set current heading to next heading since no obstacles forward
    float new_heading  = stateGetNedToBodyEulers_f()->psi;
    //printf("\n safe %f \n", new_heading*180/M_PI);

    // update trajectory coordinates based on the new heading and distance forward
    *TRAJECTORY_X = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(new_heading) * (distance_forward));
    *TRAJECTORY_Y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(new_heading) * (distance_forward));

    // keep the escape trajectory by setting the safety mode and iteration count
    safety_mode = ESCAPE_IN_PROGRESS;
    AVOID_keep_escape_count += 1;
    }
  
  // if a threatening object is currently being avoided, keep the same radius
  //else if(safety_mode==ESCAPE_IN_PROGRESS){}

}

/*
 * Function that switches trajectory when a certain time has passed
 */
void switch_path(enum trajectory_mode_t next_mode){
  if (current_time > TRAJECTORY_SWITCHING_TIME){
      current_time=0;
      TRAJECTORY_Y=0;
      TRAJECTORY_X=0;  
      square_mode=1; 
      trajectory_mode = next_mode;
    }
return;
}

// ***************************************** AVOIDANCE STRATEGY FUNCTIONS *****************************************
/*
 * Function that determines the safety mode
 */
void determine_if_safe(float dist_stop_escape, float dist_threat){

  if(stateGetPositionEnu_f()->z < 0.1){
    safety_mode = WAIT;
    return;
  }

  if(AVOID_hold_position!=0){
    safety_mode = HOLD;
    AVOID_hold_position += 1;

    if(AVOID_hold_position>2000){
      AVOID_hold_position = 0; 
    }
  return;
  }

  // if the trajectory function has determined an escpae trajectory
  if(AVOID_keep_escape_count!=0){
    safety_mode = ESCAPE_IN_PROGRESS;
    AVOID_keep_escape_count += 1;

    if(trajectory_mode==TAKE_OFF){
      //if(isCoordOutsideRadius(&AVOID_start_avoid_coord, dist_stop_escape) == true || AVOID_keep_escape_count > AVOID_keep_escape_count_max){
      if(isCoordOutsideRadius(&AVOID_start_avoid_coord, dist_stop_escape) == true){
        AVOID_keep_escape_count = 0;
        trajectory_mode = SQUARE;
        current_time = 0;
      }
    }
    else{
      if(isCoordOutsideRadius(&AVOID_start_avoid_coord, dist_stop_escape) == true || AVOID_keep_escape_count > AVOID_keep_escape_count_max){
          AVOID_keep_escape_count = 0;
      }
    }
    return;
  }

  // default safety mode
  safety_mode = SAFE;

  // iterating trhough each identified objects
  for(int i = 0; i < AVOID_number_of_objects; i++){

     // checks whether the object is contained within the vision cone defined by AVOID_safety_angle
    if((fabs(AVOID_objects[i][0]) < AVOID_safety_angle || fabs(AVOID_objects[i][1]) < AVOID_safety_angle || AVOID_objects[i][0]*AVOID_objects[i][1] < 0) &&  final_objs[i][2]<dist_threat){
      
      // define the biggest threat as the closest object (final_objs[i][2] is the distance of object i to the drone)
      if(i==0 || final_objs[i][2] < final_objs[i-1][2]){
          AVOID_biggest_threat = i;
      }

    // switch safety mode and record coordinates of where the avoidance is to be started
    safety_mode = THREAT;
    setCoordHere(&AVOID_start_avoid_coord); 
  }
}
return;
}

/* 
 * Function that checks the safety according to optical flow input
 * Returns true if it's not safe and there's the need to change heading
 * Tuning parameters: AVOID_PERCENTAGE_THRESHOLD and AVOID_OF_angle 
 */
bool safety_check_optical_flow(float *AVOID_safety_optical_flow, float x2, float y2){
  int i1=convert_heading_to_index(-AVOID_OF_angle, OF_NUMBER_ELEMENTS);
  int i2=convert_heading_to_index(AVOID_OF_angle, OF_NUMBER_ELEMENTS);

  bool change_heading=false;
  for (int i = i1; i <= i2; i++){
    if(AVOID_safety_optical_flow[i]>OPTICAL_FLOW_THRESHOLD){
          change_heading=true;
      }
  }

  return change_heading;
}

/* Function that returns the safest heading according to OF values
 * - divides the field of view in NUMBER_OF_PARTITIONS partitions
 * - Finds the partition with smallest OF (partition i)
 * - saffest heading is the middle value of the i-th partition with span="angular_span"
 */
float safe_heading(float array_of[]){
  //printf("\nOF activated\n");
  float field_of_view=M_PI/2;
  float angular_span=field_of_view/NUMBER_OF_PARTITIONS;
  
  float partition_OF[NUMBER_OF_PARTITIONS]; //array with field of view partitions
  int indexis[NUMBER_OF_PARTITIONS];        //array with indexis correspondent to each partition
  for (int i=0; i<NUMBER_OF_PARTITIONS; i++){   
      partition_OF[i]=0;
      indexis[i]=i;
  }

  float h1=-1*field_of_view/2;     //h2 and h1 are the limits 
  float h2=h1+angular_span;        //of the partition being processed
  int i1,i2;      //i1 and i2 are the indices corresponding to h1 and h2 

  //calculating OF in each partition
  for (int i = 0; i < NUMBER_OF_PARTITIONS; i++){
    i1=convert_heading_to_index(h1, OF_NUMBER_ELEMENTS);
    i2=convert_heading_to_index(h2, OF_NUMBER_ELEMENTS);

    //average of OF in the i-th span
    for (int j=i1; j <= i2; j++){   
      partition_OF[i]+=array_of[j];
    }
    partition_OF[i]=partition_OF[i]/(i2-i1+1);

    //updating the values for next iteration
    h1+=angular_span;
    h2=h1+angular_span;
  }

  //prioritizing the last heading for consistency
  if(safe_mode_previous){
    if(partition_OF[last_iteration_safe_heading]<0.5){
      float safest_heading = -1*field_of_view/2 + last_iteration_safe_heading * angular_span + angular_span/2;
      return safest_heading;
    }
  }

  //prioriziting left headings
  for (int i = 0; i < NUMBER_OF_PARTITIONS; i++){
    if(partition_OF[i]<0.5){
      float safest_heading = -1*field_of_view/2 + i * angular_span + angular_span/2;
      last_iteration_safe_heading=i;
      return safest_heading;
    }
  }
  
  //Finds the partition with smallest OF
  quickSort(partition_OF,indexis,0,NUMBER_OF_PARTITIONS-1);

  float safest_heading = -1*field_of_view/2 + indexis[0] * angular_span + angular_span/2; //partition with lowest OF average
  last_iteration_safe_heading=indexis[0];
  return safest_heading;
}

/* 
 * Function that converts the index of OF array to the corresponding heading 
 * (due to distortion)
 */
float convert_index_to_heading(int index, int N){
  float heading=2.0*index/(N-1)-1.0;  //normalize the index 
  heading=atan(heading); 
  return heading; //heading in radians
}

/* 
 * Converts the heading to the index of OF array (due to distortion)
 */
int convert_heading_to_index(float heading, int N){
  int index=(1+tan(heading))*(N-1)/2;
  return index; 
}

/* 
 * Sorting function using quick sort algorithm
 */
void quickSort(float array[], int indecis[], int first,int last){
   int i, j, pivot, temp2;
   float temp;

   if(first<last){
      pivot=first;
      i=first;
      j=last;

      while(i<j){
         while(array[i]<=array[pivot]&&i<last)
            i++;
         while(array[j]>array[pivot])
            j--;
         if(i<j){
            temp=array[i];
            array[i]=array[j];
            array[j]=temp;
            temp2=indecis[i];
            indecis[i]=indecis[j];
            indecis[j]=temp2;            
         }
      }

      temp=array[pivot];
      array[pivot]=array[j];
      array[j]=temp;
      temp2=indecis[pivot];
      indecis[pivot]=indecis[j];
      indecis[j]=temp2;
      quickSort(array, indecis, first,j-1);
      quickSort(array, indecis ,j+1,last);
   }
}

/*
 * Function that counts the number of objects currently being detected
 */
void count_objects(){
  for (int i = 0; i < 10; i++) {
      if (final_objs[i][0] != 0 || final_objs[i][1] != 0){
          AVOID_number_of_objects = AVOID_number_of_objects+1;
      }
  }
  return;
}

/*
 * Function that converts the pixel index to relative heading for all objects found
 */
void unpack_object_list(){
  for (int i = 0; i < AVOID_number_of_objects; i++) {
      AVOID_objects[i][0] = convert_index_to_heading(final_objs[i][0], 519);
      AVOID_objects[i][1] = convert_index_to_heading(final_objs[i][1], 519);
      AVOID_objects[i][2] = final_objs[i][2];
  }
  return;
}

// ************************************* NAVIGATION FUNCTIONS ********************************************
/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading and given direction
 */
static uint8_t calculateForwardsWithDirection(struct EnuCoor_i *new_coor, float distanceMeters, float direction)
{
  float heading  = stateGetNedToBodyEulers_f()->psi + direction; 

  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));

  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor' (copied from orange_avoider.c)
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                // POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward with given direction and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForwardWithDirection(uint8_t waypoint, float distanceMeters, float direction)
{
  struct EnuCoor_i new_coor;
  calculateForwardsWithDirection(&new_coor, distanceMeters, direction);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Set a pair of coordinates to current position
 */
void setCoordHere(struct EnuCoor_i *coord){
  coord->x = stateGetPositionEnu_f()->x;
  coord->y = stateGetPositionEnu_f()->y;
  return;
}

/*
 * Check whether a coordinate is outside a certain distance from the current position
 */
bool isCoordOutsideRadius(struct EnuCoor_i *coord, float radius){

  float dx = coord->x - stateGetPositionEnu_f()->x;
  float dy = coord->y - stateGetPositionEnu_f()->y;
  float dist = sqrt(dx*dx+dy*dy);
  
  if(dist > radius){
    return true;
  }
  else{
  return false;
  }
}

/*
 * Returns the distance travelled in the last iteration
 */
float distance_travelled_last_iteration(){
  float dx=stateGetPositionEnu_f()->x - last_x;
  float dy=stateGetPositionEnu_f()->y - last_y;

  //update last position
  last_x=stateGetPositionEnu_f()->x;
  last_y=stateGetPositionEnu_f()->y;

  //add new distance travelleds
  float r=sqrt(dx*dx+dy*dy);
  return r;
}