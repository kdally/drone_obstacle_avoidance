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
 * @file "modules/TRAJECTORY/TRAJECTORY.h"
 * @author Team Wonder
 * Just pathing in TRAJECTORY
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#define OF_NUMBER_ELEMENTS 173
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdbool.h> 

#include "modules/computer_vision/cv.h"


// settings
extern int TRAJECTORY_L;
extern int TRAJECTORY_D;
extern int AVOID_number_of_objects;
extern float AVOID_h1,AVOID_h2;
extern float AVOID_d;
extern float AVOID_safety_angle;
extern float TRAJECTORY_X;
extern float TRAJECTORY_Y;
extern float TRAJECTORY_SWITCHING_TIME;

extern float *GLOBAL_OF_VECTOR;
//extern uint16_t *GLOBAL_OBJECTS_VECTOR;

// functions
extern void trajectory_init();
extern void trajectory_periodic();
extern float distance_travelled_last_iteration();
extern void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r);
extern void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r);
extern void lace(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r);
extern void lace_inverted(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r);
extern float convert_index_to_heading(int index, int N);
extern int convert_heading_to_index(float heading, int N);
extern void quickSort(float array[], int indecis[], int first,int last);
extern bool safety_check_optical_flow(float *AVOID_safety_optical_flow, float x2, float y2);
extern float safe_heading(float array_of[]);
extern void unpack_object_list();
extern void count_objects();
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(float incrementDegrees);
extern uint8_t moveWaypointForwardWithDirection(uint8_t waypoint, float distanceMeters, float direction);
extern bool isCoordInRadius(struct EnuCoor_i *new_coor, float radius);

#endif

