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


// declare external variables
extern float *GLOBAL_OF_VECTOR;


// functions
extern void trajectory_init();
extern void trajectory_periodic();
float distance_travelled_last_iteration();
void circle(float current_time, float *TRAJECTORY_X, float *TRAJECTORY_Y);
void square(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y);
void take_off(float *TRAJECTORY_X, float *TRAJECTORY_Y);
void lace(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r);
void lace_inverted(float dt, float *TRAJECTORY_X, float *TRAJECTORY_Y, int r);
float convert_index_to_heading(int index, int N);
int convert_heading_to_index(float heading, int N);
void quickSort(float array[], int indecis[], int first,int last);
bool safety_check_optical_flow(float *AVOID_safety_optical_flow, float x2, float y2);
float safe_heading(float array_of[]);
void unpack_object_list();
void count_objects();
void determine_if_safe(float dist_stop_escape, float dist_threat);
void setCoordHere(struct EnuCoor_i *coord);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForwardWithDirection(uint8_t waypoint, float distanceMeters, float direction);
bool isCoordOutsideRadius(struct EnuCoor_i *new_coor, float radius);

#endif

