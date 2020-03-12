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
 * @file "modules/circle/circle.h"
 * @author Team Wonder
 * Just pathing in circle
 */

#ifndef CIRCLE_H
#define CIRCLE_H



// settings
extern int CIRCLE_L;
extern int CIRCLE_D;
extern int CIRCLE_I;
extern float CIRCLE_X;
extern float CIRCLE_Y;
extern int AVOID_number_of_objects;
extern float AVOID_h1,AVOID_h2;
extern float AVOID_d;
extern float AVOID_safety_angle;

// functions
extern void trajectory_init();
extern void trajectory_periodic();

#endif

