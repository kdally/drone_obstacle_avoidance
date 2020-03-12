/*
 * Copyright (C) abarberia
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
 * @file "modules/observer/observer.h"
 * @author abarberia
 * First version of computer vision algorithm for MAVLAB course
 */

#ifndef OBSERVER_H
#define OBSERVER_H

#include "opencv/cv.h"

// extern void observer_init();
// extern void observer_look();

void observer_init();
void observer_look();

int color_filters[1][6];

void convolve();

#endif

