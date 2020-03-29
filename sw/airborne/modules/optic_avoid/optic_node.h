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
 * @file "modules/optic_avoid/optic_node.h"
 * @author Team Wonder
 * Node for avoidance with optical flow
 */

#include "modules/computer_vision/cv.h"
#include "modules/optic_avoid/optic_avoid.h"
#include <stdio.h>
#include <time.h>

#ifndef OPTIC_AVOID_NODE_H
#define OPTIC_AVOID_NODE_H

/**
   * @brief Initialises the optic avoid module. Adds optic_avoid_func to the video stream
   * @return: void.
*/
extern void optic_avoid_node_init(void);

/**
   * @brief Calculates the optical flow of image, given a previous image, and indicates safety of each row of pixel corresponding to heading
   * @param img: pointer to latest frame
   * @return: Returns image of struct image_t. Changes global 1D array (GLOBAL_OF_VECTOR) of 'danger' factor (0 -> 1) for each row of pixels (frame is rotated from physical orientation)
*/
struct image_t *optic_avoid_func(struct image_t *img);

#endif
