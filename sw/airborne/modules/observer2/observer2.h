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

#include <stdint.h>
#include <stdio.h>

// #include <opencv2/imgproc/imgproc_c.h>
// #include <opencv2/core/core_c.h>
// #include <opencv2/highgui/highgui_c.h>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "modules/computer_vision/cv.h"


// #include <opencv/cv.h>

// #include <opencv2/highgui/highgui_c.h>
// #include ""

// #include "opencv/highgui.h"
// #include <opencv2/core/core_c.h>
// #include "opencv2/highgui/highgui_c.h"
// #include <opencv2/highgui/highgui_c.h>
// #include "subsystems/abi.h"
// #include "modules/computer_vision/opencv_image_functions.h"
// #include "modules/computer_vision/opticflow/"
// #include "modules/computer_vision/"
// #include "modules/computer_vision/lib/vision/image.h"



extern void observer_init(void);
// extern void observer_look();

struct image_t *colorfilter_f(struct image_t *img);


void filter_color(struct image_t *input, 
                             struct image_t *output);


struct video_listener *listener;
struct image_t *dest_img;


#endif

