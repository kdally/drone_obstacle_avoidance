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
 * @file "modules/optic_avoid/optic_node.c"
 * @author Team Wonder
 * Node for avoidance with optical flow
 */

#include "modules/optic_avoid/optic_node.h"

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef COLORFILTER_CAMERA
#define COLORFILTER_CAMERA front_camera
#endif

PRINT_CONFIG_VAR(OPENCVDEMO_FPS)


struct image_t *optic_avoid_func(struct image_t *img){

  // Check image is of the correct type
  if (img->type == IMAGE_YUV422) {

    // Begin clock
    // clock_t optic_begin = clock();

    // call the optic avoid function
    optic_avoid((char *) img->buf, img->w, img->h);

    // End Clock and print time spend
    // clock_t optic_end = clock();
    // double optic_time_spent = (double)(optic_end - optic_begin) / CLOCKS_PER_SEC;
    // printf("Time for Optic Avoid: %lf \n", optic_time_spent);

    return img;
  }

  return NULL;
}


void optic_avoid_node_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, optic_avoid_func, OPENCVDEMO_FPS);
}
