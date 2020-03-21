/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
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

  if (img->type == IMAGE_YUV422) {

    // call the C++ interface
    optic_avoid((char *) img->buf, img->w, img->h);

    // obs(img);
    return img;
  }

// opencv_example(NULL, 10,10);

  return NULL;
}

void optic_avoid_node_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, optic_avoid_func, OPENCVDEMO_FPS);
}
