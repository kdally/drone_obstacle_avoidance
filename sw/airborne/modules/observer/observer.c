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
 * @file "modules/observer/observer.c"
 * @author abarberia
 * First version of computer vision algorithm for MAVLAB course
 */

#include "modules/observer/observer.h"


#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef COLORFILTER_CAMERA
#define COLORFILTER_CAMERA front_camera
#endif


struct image_t *colorfilter_f(struct image_t *img){
  filter_color(img, img);


  // cvNamedWindow("image");
  // cvShowImage("image", dest_img);

  // cvWaitKey(0);
  // cvReleaseImage(&dest_img);
  // cvDestroyAllWindows();

  return img;
}


void observer_init() {
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_f, COLORFILTER_FPS);

}

void filter_color(struct image_t *input, 
                             struct image_t *output){

  uint16_t cnt = 0;
  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest = (uint8_t *)output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  uint8_t n_cf = 2;
  uint8_t color_filters[2][6] = {{1,2,3,4,5,6},
                                {1,2,3,4,5,6}};

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      for (uint8_t cf_i = 0; cf_i < n_cf; cf_i++){
        // Check if the color is inside the specified values
        if (
          (dest[1] >= color_filters[cf_i][0])
          && (dest[1] <= color_filters[cf_i][1])
          && (dest[0] >= color_filters[cf_i][2])
          && (dest[0] <= color_filters[cf_i][3])
          && (dest[2] >= color_filters[cf_i][4])
          && (dest[2] <= color_filters[cf_i][5])
        ) {
          cnt ++;
          // UYVY
          dest[0] = 64;        // U
          dest[1] = source[1];  // Y
          dest[2] = 255;        // V
          dest[3] = source[3];  // Y
        } else {
          // UYVY
          char u = source[0] - 127;
          u /= 4;
          dest[0] = 127;        // U
          dest[1] = source[1];  // Y
          u = source[2] - 127;
          u /= 4;
          dest[2] = 127;        // V
          dest[3] = source[3];  // Y
        }

        // Go to the next 2 pixels
        dest += 4;
        source += 4;
      }
    }
  }
}

// void observer_look() {
// }


