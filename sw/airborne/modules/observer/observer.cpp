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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "observer.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/highgui/highgui_c.h>
#include "observer_lib.h"
// #include <iostream.h>
#include <iostream>

bool init_obs = false;

void observer(char *img, int width, int height) {

  std::cout << "Observeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeer" << std::endl;
  std::cout << *img << std::endl;

  // for(int i=0; i<sizeof(&img)/sizeof(&img[0]); i++){
  //   std::cout << &img[i] << std::endl;
  // }


  if (!init_obs) {
    initialize_observer();
  } else {

    // Create a new image, using the original bebop image.
    cv::Mat M(height, width, CV_8UC2, img);
    cv::Mat image;




  // Color image example
  // Convert the image to an OpenCV Mat
  cv::cvtColor(M, image, CV_YUV2BGR_Y422);
  // Blur it, because we can
  cv::blur(image, image, cv::Size(5, 5));
  // Convert back to YUV422 and put it in place of the original image
  colorbgr_opencv_to_yuv422(image, img, width, height);
  // #endif // OPENCVDEMO_GRAYSCALE
  }
}

void obs(struct image_t *img) {

  std::cout << "TTTTTTTTTTTTTTTTTTTTTTTTTTOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;

  for(int i=0; i<img->buf_size; i++) {
    std::cout << img->buf << std::cout;
  }
}

// Initialize observer functionality
void initialize_observer() {
  // cv::namedWindow("named_window", cv::WINDOW_AUTOSIZE);
  init_obs = true;
  // cv::destroyAllWindows();
}

// void colorfilter_f(struct image_t &img){

//   filter_color(img, dest_img);

//   // cvShowImage("image", &dest_img);


//   // uint8_t blur[3][3] = {{1/9.0, 1/9.0, 1/9.0},
//   //                       {1/9.0, 1/9.0, 1/9.0},
//   //                       {1/9.0, 1/9.0, 1/9.0}}; 

//   // cvMat M(3, 3, uint8_t, 1/9.0);
//   // cvMat a = cvMat(3, 3, uint8_t, 1);



//   // cvFilter2D(dest_img, -1, );

//   // cvWaitKey(0);
//   // cvReleaseImage(&dest_img);
//   // cvDestroyAllWindows();


//   // cvNamedWindow("image");
//   // cvShowImage("image", dest_img);

//   // cvWaitKey(0);
//   // cvReleaseImage(&dest_img);
//   // cvDestroyAllWindows();

// }


// void filter_color(struct image_t &input, 
//                              struct image_t &output){

//   uint16_t cnt = 0;
//   uint8_t temp = input.;
//   uint8_t *source = (uint8_t *)input->buf;
//   uint8_t *dest = (uint8_t *)output->buf;

//   // Copy the creation timestamp (stays the same)
//   output->ts = input->ts;

//   uint8_t n_cf = 2;
//   uint8_t color_filters[2][6] = {{1,2,3,4,5,6},
//                                 {1,2,3,4,5,6}};

//   // Go trough all the pixels
//   for (uint16_t y = 0; y < output->h; y++) {
//     for (uint16_t x = 0; x < output->w; x += 2) {
//       for (uint8_t cf_i = 0; cf_i < n_cf; cf_i++){
//         // Check if the color is inside the specified values
//         if (
//           (dest[1] >= color_filters[cf_i][0])
//           && (dest[1] <= color_filters[cf_i][1])
//           && (dest[0] >= color_filters[cf_i][2])
//           && (dest[0] <= color_filters[cf_i][3])
//           && (dest[2] >= color_filters[cf_i][4])
//           && (dest[2] <= color_filters[cf_i][5])
//         ) {
//           cnt ++;
//           // UYVY
//           dest[0] = 64;        // U
//           dest[1] = source[1];  // Y
//           dest[2] = 255;        // V
//           dest[3] = source[3];  // Y
//         } else {
//           // UYVY
//           char u = source[0] - 127;
//           u /= 4;
//           dest[0] = 127;        // U
//           dest[1] = source[1];  // Y
//           u = source[2] - 127;
//           u /= 4;
//           dest[2] = 127;        // V
//           dest[3] = source[3];  // Y
//         }

//         // Go to the next 2 pixels
//         dest += 4;
//         source += 4;
//       }
//     }
//   }
// }

// void observer_look() {
// }