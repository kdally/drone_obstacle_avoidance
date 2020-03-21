
#include "observer.h"
#include "observer.hpp"
#include "modules/computer_vision/lib/vision/image.h"

// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/highgui/highgui_c.h>
#include "observer_lib.h"
// #include <iostream.h>
#include <iostream>

bool init_obs = false;

void observer(char *img, int width, int height) {

  // Create a new image, using the original bebop image.
  std::cout << "Issue here" << std::endl;

  cv::Mat M(height, width, CV_8UC2, img);

  std::cout << "Yes" << std::endl;
  cv::Mat processed;
  cv::Mat mask;

  // filter_color(M, processed, mask);

  // Color image example
  // Convert the image to an OpenCV Mat
  // cv::cvtColor(M, processed, CV_YUV2BGR_Y422);
  // Blur it, because we can
  // cv::blur(image, image, cv::Size(5, 5));

  std::cout << "Issue here" << std::endl;
  // Convert back to YUV422 and put it in place of the original image
  // colorbgr_opencv_to_yuv422(processed, img, width, height);
  std::cout << "Yes" << std::endl;
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


// void filter_color(cv::Mat &input, cv::Mat &output, cv::Mat &msk){
 
//   for (uint8_t cf_i=0; cf_i < n_cf; cf_i++) {
//     std::cout << " YYYYYYYYYYYYYYYYYYYYYEEEEEEEEEEEEEEEEEEEEEEEE " << std::endl;
//     // for (int x=0; x<3; x++){
//     //   std::cout << color_filters[cf_i][0][x] << std::endl;
//     // }

//     cv::inRange(input, color_filters[cf_i][0], color_filters[cf_i][1], msk);
//     std::cout << " YYYYYYYYYYYYYYYYYYYYYEEEEEEEEEEEEEEEEEEEEEEEE " << std::endl;
//     cv::bitwise_and(input, input, output, msk);
//     std::cout << " YYYYYYYYYYYYYYYYYYYYYEEEEEEEEEEEEEEEEEEEEEEEE " << std::endl;
//   }
// }


















// void shitty_cf(){
//   uint16_t cnt = 0;
//   uint8_t temp = input;
//   uint8_t *source = (uint8_t *)input->buf;
//   uint8_t *dest = (uint8_t *)output->buf;

//   // Copy the creation timestamp (stays the same)
//   output->ts = input->ts;

//   Go trough all the pixels

//   for (uint16_t y = 0; y < input.rows; y++) {
//     for (uint16_t x = 0; x < input.cols; x ++) {
//       for (uint8_t cf_i = 0; cf_i < n_cf; cf_i++){
//         // Check if the color is inside the specified values
//         if (
//           (input.at(y, x) >= color_filters[cf_i][0])
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