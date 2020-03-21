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


#include "optic_avoid.h"
// #include "modules/trajectory/trajectory.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/imgproc.hpp> //#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp> //#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <cmath>
#include <vector>

bool init_optic_avoid_ = false;
bool first_round_flag_ = true; // Check if this is the first iteration as there is no image history to calculate optical flow

// Declaration of Image Variables
cv::Mat im1_optic_avoid_low; // Previous Image gray and low resolution (always copied from current image in previous step)

cv::Mat im2_optic_avoid_grey; // Temporary gray image of current frame
cv::Mat im2_optic_avoid_low; // Low resolution image of current frame

// Flow Variables
cv::UMat flowUmat; // Temporary variable to store flow
cv::Mat flow; // Flow Storage

// Find resolution factor changes
const float resolution_step_down = 3.0; // TODO: This needs to be changed to a define setting in xml file
const int lower_width = (int)(240./resolution_step_down);
const int lower_height = (int)(520./resolution_step_down);
// Blurring
const int GB_parameter = 33;
const float goTo_threshold = 0.1;


// Create global variable for heading safety
std::vector<float> normalised_heading_flow (lower_height, 0); // Create std::vecotr of shape (1,lower_height) filled with zeros
std::vector<float> smoothed_normalised_heading_flow (lower_height, 0); // Create std::vecotr of shape (1,lower_height) filled with zeros

float optic_avoid_heading_information[lower_height] = {0};
extern "C" float *GLOBAL_OF_VECTOR{optic_avoid_heading_information};

void optic_avoid(char *img, int width, int height) {
  // float start = cv::getTickCount();

  // Create New Image that the previous one has 'flowed' to.
  cv::Mat M(height, width, CV_8UC2, img);

  // Convert to opencv gray format
  cv::cvtColor(M, im2_optic_avoid_grey, CV_YUV2GRAY_Y422);

  // Lower the resolution of the image by a factor (better to keep this integer valued)
  cv::resize(im2_optic_avoid_grey, im2_optic_avoid_low, cv::Size((int)lower_width, lower_height), 0, 0, 1);



  // On first iteration set im1 to im2
  if (first_round_flag_ == true){
    im2_optic_avoid_low.copyTo(im1_optic_avoid_low);
    // Set flag to false
    first_round_flag_ = false;
  }

  // Calculate Optical flow with Farneback method
  cv::calcOpticalFlowFarneback(im1_optic_avoid_low, im2_optic_avoid_low, flowUmat, 0.4, 1, 24, 5, 8, 1.2, 0);
  flowUmat.copyTo(flow); // Copy temp flow to flow variable


  // Sum up flow per row (in vertical direction in normal frame)
  std::vector<float> sum_flow_row (lower_height, 0); // Vector to store values of the flow
  float row_total = 0.0; // Temporary variable to hold row flow sum

  // Loop through every point in the flow field
  // TODO: Try get the flowatxy declared outside of the loop
  for (int y = 0; y < lower_height; y++) {
      for (int x = 0; x < (int)lower_width; x++)
      {
          const cv::Point2f flowatxy = flow.at<cv::Point2f>(y, x); // Extract flow information
          row_total += std::hypot(flowatxy.y,flowatxy.x); // Find flow vector norm
      }
      sum_flow_row[y] = row_total; // Set row flow sum
      row_total = 0.0; // Reset row ounter
  }

  // Find maximum of sum_flow_row
  float max_flow_sum{0}; // Initialise maximum to zero
  for(int i = 0; i < lower_height; i++) {
    if (sum_flow_row[i] > max_flow_sum)
    {
        max_flow_sum = sum_flow_row[i];
    }
  }

  // Find minimum of sum_flow_row
  float min_flow_sum{max_flow_sum}; // Initialise minimum to maximum
  for(int i = 0; i < lower_height; i++) {
    if (sum_flow_row[i] < min_flow_sum)
    {
        min_flow_sum = sum_flow_row[i];
    }
  }

  // Create Normalised heading flow vector
  for(int i = 0; i < lower_height; i++) {
    normalised_heading_flow[i] = ((sum_flow_row[i]-min_flow_sum)/(max_flow_sum-min_flow_sum)); // Become percentage from 0 to 1
  }

  // Smoothing filter
  cv::GaussianBlur(normalised_heading_flow, smoothed_normalised_heading_flow, cv::Size(GB_parameter, GB_parameter), 0);



  // Write to external array after doing final thresholding
  for (int i = 0; i < lower_height; i++) {
    if(smoothed_normalised_heading_flow[i] > goTo_threshold){
      optic_avoid_heading_information[i] = smoothed_normalised_heading_flow[i];
    }
    else {
      optic_avoid_heading_information[i] = 0.0;
    }
  }

  for (int i = 0; i < lower_height; i++) {
    std::cout << "i" << i << " :" << GLOBAL_OF_VECTOR[i] << std::endl;
  }



  // Set previous image to current image for next iteration
  im2_optic_avoid_low.copyTo(im1_optic_avoid_low);

  // float time_taken = (cv::getTickCount() - start)/cv::getTickFrequency();
  // std::cout << "Time taken: " << time_taken*1000 << " [ms]" << std::endl;
}

// Initialize optic functionality
void initialize_optic_avoid() {
  // cv::namedWindow("named_window", cv::WINDOW_AUTOSIZE);
  init_optic_avoid_ = true;
  // cv::destroyAllWindows();
}
