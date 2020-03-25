/**
 * @file "modules/optic_avoid/optic_avoid.cpp"
 * @author Group 11 2020
 * Module to use optical flow for collision avoidance
 */


#include "optic_avoid.h"
#include "modules/trajectory/trajectory.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <cmath>
#include <vector>

bool init_optic_avoid_ = false; // Information if the node is intialised
bool first_round_flag_ = true; // Check if this is the first iteration as there is no image history to calculate optical flow

// Declaration of Image Variables
cv::Mat im1_optic_avoid_low; // Previous Image gray and low resolution (always copied from current image in previous step)

cv::Mat im2_optic_avoid_grey; // Temporary gray image of current frame
cv::Mat im2_optic_avoid_low; // Low resolution image of current frame

// Flow Variables
cv::UMat flowUmat; // Temporary variable to store flow in UMat format
cv::Mat flow; // Flow Storage

// Find resolution factor changes
const int lower_width = (int)(240./OPTIC_RESOLUTION_STEP_DOWN);
const int lower_height = (int)(520./OPTIC_RESOLUTION_STEP_DOWN);


// Declare local std::vectors for analysing optic flow for headings
std::vector<float> sum_flow_row (lower_height, 0); // Vector to store values of the sum  of flow per row
std::vector<float> normalised_heading_flow (lower_height, 0); // Create std::vecotr of shape (1,lower_height) filled with zeros
std::vector<float> smoothed_normalised_heading_flow (lower_height, 0); // Create std::vecotr of shape (1,lower_height) filled with zeros

// Create global variable for heading safety
float optic_avoid_heading_information[lower_height] = {0}; // Initialise array with 0's
extern "C" float *GLOBAL_OF_VECTOR{optic_avoid_heading_information}; // External pointer to array with optic avoid collision danger information

void optic_avoid(char *img, int width, int height) {
  // Create New Image that the previous one has 'flowed' to with latest frame from bebop.
  cv::Mat M(height, width, CV_8UC2, img);

  // Convert img to opencv gray format
  cv::cvtColor(M, im2_optic_avoid_grey, CV_YUV2GRAY_Y422);

  // Lower the resolution of the image by a constant factor (better to keep this integer valued)
  cv::resize(im2_optic_avoid_grey, im2_optic_avoid_low, cv::Size(lower_width, lower_height), 0, 0, 1);


  // On first iteration set im1 to im2 as im1 does not have significant data yet
  if (first_round_flag_ == true){
    im2_optic_avoid_low.copyTo(im1_optic_avoid_low);
    // Set first round flag to false
    first_round_flag_ = false;
  }

  // Calculate Optical flow with Farneback method
  cv::calcOpticalFlowFarneback(im1_optic_avoid_low, im2_optic_avoid_low, flowUmat, 0.4, 1, 24, 5, 8, 1.2, 0);
  flowUmat.copyTo(flow); // Copy temp flow in UMat format to flow variable in Mat format


  // Sum up flow per row (in vertical direction in normal frame)
  float row_total = 0.0; // Temporary variable to hold row flow sum

  // Loop through every point in the flow field
  for (int y = 0; y < lower_height; y++) {
      for (int x = 0; x < (int)lower_width; x++)
      {
          const cv::Point2f flowatxy = flow.at<cv::Point2f>(y, x); // Extract flow information
          row_total += std::hypot(flowatxy.y,flowatxy.x); // Find flow vector norm
      }
      sum_flow_row[y] = row_total; // Set row flow sum
      row_total = 0.0; // Reset row sum
  }

  // Find maximum of sum_flow_row
  float max_flow_sum{0}; // Initialise maximum to zero
  for(int i = 0; i < lower_height; i++) {
    // If current value larger than current maximum replace maximum
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

  // Smoothing filter on normalised_heading_flow vector
  cv::GaussianBlur(normalised_heading_flow, smoothed_normalised_heading_flow, cv::Size(GB_WINDOW_PARAMETER, GB_WINDOW_PARAMETER), 0);



  // Write to external array after doing final thresholding
  for (int i = 0; i < lower_height; i++) {
    if(smoothed_normalised_heading_flow[i] > OPTIC_AVOID_GOTO_THRESHOLD){
      optic_avoid_heading_information[i] = smoothed_normalised_heading_flow[i];
    }
    else {
      optic_avoid_heading_information[i] = 0.0;
    }
  }

  // for (int i = 0; i < lower_height; i++) {
  //   std::cout << "i" << i << " :" << GLOBAL_OF_VECTOR[i] << std::endl;
  // }

  // Set previous image to current image for next iteration
  im2_optic_avoid_low.copyTo(im1_optic_avoid_low);
}

// Initialize optic functionality
void initialize_optic_avoid() {
  // cv::namedWindow("named_window", cv::WINDOW_AUTOSIZE);
  init_optic_avoid_ = true;
  // cv::destroyAllWindows();
}
