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
 * @file "modules/TRAJECTORY/TRAJECTORY.h"
 * @author Team Wonder
 * Just pathing in TRAJECTORY
 */

#include "modules/computer_vision/cv.h"
#include "state.h"
#include <stdio.h>
#include <time.h>

#ifndef OBSERVER_NODE_H
#define OBSERVER_NODE_H


// Main function

/**
   * @brief Image processing service that detects object from color filters and 
   *        convolution-detected edges.
   * @return: void
*/
struct image_t *observer_func(struct image_t *img);

// Image processing functions
/**
   * @brief read the drone's state
   * @return: void.
*/
void read_drone_state(void);

/**
   * @brief define the shape of the output image to match that of the input
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @return: void.
*/
void create_img(struct image_t *input, struct image_t *output);

/**
   * @brief define the shape of the reduced output image
   * @param output: pointer to output image
   * @return: void.
*/
void create_small_img(struct image_t *output);

/**
   * @brief decrease image resolution of input image to 'new_h' and 'new_w'
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @return: void.
*/
void downsize_img(struct image_t *input, struct image_t *output);

/**
   * @brief copy content of input image to output image
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @return: void.
*/
void copy2img(struct image_t *input, struct image_t *output);

/**
   * @brief filter specific color on input image, returning a colored output
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @param y_m: lower bound for the color filter's Y value
   * @param y_M: upper bound for the color filter's Y value
   * @param u_m: lower bound for the color filter's U value
   * @param u_M: upper bound for the color filter's U value
   * @param v_m: lower bound for the color filter's V value
   * @param v_M: upper bound for the color filter's V value
   * @param mask: pointer to output list used as mask (1's for pixels matching 
   *              the filtered color, 0's for the rest)
   * @return: void.
*/
void image_specfilt(struct image_t *input, struct image_t *output,
                      uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M,
                      uint8_t v_m, uint8_t v_M, uint8_t *mask);

/**
   * @brief filter specific color on input image, returning a grayscale output
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @param y_m: lower bound for the color filter's Y value
   * @param y_M: upper bound for the color filter's Y value
   * @param u_m: lower bound for the color filter's U value
   * @param u_M: upper bound for the color filter's U value
   * @param v_m: lower bound for the color filter's V value
   * @param v_M: upper bound for the color filter's V value
   * @return: void.
*/
void image_bgfilt(struct image_t *input, struct image_t *output,
                  uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M,
                  uint8_t v_m, uint8_t v_M);

/**
   * @brief convolve the input image with 'big_kernel' to output image
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @return: void.
*/
void convolve_big(struct image_t *input, struct image_t *output);

/**
   * @brief blur the input image with to output image by averaging over a 
   *        kernel shaped with 'blur_w' and 'blur_h'
   * @param input: pointer to input image
   * @param output: pointer to output image
   * @return: void.
*/
void blur_big(struct image_t *input, struct image_t *output);

/**
   * @brief identify orange poles from orange mask
   * @return: void.
*/
void find_orange_objs(void);

/**
   * @brief identify objects from green mask obtained from discontinuities on 
   *        the horizon line
   * @return: void
*/
void find_green_objs(void);

/**
   * @brief identify objects from edge mask. When pointing at the black curtain, 
   *        considers darker pixels background. When pointing at the 
   *        curtain-less side, considers continuous edges as background. When 
   *        pointing in between, do mixed logic
   * @param input: pointer to input image
   * @return: void
*/
void find_edge_objs(struct image_t *input);

/**
   * @brief identify objects between input pixels. Where an edge is detected, 
   *        consider the brighter pixels as object
   * @param idx1: starting pixel where curtain is considered
   * @param idx2: finishing pixel where curtain is considered
   * @param input: pointer to input image
   * @return: void
*/
void logic_curtain(uint16_t idx1, uint16_t idx2, struct image_t *input);

/**
   * @brief identify objects between input pixels. Where a notable edge is 
   *        detected, consider the next edge-free pixels as an object
   * @param idx1: starting pixel where curtain is considered
   * @param idx2: finishing pixel where curtain is considered
   * @return: void
*/
void logic_free(uint16_t idx1, uint16_t idx2);

/**
   * @brief Combine objects detected by 'find_orange_objects', 'find_green_objs'
   *        and 'find_edge_objs', and associate those objects that have been 
   *        detected by different algorithms as the same object
   * @return: void
*/
void combine_measurements(void);

/**
   * @brief Ensure continuous object detection by keeping track of objects 
   *        detected in the past. Only presents an object as such when a 
   *        measurement of an object has been recorded multiple times (i.e. 
   *        gets rid of random noisy measurements)
   * @return: void
*/
void delete_outliers(void);

/**
   * @brief Estimates distance to object based on size of intrusion of object on
   *        the green mask for all objects, and based on pixel width for orange 
   *        poles (detected from 'find_orange_objs')
   * @return: void
*/
void find_distances(void);

/**
   * @brief Map the coordinates of the detected objects to those of the original 
   *        image
   * @return: void
*/
void remap_to_original_img(void);

//
extern float final_objs[50][4];

/**
   * @brief Initialise the 'observer_func' service
   * @return: void
*/
extern void observer_node_init(void);


#endif