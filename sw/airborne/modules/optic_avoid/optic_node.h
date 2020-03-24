/**
 * @file "modules/optic_avoid/optic_node.h"
 * @author M. Gonzalez & B. Keltjens
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
