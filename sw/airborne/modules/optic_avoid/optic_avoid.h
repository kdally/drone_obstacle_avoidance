/**
 * @file "modules/optic_avoid/optic_avoid.h"
 * @author Group 11 2020
 * Header File for optic avoid cpp file
 */


#ifndef OPTIC_AVOID_H
#define OPTIC_AVOID_H

#ifdef __cplusplus
extern "C" {
#endif

/**
   * @brief function
   * @param img: pointer to image
   * @param width: the width of the image
   * @param height: the height of the image
   * @return: void.
*/
void optic_avoid(char *img, int width, int height);

/**
   * @brief Initialises the optic avoid function
   * @return: void.
*/
void initialize_optic_avoid();



#ifdef __cplusplus
}
#endif

#endif
