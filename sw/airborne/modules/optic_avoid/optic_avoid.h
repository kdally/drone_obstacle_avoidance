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
 * @file "modules/optic_avoid/optic_avoid.h"
 * @author Team Wonder
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
