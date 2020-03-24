/**
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#ifndef OPTIC_AVOID_H
#define OPTIC_AVOID_H

#ifdef __cplusplus
extern "C" {
#endif
// Define stuff C++ style?

void optic_avoid(char *img, int width, int height);

void initialize_optic_avoid();

// struct image_t dest_img;




#ifdef __cplusplus
}
#endif

#endif
