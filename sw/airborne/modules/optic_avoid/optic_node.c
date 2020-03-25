/**
 * @file "modules/optic_avoid/optic_node.c"
 * @author B. Keltjens & M. Gonzalez
 * Node for avoidance with optical flow
 */

#include "modules/optic_avoid/optic_node.h"

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef COLORFILTER_CAMERA
#define COLORFILTER_CAMERA front_camera
#endif

PRINT_CONFIG_VAR(OPENCVDEMO_FPS)


struct image_t *optic_avoid_func(struct image_t *img){

  // Check image is of the correct type
  if (img->type == IMAGE_YUV422) {

    // Begin clock
    clock_t optic_begin = clock();

    // call the optic avoid function
    optic_avoid((char *) img->buf, img->w, img->h);

    // End Clock and print time spend
    clock_t optic_end = clock();
    double optic_time_spent = (double)(optic_end - optic_begin) / CLOCKS_PER_SEC;
    // printf("Time for Optic Avoid: %lf \n", optic_time_spent);

    return img;
  }

  return NULL;
}


void optic_avoid_node_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, optic_avoid_func, OPENCVDEMO_FPS);
}
