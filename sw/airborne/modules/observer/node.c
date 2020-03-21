
#include "modules/observer/node.h"
#include <stdio.h>


#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef COLORFILTER_CAMERA
#define COLORFILTER_CAMERA front_camera
#endif


// orange filter            y_m  y_M  u_m  u_M  v_m  v_M
uint8_t orange_cf[6] = {26,  164,  74, 112, 173, 192};

// background filter
uint8_t n_cf = 3;
uint8_t bg_cf[3][6] = {{26,  164,  74, 112, 173, 192}, {70, 255, 128, 255,  0, 128},  // blue
                    {0,   255,   0, 110,   0, 130}}; // green
// uint8_t bg_cf[1][6] = {{26,  164,  74, 112, 173, 192}};

// kernel
uint8_t k_size = 3;
uint8_t kernel[3][3] = {{0, 1, 0},
                        {1, -4, 1},
                        {0, 1, 0}};
bool color;

struct image_t *observer_func(struct image_t *img){
  // uint8_t cf[3][6] = {{26,  164,  74, 112, 173, 192},  // orange
  //                     {BLUE_y_m, BLUE_y_M, BLUE_u_m, BLUE_u_M, BLUE_v_m, BLUE_v_M},  // blue?
  //                     {0,   255,   0, 110,   0, 130}}; // green
  // printf("The");
  // struct image_t *outimg;
  // printf("problem is:");
  // image_create(outimg, img->w, img->h, img->type);
  // printf("here\n");

  if (img->type == IMAGE_YUV422) {

    // Filter orange
    image_orangefilt(img, img, orange_cf[0], orange_cf[1], orange_cf[2],
                    orange_cf[3], orange_cf[4], orange_cf[5]);

    // printf("Image has: %d rows and %d cols", img->h, img->w);


    // convolve(img, img);

    // find_poles(img);    

    // Remove floor (blue and greem)
    // for (uint8_t cf_i = 0; cf_i < n_cf; cf_i++) {
    //   color = (n_cf-1) - cf_i;
    //   // color = true;

    //   image_bgfilt(img, img, bg_cf[cf_i][0], bg_cf[cf_i][1], bg_cf[cf_i][2], 
    //                   bg_cf[cf_i][3], bg_cf[cf_i][4], bg_cf[cf_i][5], color);
    // }

    // Convolve the image
    // convolve

    // Overwrite original image
    // img = outimg;

    // call the C++ interface
    // observer((char *) img->buf, img->w, img->h);

  }
  return img;
}


// Filter orange color and place black on the rest
void image_orangefilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M) {

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest = (uint8_t *)output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values
      if ( (source[1] >= y_m)
        && (source[1] <= y_M)
        && (source[0] >= u_m)
        && (source[0] <= u_M)
        && (source[2] >= v_m)
        && (source[2] <= v_M)
      ) {
        dest[0] = 128;  // U
        dest[1] = 255;  // Y
        dest[2] = 128;  // V
        dest[3] = 255;  // Y
      } else {
        // UYVY
        dest[0] = 128;  // U
        dest[1] = 0;    // Y
        dest[2] = 128;  // V
        dest[3] = 0;    // Y
      }
      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
}

// Filter background color and place grayscale on the rest
void image_bgfilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M, bool color) {

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest = (uint8_t *)output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values
      if ( (source[1] >= y_m)
        && (source[1] <= y_M)
        && (source[0] >= u_m)
        && (source[0] <= u_M)
        && (source[2] >= v_m) 
        && (source[2] <= v_M)
      ) {
        dest[0] = 128;          // U
        dest[1] = 255;          // Y
        dest[2] = 128;          // V
        dest[3] = 255;          // Y
      } else {
        if (color) {
          // UYVY
          dest[0] = source[0];  // U
          dest[1] = source[1];  // Y
          dest[2] = source[2];  // V
          dest[3] = source[3];  // Y
        } else {
          dest[0] = 128;        // U
          dest[1] = source[1];  // Y
          dest[2] = 128;        // V
          dest[3] = source[3];  // Y
        }
      }
      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
}

// Trye number 1 to convolve image
void convolve(struct image_t *input, struct image_t *output){


  uint8_t *source = ((uint8_t *)input->buf)+4;
  uint8_t *dest = ((uint8_t *)output->buf)+4;

  // for (uint16_t y = k_size/2; y < (input->h-k_size/2); y++) {
  //   for (uint16_t x = k_size/2; x < (input->w-k_size/2); x += 2) {
  //     // uint8_t val = 0;
  //     // for (uint8_t k_i = 0; k_i < k_size; k_i++) {
  //     //   val += kernel[k_i]
  //     // }

  //     dest[1] = (source-4)[1] + (source+4)[1] + (source-4*input->w)[1] + (source+4*input->w)[1] - 4*(source[1]);
  //     dest[3] = (source-4)[3] + (source+4)[3] + (source-4*input->w)[3] + (source+4*input->w)[3] - 4*(source[3]);

  //     dest += 4;
  //     source += 4;
  //   }
  // }
  for (uint16_t y = 0; y < input->h; y++) {
    for (uint16_t x = 1; x < input->w-1; x += 2) {
      // uint8_t val = 0;
      // for (uint8_t k_i = 0; k_i < k_size; k_i++) {
      //   val += kernel[k_i]
      // }
      printf("issue");
      // dest[1] = 255*y/(input->h);

      // bool condition = ((dest-4)[1] != (dest+4)[1]);
      bool condition = true;

      if (condition) {
        dest[1] = 255;
        dest[3] = 255;
      } else {
        dest[1] = 0;
        dest[3] = 0;
      }

      printf("here\n");
      // dest[1] = ((source-4)[1] - (source+4)[1]) * ((source-4)[1] - (source+4)[1]);
      // dest[3] = ((source-4)[3] - (source+4)[3]) * ((source-4)[3] - (source+4)[3]);

      // dest[1] = abs((source-4)[1] - (source+4)[1]);
      // dest[3] = abs((source-4)[3] - (source+4)[3]);

      dest += 4;
      source += 4;
    }
    dest += 8;
    source += 8;
  }
}

void image_convolve(struct image_t *input, struct image_t *output) {

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest = (uint8_t *)output->buf;

  // bool turned_black = false;
  // bool turned_white = false;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go trough all the pixels
  // for (uint16_t y = 0; y < output->h; y++) {
  //   for (uint16_t x = 0; x < output->w; x += 2) {
  //     // Check if the color is inside the specified values
  //     if (x!=0) {
  //       if ((dest[1] >= 100)
  //       ) {
  //         // if pixel was already white
  //         if (turned_white) {
  //           dest[1] = 0;
  //           dest[3] = 0;
  //         } else { // pixel just turned white from black
  //           turned_white = true;
  //           turned_black = false;
  //         }
  //         // dest[0] = 128;          // U
  //         // dest[1] = 255;          // Y
  //         // dest[2] = 128;          // V
  //         // dest[3] = 255;          // Y
  //       } else {
  //         if (turned_black) { // if was on black do nothing
  //         } else {            // if it comes from white
  //           turned_black = true;
  //           turned_white = false;
  //         }
  //       }
  //       // Go to the next 2 pixels
  //       dest += 4;
  //       source += 4;
  //     }
  //   }
  // }

  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // Check if the color is inside the specified values
      if (x!=0) {
        if ((dest[1] != (dest-4*(output->w))[1])
        ) {
          dest[1] = 255;
          dest[3] = 255;
        } else { // pixel just turned white from black
          dest[1] = 0;
          dest[3] = 0;
        }
        // Go to the next 2 pixels
        dest += 4;
        source += 4;
      }
    }
  }
}

void find_poles(struct image_t *input){

  uint8_t *source = (uint8_t *)input->buf;

  for (uint16_t y = 0; y < input->h; y++){
    for (uint16_t x = 0; x < input->w; x++){

      // If pixel is white
      if ( (source[1] >= 100)){

      } else {

      }
    }
  }
}

void observer_node_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, observer_func, OPENCVDEMO_FPS);
}