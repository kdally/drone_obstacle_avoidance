
#include "modules/computer_vision/cv.h"
#include "modules/observer/observer.h"
// #include <stdio.h>

#ifndef OBSERVER_NODE_H
#define OBSERVER_NODE_H


////////////////////////////////////////////////////////////////////////////////
// DEFINE FUNCTIONS

// Main function
struct image_t *observer_func(struct image_t *img);

// Image processing functions
void create_img(struct image_t *input, struct image_t *output);
void copy2img(struct image_t *input, struct image_t *output);
void image_specfilt(struct image_t *input, struct image_t *output1, 
                      struct image_t *output2, uint8_t y_m, 
                      uint8_t y_M, uint8_t u_m, uint8_t u_M, 
                      uint8_t v_m, uint8_t v_M, uint8_t *mask);
void image_bgfilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                  uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                  uint8_t v_M, bool color);
void convolve(struct image_t *input, struct image_t *output);
void convolve_big(struct image_t *input, struct image_t *output);
void blur_image(struct image_t *input, struct image_t *output);
void blur_big(struct image_t *input, struct image_t *output);
void blur_mask(struct image_t *input, uint8_t *mask);
void detect_poles(uint16_t *poles);
void find_orange_objs();
void find_green_objs();  
void find_rand_objs();
void combine_measurements();
void delete_outliers();
void find_distances();

extern uint16_t final_objs[100][3];

// Init function
extern void observer_node_init(void);


extern uint16_t poles[100][3];

////////////////////////////////////////////////////////////////////////////////
// DEFINE PARAMETERS

// Output image
// struct image_t *outimg;

// // Color filters
// const int n_cf = 2;
// const int n_cols = 6;
//                     // y_m  y_M  u_m  u_M  v_m  v_M 
// const extern uint8_t cf[n_cf][n_cols] = {{0,   255, 0,   255, 0,   255},
//                                   {100, 255, 100, 255, 100, 255}};

// Other shit


#endif