
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
void image_orangefilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M, uint8_t *mask);
void image_bgfilt(struct image_t *input, struct image_t *output, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M, bool color);
void convolve(struct image_t *input, struct image_t *output);
void image_convolve(struct image_t *input, struct image_t *output);
void find_poles(struct image_t *input);

// Init function
extern void observer_node_init(void);


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