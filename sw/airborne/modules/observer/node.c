
#include "modules/observer/node.h"
#include <stdio.h>
#include <time.h>


#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef COLORFILTER_CAMERA
#define COLORFILTER_CAMERA front_camera
#endif


// orange filter        y_m  y_M  u_m  u_M  v_m  v_M
uint8_t orange_cf[6] = {26, 164,  45, 130, 160, 192};
uint8_t green_cf[6]  = {70, 230,   0, 120,   0, 125};

// orange mask
uint8_t mask_o[520][240];

// green mask
uint8_t mask_g[520][240];

// uint8_t masks[2][520][240]; // 0: orange, 1: green

// background filter
uint8_t n_cf = 2;
uint8_t bg_cf[2][6] = {{67, 255, 120, 255,  0, 125},  // blue
                    {70,   255,   0, 120,   0, 125}}; // green
// uint8_t bg_cf[1][6] = {{70,   255,   0, 130,   0, 130}}; // green

// kernel
uint8_t k_size = 3;
uint8_t kernel[3][3] = {{0, 1, 0},
                        {1, -4, 1},
                        {0, 1, 0}};
bool color;

// detected poles (max 100 at a time) -> (left_px, right_px, distance)
uint16_t poles[100][3];
//uint16_t *GLOBAL_OBJECTS_VECTOR = poles;

struct image_t processed;
struct image_t orange_mask;
struct image_t convoluted;



// MAIN FUNCTION
struct image_t *observer_func(struct image_t *img){

  if (img->type == IMAGE_YUV422) {

    // Take the time
    clock_t begin = clock();

    // Copy input image to processed
    create_img(img, &processed);
    create_img(img, &orange_mask);
    create_img(img, &convoluted);

    // Clean poles
    for (uint16_t x = 0; x < 100; x++) {
      poles[x][0] = 0;
      poles[x][1] = 0;
      poles[x][2] = 0;
    }

    // Filter poles (orange)
    image_orangefilt(img, &processed, &orange_mask, orange_cf[0], orange_cf[1], orange_cf[2],
                    orange_cf[3], orange_cf[4], orange_cf[5], &mask_o[0]);

    // convolve(&orange_mask, &convoluted);

    // detect_poles(&poles);

    look4poles(1);

    // // Filter ground (green)
    // image_orangefilt(img, &processed, &orange_mask, green_cf[0], green_cf[1], green_cf[2],
    //                 green_cf[3], green_cf[4], green_cf[5], &mask_o);

    // convolve(&orange_mask, &convoluted);

    // look4polesg(&orange_mask);

    // // This is just to show the mask
    // for (uint16_t x=0; x<520; x++){
    //   for (uint16_t y=0; y<240; y++){
    //     printf("%d",mask[x][y]);
    //   }
    //   printf("\n");
    // }
    // printf("\n");

    // Remove floor (blue and greem)
    // for (uint8_t cf_i = 0; cf_i < n_cf; cf_i++) {
    //   color = (n_cf-1) - cf_i; // if last in the list, turn to grey
    //   // color = true;

    //   image_bgfilt(img, &processed, bg_cf[cf_i][0], bg_cf[cf_i][1], bg_cf[cf_i][2], 
    //                   bg_cf[cf_i][3], bg_cf[cf_i][4], bg_cf[cf_i][5], color);
    // }
    // convolve(&processed, &convoluted);


  clock_t end = clock();
  double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
  //printf("Time: %lf \n", time_spent);
  }
  return &processed;
  // return img;
}

void create_img(struct image_t *input, struct image_t *output){
  // Set the variables
  output->type = input->type;
  output->w = input->w;
  output->h = input->h;

  // Depending on the type the size differs
  if (output->type == IMAGE_YUV422) {
    output->buf_size = sizeof(uint8_t) * 2 * input->w * input->h;
  } else if (output->type == IMAGE_JPEG) {
    output->buf_size = sizeof(uint8_t) * 2 * input->w * input->h;  // At maximum quality this is enough
  } else if (output->type == IMAGE_GRADIENT) {
    output->buf_size = sizeof(int16_t) * input->w * input->h;
  } else {
    output->buf_size = sizeof(uint8_t) * input->w * input->h;
  }

  output->buf = malloc(input->buf_size);
}


// Filter orange color and place black on the rest
void image_orangefilt(struct image_t *input, struct image_t *output1, struct image_t *output2, uint8_t y_m, 
                     uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, 
                     uint8_t v_M, uint8_t *mask) {

  uint8_t *source = (uint8_t *)input->buf;
  uint8_t *dest1 = (uint8_t *)output1->buf;
  uint8_t *dest2 = (uint8_t *)output2->buf;

  // Copy the creation timestamp (stays the same)
  output1->ts = input->ts;
  output2->ts = input->ts;

  // Go trough all the pixels
  for (uint16_t y = 0; y < input->h; y++) { // horizontal (for every column) 520
    for (uint16_t x = 0; x < input->w; x += 2) { // vertical (for every pixel in every column) 240
      // Check if the color is inside the specified values
      if ( (source[1] >= y_m)
        && (source[1] <= y_M)
        && (source[0] >= u_m)
        && (source[0] <= u_M)
        && (source[2] >= v_m)
        && (source[2] <= v_M)
      ) {
        dest1[0] = 128;  // U
        dest1[1] = 255;  // Y
        dest1[2] = 128;  // V
        dest1[3] = 255;  // Y

        dest2[0] = 128;
        dest2[1] = 255;
        dest2[2] = 128;
        dest2[3] = 255;

        *(mask+x+y*(output1->w)) = 1;
        *(mask+x+1+y*(output1->w)) = 1;
      } else {
        // UYVY
        // dest[0] = 128;  // U // Black
        // dest[1] = 0;    // Y
        // dest[2] = 128;  // V
        // dest[3] = 0;    // Y
        // dest[0] = 128;       // U // Grayscale
        // dest[1] = source[1]; // Y
        // dest[2] = 128;       // V
        // dest[3] = source[3]; // Y
        dest1[0] = source[0];  // U // Color
        dest1[1] = source[1];  // Y
        dest1[2] = source[2];  // V
        dest1[3] = source[3];  // Y

        dest2[0] = 128;
        dest2[1] = 0;
        dest2[2] = 128;
        dest2[3] = 0;

        *(mask+x+y*(output1->w)) = 0;
        *(mask+x+1+y*(output1->w)) = 0;
      }
      // Go to the next 2 pixels
      dest1 += 4;
      dest2 += 4;
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
          dest[1] = source[1]*2;  // Y
          dest[2] = 128;        // V
          dest[3] = source[3]*2;  // Y
        }
      }
      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
}

void detect_poles(uint16_t *poles){

}

// Try number 1 to convolve image
void convolve(struct image_t *input, struct image_t *output){

  uint8_t *source = ((uint8_t *)input->buf)+(4*input->w);
  uint8_t *dest = ((uint8_t *)output->buf)+(4*output->w);

  int16_t max_k1 = 0;
  int16_t min_k1 = 32000;
  int16_t max_k3 = 0;
  int16_t min_k3 = 32000;

  for (uint16_t y = 1; y < (input->h)-3; y++) { // horizontal (for every column) 520
    dest += 4;
    source += 4;
    for (uint16_t x = 1; x < (input->w)-3; x += 2) { // vertical (for every pixel in every column) 240

      // for (uint8_t k_y = 0; k_y < k_size; k_y++) {
      //   for (uint8_t k_x = 0; k_x < k_size; k_x++) {
      //     // idx = source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w)
      //     val1 += kernel[k_x][k_y] * (source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w))[1];
      //     val2 += kernel[k_x][k_y] * (source + (k_x-k_size/2)*4 + (k_y-k_size/2)*4*(output->w))[3];
      //   }
      // }
      // dest[1] = val1;
      // dest[3] = val2;

      // val1 = (source-(4*input->w))[1] + (source+(4*input->w))[1] - 2*(source[1]);
      // val3 = (source-(4*input->w))[3] + (source+(4*input->w))[3] - 4*(source[3]);


      dest[0] = 128;
      dest[1] = abs((source-4)[1] + (source+4)[1] + (source-(4*input->w))[1] + (source+(4*input->w))[1] - 4*(source[1]));
      dest[2] = 128;
      dest[3] = abs((source-4)[3] + (source+4)[3] + (source-(4*input->w))[3] + (source+(4*input->w))[3] - 4*(source[3]));

      if (dest[1] > max_k1) {
        max_k1 = dest[1];
      } if(dest[1] < min_k1) {
        min_k1 = dest[1];
      } if (dest[3] > max_k3) {
        max_k3 = dest[3];
      } if (dest[3] < min_k3) {
        min_k3 = dest[3];
      }


      dest += 4;
      source += 4;
    }
    // printf("\n");
    dest += 4;
    source += 4;
  }

  source = ((uint8_t *)input->buf)+(4*input->w);
  dest = ((uint8_t *)output->buf)+(4*output->w);

  for (uint16_t y = 1; y < (input->h)-3; y++) { // horizontal (for every column) 520
    dest += 4;
    source += 4;
    for (uint16_t x = 1; x < (input->w)-3; x += 2) { // vertical (for every pixel in every column) 240
      source[1] = (dest[1]-min_k1) * 255 / (max_k1-min_k1+1);
      source[3] = (dest[3]-min_k3) * 255 / (max_k3-min_k3+1);

      // printf("%d %d %d\t", dest[0], dest[2], dest[1]);
      dest += 4;
      source += 4;
    }
    // printf("\n");
    dest += 4;
    source += 4;
  }
  // printf("\n");



  // for (uint16_t y = 1; y < (input->h)-1; y++) {
  //   for (uint16_t x = 1; x < (input->w)-1; x += 2) {
  //     // uint8_t val = 0;
  //     // for (uint8_t k_i = 0; k_i < k_size; k_i++) {
  //     //   val += kernel[k_i]
  //     // }
  //     printf("issue");
  //     // dest[1] = 255*y/(input->h);

  //     // bool condition = ((dest-4)[1] != (dest+4)[1]);
  //     bool condition = true;

  //     if (condition) {
  //       dest[1] = 255;
  //       dest[3] = 255;
  //     } else {
  //       dest[1] = 0;
  //       dest[3] = 0;
  //     }

  //     printf("here\n");
  //     // dest[1] = ((source-4)[1] - (source+4)[1]) * ((source-4)[1] - (source+4)[1]);
  //     // dest[3] = ((source-4)[3] - (source+4)[3]) * ((source-4)[3] - (source+4)[3]);

  //     // dest[1] = abs((source-4)[1] - (source+4)[1]);
  //     // dest[3] = abs((source-4)[3] - (source+4)[3]);

  //     dest += 4;
  //     source += 4;
  //   }
  //   dest += 8;
  //   source += 8;
  // }
}


void look4poles(uint8_t idx){

  uint16_t sums[520]; // cumulative sum

  // Get number of appearances
  for (uint16_t x = 0; x < 520; x++){
    sums[x] = 0;
    for (uint16_t y = 0; y < 240; y++) {
      sums[x] += mask_o[x][y];
    }
  }

  // Get derivative in number of appearances
  int8_t der1 = 0;
  int16_t count = 0;

  for (uint16_t x = 1; x < 519; x++){

    // LOGIC WITH THRESHOLD ON 1st DERIVATIVE
    der1 = sums[x+1] - sums[x-1];

    if (abs(der1) > 34) {
      if (der1 > 0) {
        poles[count][0] = x;
      } else {
        poles[count][1] = x;
        count++;
      }
      x += 8;
    }
  }

    // LOGIC WITH 2nd DERIVATIVE
        // der1 = sums[x+1] - sums[x-1];

        // if (abs(der1) > 15){    // if there are more than X 1's difference
        //   der2new = sums[x+1] + sums[x-1] - 2*sums[x];

        //   // if 2nd derivative have opposite sign, we have a pole!
        //   if (((der2new < 0) && (der2old > 0)) || ((der2new > 0) && (der2old < 0))){
    
        //     if (der1 > 0){ // if the pole begins
        //       poles[count][0] = x;
        //     } else {       // if the pole ends
        //       poles[count][1] = x;
        //       count ++;
        //     }

        //   } else {
        //     der2old = der2new;
        //   }
        // } else {
        //   der2old = 0;
        //   der2new = 0;
        // }


  // for (uint16_t x = 0; x < 10; x++){
  //   printf("[%d, %d] \n", poles[x][0], poles[x][1]);
  // }
  // printf("\n");
}


void look4polesg(struct image_t *input){

  uint16_t floors[520]; // cumulative sum

  uint8_t *source = (uint8_t *)input->buf;
  // Go trough all the pixels
  for (uint16_t y = 0; y < input->h; y++) { // 520
    for (uint16_t x = 0; x < input->w; x += 2) { // 240

      if ((source + input->w - 2*x)[1] == 255){
        floors[y] = (input->w - x);

        source += 4*(input->w-x);
        break;
      } else {
        source+= 4;
      }
    }
  }


  // for (uint16_t x = 0; x < 520; x++){
  //   printf("%d, ", floors[x]);
  // }
  // printf("\n");


  // Get derivative in number of appearances
  int8_t der1 = 0;
  int16_t count = 99;

  for (uint16_t x = 1; x < 519; x++){

    // LOGIC WITH THRESHOLD ON 1st DERIVATIVE
    der1 = floors[x+1] - floors[x-1];

    if (abs(der1) > 10) {
      if (der1 > 0) {
        poles[count][0] = x;
      } else {
        poles[count][1] = x;
        count--;
      }
      x += 13;
    }
  }

  for (uint16_t x = 99; x > 89; x--){
    printf("[%d, %d] \n", poles[x][0], poles[x][1]);
  }
  printf("\n");

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

void observer_node_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, observer_func, OPENCVDEMO_FPS);
}