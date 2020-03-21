#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void colorfilter_f(struct image_t *img);

void filter_color(cv::Mat &input, cv::Mat &output, cv::Mat &msk);

const uint8_t n_cf = 2;

// Lower bounds
cv::Scalar l1 {0,0,0};
cv::Scalar u1 {255,255,255};

cv::Scalar l2 {100,100,100};
cv::Scalar u2 {255,255,255};




std::vector<std::vector<cv::Scalar>> color_filters {{l1,u1}, {l2,u2}};


// std::vector<std::vector<std::vector<int>>> color_filters[n_cf][2][3]
//     {{{0,0,0},{255,255,255}},
//     {{100,100,100}, {255,255,255}}};

// std::vector<int> hey {1,2,3,4};
// std::vector<std::vector<int>> hey2 {hey,hey,hey,hey};


// int color_filters[n_cf][2][3] = {{{0,0,0},{255,255,255}},
//                                  {{100,100,100}, {255,255,255}}};


