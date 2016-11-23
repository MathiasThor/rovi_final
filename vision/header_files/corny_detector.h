#ifndef CORNY
#define CORNY

// Includes
#include "setup.h"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

// Decleration
void sift(Mat &input_image, vector<Point> &marker_points);
void pose_estimator(Mat &input_image, vector<Point> &marker_points);

#endif
