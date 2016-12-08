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

#define MIN_HESSIAN       400

struct SIFT_parameters{
  std::vector<KeyPoint> keypoints;
  Mat descriptors;

  std::vector< DMatch > matches;

  Mat image;
};

// Decleration
void corny_detector(Mat &input_image, vector<Point2f> &marker_points, SIFT_parameters &object);
Mat draw_sift_matches(SIFT_parameters &object, SIFT_parameters &scene);
void draw_object(Mat &input, vector<Point2f> &marker_points);
void pose_estimator(Mat &input_image, vector<Point2f> &marker_points);
void init_corny(SIFT_parameters &marker, Mat &marker_im);

#endif
