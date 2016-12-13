#ifndef COLOR_DETECTOR
#define COLOR_DETECTOR

// Includes
#include "setup.h"
#include "general_functions.h"
using namespace std;
using namespace cv;

// Decleration
void color_detector(Mat &input_image, vector<Point2f> &marker_points);
Mat color_segmentation(Mat &input, int type);
vector<vector<Point> > find_circle_contours(Mat &input, int perimeter_thresh, int circle_thresh);
vector<Point2f> find_centers(vector<vector<Point> > &input_contours);

#endif
