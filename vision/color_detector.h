#ifndef COLOR_DETECTOR
#define COLOR_DETECTOR

// Includes
#include "setup.h"
#include "general_functions.h"
using namespace std;
using namespace cv;

// Decleration
vector<Mat> color_detector();
vector<Mat> color_segmentation(vector<Mat> &input, int type);
vector<vector<vector<Point> > > find_circle_contours(vector<Mat> &input, int perimeter_thresh, int circle_thresh);
vector<vector<Point> > find_centers(vector<vector<vector<Point> > > &input_contours);

#endif
