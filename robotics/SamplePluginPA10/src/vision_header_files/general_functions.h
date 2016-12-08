#ifndef GENERAL_FUNCTIONS
#define GENERAL_FUNCTIONS

// Includes
#include "setup.h"
using namespace std;
using namespace cv;

// Decleration
void load_data(vector<Mat> &input, String &path, int type);
void draw_circles(Mat &input, vector<Point2f> &centers);
void ColorThreshold(int, void*);
void CannyThreshold(int, void*);

#endif
