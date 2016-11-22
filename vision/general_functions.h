#ifndef GENERAL_FUNCTIONS
#define GENERAL_FUNCTIONS

// Includes
#include "setup.h"
using namespace std;
using namespace cv;

// Decleration
void load_data(vector<Mat> &input, String &path, int type);
void ColorThreshold(int, void*);
void CannyThreshold(int, void*);

#endif
