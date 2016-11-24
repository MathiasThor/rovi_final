// GENERAL FUNCTIONS
// These functions are used for general purpose opencv
//______________________________________________________________________________

#include "general_functions.h"

// *** Load data ***
// Loads the image data into a vector of Mat's, and converts to either gray or HSV
void load_data(vector<Mat> &input, String &path, int type)
{
  vector<String> fn;
  glob(path, fn, true); // true = Recursive

  for (size_t k = 0; k < fn.size(); k++)
  {
       Mat im_in = imread(fn[k]);
       Mat im_out;

       switch (type) {
         case GRAY:
          cvtColor(im_in, im_out, CV_BGR2GRAY);
          break;
         case HSV:
          cvtColor(im_in, im_out, CV_BGR2HSV);
          break;
         default:
          im_out = im_in;
          break;
       }
       input.push_back(im_out);
  }
}

// *** Draw Circles ***
// Loads the image data into a vector of Mat's, and converts to either gray or HSV
void draw_circles(Mat &input, vector<Point> &centers)
{
  for(int i = 0; i < centers.size(); i++){
    circle(input, centers[i], 5, Scalar(255, 255, 255));
  }
}