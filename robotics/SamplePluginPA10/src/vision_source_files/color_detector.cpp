// COLOR DETECTOR
// These functions are used for detection of color marker
//______________________________________________________________________________

#include "../vision_header_files/color_detector.h"

// *** Color segmentation ***
// Example on syntax for function
void color_detector(Mat &input_image, vector<Point2f> &marker_points)
{
    // Segment the blue and red color in the images
    Mat blue_output  = color_segmentation(input_image, BLUE);
    Mat red_output   = color_segmentation(input_image, RED);

    // Find contours that belong to circles
    vector<vector<Point> > blue_circles = find_circle_contours(blue_output, 100, 0.7);
    vector<vector<Point> > red_circles  = find_circle_contours(red_output, 100, 0.7);

    // Find the center of the contours
    vector<Point2f> blue_centers = find_centers(blue_circles);
    vector<Point2f> red_centers  = find_centers(red_circles);

    float u_center = 0;
    float v_center = 0;

    for(int i = 0; i < blue_centers.size(); i++){
       u_center += blue_centers[i].x;
       v_center += blue_centers[i].y;
       //marker_points.push_back(blue_centers[i]);
    }

    for(int i = 0; i < red_centers.size(); i++){
      u_center += red_centers[i].x;
      v_center += red_centers[i].y;
      marker_points.push_back(red_centers[i]);
    }

    u_center = u_center/(blue_centers.size() + red_centers.size());
    v_center = v_center/(blue_centers.size() + red_centers.size());

    marker_points.push_back(Point(floor(u_center), floor(v_center)));
}

// *** Color segmentation ***
// Example on syntax for function
Mat color_segmentation(Mat &input, int type)
{
  Mat output;

  int Sat_lower = 30;
  int Sat_upper = 255;
  int Val_lower = 30;
  int Val_upper = 255;
  int Hue_lower = 0;
  int Hue_upper = 255;

  if(type == RED){
    Hue_lower = 0;
    Hue_upper = 15;

    inRange(input, Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output);
  }
  else if(type == BLUE){
    Hue_lower = 110;
    Hue_upper = 170;

    inRange(input, Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output);
  }
  else{
    cout << "Error, no type chosen" << endl;
    return input;
  }

  return output;
}

// *** Find Circle Contours ***
// Example on syntax for function
vector<vector<Point> > find_circle_contours(Mat &input, int perimeter_thresh, int circle_thresh)
{
  vector<vector<Point> > contours;
  vector<vector<Point> > circle_contours;
  vector<Vec4i> hierarchy;

  /// Find contours
  findContours( input, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Extract correct contours
  for( int j = 0; j < contours.size(); j++ )
  {
    // Calculate parameters
    double area             = abs(contourArea(contours[j], true));
    double perimeter        = arcLength(contours[j], 1);
    double circle_constant  = (4 * PI * area) / (perimeter*perimeter);

    if(perimeter > 100 && circle_constant > 0.7)
    {
      circle_contours.push_back(contours[j]);
    }
  }

  return circle_contours;

}

// *** Find Centers ***
// Example on syntax for function
vector<Point2f> find_centers(vector<vector<Point> > &input_contours)
{
  vector<Point2f> circle_centers;

  for(int i = 0; i < input_contours.size(); i++) // For every contour
  {
    Moments circle_moments = moments(input_contours[i], false);
    int center_u = floor(circle_moments.m10/circle_moments.m00);
    int center_v = floor(circle_moments.m01/circle_moments.m00);

    circle_centers.push_back(Point2f(center_u, center_v));
  }

  return circle_centers;
}
