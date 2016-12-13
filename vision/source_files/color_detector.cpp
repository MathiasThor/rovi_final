// COLOR DETECTOR
// These functions are used for detection of color marker
//______________________________________________________________________________

#include "color_detector.h"

// *** Color segmentation ***
// Takes a color image and finds the reference points for marker 1.
void color_detector(Mat &input_image, vector<Point2f> &marker_points)
{
    vector<Point2f> temp_points;

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

    // Push back the points found and calculate center point for the marker.
    for(int i = 0; i < red_centers.size(); i++){
      u_center += red_centers[i].x;
      v_center += red_centers[i].y;
      temp_points.push_back(red_centers[i]);
    }

    for(int i = 0; i < blue_centers.size(); i++){
       u_center += blue_centers[i].x;
       v_center += blue_centers[i].y;
       temp_points.push_back(blue_centers[i]);
    }

    u_center = u_center/(blue_centers.size() + red_centers.size());
    v_center = v_center/(blue_centers.size() + red_centers.size());

    temp_points.push_back(Point(floor(u_center), floor(v_center)));

    // If 5 points are found, based on the red circle point, find th opposite blue circle point.
    if(temp_points.size() == 5){
      int index = 0;
      float max_dist = 0;
      for(int i = 1; i < 4; i++){
        // Calculate the distance between one of the blue circle points and the red.
        float current_dist = sqrt( powf(temp_points[0].x - temp_points[i].x, 2) + powf(temp_points[0].y - temp_points[i].y,2) );
        if(current_dist > max_dist){
          max_dist = current_dist;
          index = i;
        }
      }

      // Push back the center of the marker, along with the red and opposite blue.
      marker_points.push_back(temp_points[4]);
      Point2f red = temp_points[0];
      Point2f blue_op = temp_points[index];

      marker_points.push_back(red);
      marker_points.push_back(blue_op);
      temp_points.erase(temp_points.begin() + index);
      temp_points.erase(temp_points.begin());

      // Calculate determinant of oriented triangle. Used to correctly sort the remaining points.
      float determinant = (blue_op.x * temp_points[0].y - blue_op.y * temp_points[0].x) - red.x * (temp_points[0].y - blue_op.y) + red.y * (temp_points[0].x - blue_op.x);

      if(determinant >= 0){
        marker_points.push_back(temp_points[1]);
      }
      else{
        marker_points.push_back(temp_points[0]);
      }
    }
    // If 5 points are not found, return all points.
    else{
      marker_points = temp_points;
    }

}

// *** Color segmentation ***
// Segments either blue or red color and returns binary image.
Mat color_segmentation(Mat &input, int type)
{
  Mat output;

  // Values used for segmentation
  int Sat_lower = 30;    // Saturation lower limit
  int Sat_upper = 220;    // Saturation upper limit
  int Val_lower = 30;      // Value lower limit
  int Val_upper = 220;    // Value upper limit
  int Hue_lower = 0;      // Hue lower limit
  int Hue_upper = 255;     // Hue upper limit

  if(type == RED){
    // Change hue limits to match red colors
    Hue_lower = 0;
    Hue_upper = 15;
    Sat_lower = 150;

    inRange(input, Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output);
  }
  else if(type == BLUE){
    // Change hue limits to match blue colors
    Hue_lower = 110;
    Hue_upper = 120;

    inRange(input, Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output);
  }
  else{
    cout << "Error, no type chosen" << endl;
    return input;
  }

  return output;
}

// *** Find Circle Contours ***
// Sorts out circle contours based on parameters of the input contours.
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

    // Sort out bad contours
    if(perimeter > 100 && circle_constant > 0.7)
    {
      circle_contours.push_back(contours[j]);
    }
  }

  return circle_contours;

}

// *** Find Centers ***
// Find the centers of the contours based on image moments
vector<Point2f> find_centers(vector<vector<Point> > &input_contours)
{
  vector<Point2f> circle_centers;

  for(int i = 0; i < input_contours.size(); i++) // For every contour
  {
    // Calculate moments and determine center
    Moments circle_moments = moments(input_contours[i], false);
    int center_u = floor(circle_moments.m10/circle_moments.m00);
    int center_v = floor(circle_moments.m01/circle_moments.m00);

    circle_centers.push_back(Point2f(center_u, center_v));
  }

  return circle_centers;
}
