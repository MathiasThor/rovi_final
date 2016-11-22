//______________ FINAL PROJECT ROVI | COMPUTER VISION ________________________
// Description
//
//
// Made by: Mathias Thor               mthor13@student.sdu.dk
//          Christian Koed Pedersen    chped13@student.sdu.dk
//______________________________________________________________________________

//____________________ INCLUDE FILES ____________________
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "stdio.h"

//____________________ NAME SPACES ____________________
using namespace cv;
using namespace std;

//_____________________ DEFINES _______________________
#define HSV         3
#define GRAY        2

#define RED         1
#define BLUE        2
#define B_W         3

#define COLOR_INPUT             1
#define COLOR_INPUT_HARD        2
#define LINE_INPUT              3
#define LINE_INPUT_HARD         4
#define THICKLINE_INPUT         5
#define THICKLINE_INPUT_HARD    6
#define TEST                    7

#define PI                  3.14159265359

// _____________ GLOBAL VARIABLES ____________________
Mat src, src_threshold;
Mat dst, output_threshold;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 255;
int highThreshold;
int const max_highThreshold = 255;
int ratio = 3;
int kernel_size;
int const max_kernel_size = 21;
int low_hueThreshold;
int high_hueThreshold;
int low_satThreshold;
int high_satThreshold;
int low_valThreshold;
int high_valThreshold;
int const max_colorThreshold = 255;

char* window_name = "Output Sequence";

//______________ FUNCTION DECLARATIONS ________________
// See explanations of functions below the function, further down in the code.
void load_data(vector<Mat> &input, String &path, int type = 1);
bool intersection(Point o1, Point p1, Point o2, Point p2, Point &r);
vector<Mat> color_segmentation(vector<Mat> &input, int type);
vector<vector<vector<Point> > > find_circle_contours(vector<Mat> &input, int perimeter_thresh, int circle_thresh);
vector<vector<Point> > find_centers(vector<vector<vector<Point> > > &input_contours);
void CannyThreshold(int, void*);
void ColorThreshold(int, void*);

//____________________ MAIN PROGRAM ____________________
int main( int argc, char** argv)
{
  // color_input = 1, color_input_hard = 2, line_input = 3,
  // line_input_hard = 4, thickline_input = 5, thickline_input_hard = 6.
  // test_input = 7.

  int sequence_number;
  cout << "____________ Final Project ROVI - Computer vision ______________" << endl << endl;
  cout << "Choose a sequence of images..." << endl;
  cout << "----------------------------------------------------------------" << endl;
  cout << "Color_input = 1, Color_input_hard = 2, Line_input = 3," << endl;
  cout << "Line_input_hard = 4, Corny_input = 5, Corny_input_hard = 6." << endl;
  cout << "----------------------------------------------------------------" << endl;
  cout << "Enter sequence number: ";
  cin >> sequence_number;

  //_____________ VARIABLES AND DATA ______________
  vector<Mat> input_sequence;
  vector<Mat> output_sequence;
  Mat output_image;

  // ________________ Object Recognition ____________________
  switch(sequence_number){
    case COLOR_INPUT:
        {
          //_________ LOAD DATA __________
          String color_path("../sequences/marker_color/*.png");
          load_data(input_sequence, color_path, HSV);

          // Segment the blue color in the images
          vector<Mat> blue_output = color_segmentation(input_sequence, BLUE);

          // Find contours that belong to circles
          vector<vector<vector<Point> > > circles = find_circle_contours(blue_output, 100, 0.7);

          // Find the center of the contours
          vector<vector<Point> > centers = find_centers(circles);

          // Set the output sequence and draw the centers on the images.
          output_sequence = input_sequence;

          for(int i = 0; i < output_sequence.size();i++)
          {
            for(int j = 0; j < centers[i].size(); j++)
            {
                circle(output_sequence[i], centers[i][j], 5, Scalar(255, 255, 255));
            }
          }
        }
        break;
    case COLOR_INPUT_HARD:
        {
          // LOAD DATA
          String color_path_hard("../sequences/marker_color_hard/*.png");
          load_data(input_sequence, color_path_hard, HSV);

          // Segment the blue and red color in the images
          vector<Mat> blue_output = color_segmentation(input_sequence, BLUE);
          vector<Mat> red_output = color_segmentation(input_sequence, RED);

          // Find contours that belong to circles
          vector<vector<vector<Point> > > blue_circles = find_circle_contours(blue_output, 100, 0.7);
          vector<vector<vector<Point> > > red_circles = find_circle_contours(red_output, 100, 0.7);

          // Find the center of the contours
          vector<vector<Point> > blue_centers = find_centers(blue_circles);
          vector<vector<Point> > red_centers = find_centers(red_circles);
          vector<vector<Point> > coordinates;

          float u_center = 0;
          float v_center = 0;

          for(int i = 0; i < blue_centers.size(); i++){
            for(int j = 0; j < blue_centers[i].size(); j++){
              u_center += blue_centers[i][j].x;
              v_center += blue_centers[i][j].y;
            }

            for(int j = 0; j < red_centers[i].size(); j++){
              u_center += red_centers[i][j].x;
              v_center += red_centers[i][j].y;
            }

            u_center = u_center/(blue_centers[i].size() + red_centers[i].size());
            v_center = v_center/(blue_centers[i].size() + red_centers[i].size());

            vector<Point> temp_coordinate;
            temp_coordinate.push_back(Point(floor(u_center), floor(v_center)));
            if(red_centers[i].size() > 0){
                temp_coordinate.push_back(red_centers[i][0]);   // STUPID FRAME
            }
            coordinates.push_back(temp_coordinate);

            u_center = 0;
            v_center = 0;
          }

          // Set the output sequence and draw the centers on the images.
          output_sequence = input_sequence;

          for(int i = 0; i < output_sequence.size();i++)
          {
            for(int j = 0; j < coordinates[i].size(); j++)
            {
                circle(output_sequence[i], coordinates[i][j], 5, Scalar(255, 255, 255));
            }
          }

        }
        break;
    case LINE_INPUT:
        {
        // LOAD DATA
        String line_path("../sequences/marker_thinline/*.png");
        load_data(input_sequence, line_path, GRAY);

        vector<Mat> canny_sequence;
        vector<vector<Vec2f> > hough_lines;

        for(int i = 0; i < input_sequence.size(); i++){
          Mat temp;
          Canny( input_sequence[i], temp, 100, 150, 3 );
          canny_sequence.push_back(temp);

          vector<Vec2f> lines;
          HoughLines(temp, lines, 1, CV_PI/180, 250, 0, 0 );
          hough_lines.push_back(lines);
        }

        vector<vector<vector<Point> > > line_points;

        for( int i = 0; i < input_sequence.size(); i++ ){
          Point pt1, pt2;
          vector<vector<Point> > temp_lines;
          line_points.push_back(temp_lines);
          for( int j = 0; j < hough_lines[i].size(); j++ ){
           float rho = hough_lines[i][j][0], theta = hough_lines[i][j][1];
           double a = cos(theta), b = sin(theta);
           double x0 = a*rho, y0 = b*rho;
           pt1.x = cvRound(x0 + 1000*(-b));
           pt1.y = cvRound(y0 + 1000*(a));
           pt2.x = cvRound(x0 - 1000*(-b));
           pt2.y = cvRound(y0 - 1000*(a));

           vector<Point> temp_line;
           temp_line.push_back(pt1);
           temp_line.push_back(pt2);

           line_points[i].push_back(temp_line);
          }
        }

        vector<vector<Point> > intersections;
        for(int i = 0; i < input_sequence.size(); i++){
          vector<Point> temp;
          intersections.push_back(temp);
          for(int j = 0; j < line_points[i].size(); j++){
            for(int k = j + 1; k < line_points[i].size(); k++){
              Point intersection_point;
              if(intersection(line_points[i][j][0], line_points[i][j][1], line_points[i][k][0], line_points[i][k][1], intersection_point)){
                intersections[i].push_back(intersection_point);
              }
            }
          }
        }

        // Draw lines and intersections
        for(int i = 0; i < input_sequence.size(); i++){
          for(int j = 0; j < line_points[i].size(); j++){
             line( input_sequence[i], line_points[i][j][0], line_points[i][j][1], Scalar(0,0,255), 3, CV_AA);
          }
          for(int j = 0; j < intersections[i].size(); j++){
            circle(input_sequence[i], intersections[i][j], 5, Scalar(255, 255, 255));
          }
        }

        output_sequence = input_sequence;


        }
        break;
    case LINE_INPUT_HARD:
        {
        // LOAD DATA
        String line_path_hard("../sequences/marker_thinline/*.png");
        load_data(input_sequence, line_path_hard, GRAY);

        output_sequence = input_sequence;
        }
        break;
    case THICKLINE_INPUT:
        {
        // LOAD DATA
        String thickline_path("../sequences/marker_thickline/*.png");
        load_data(input_sequence, thickline_path, HSV);

        vector<Mat> canny_sequence;
        vector<vector<Vec4i> > hough_lines;

        for(int i = 0; i < input_sequence.size(); i++){
          Mat temp;
          Canny( input_sequence[i], temp, 100, 150, 3 );
          canny_sequence.push_back(temp);

          vector<Vec4i> lines;
          HoughLinesP(temp, lines, 1, CV_PI/180, 50, 50, 10 );
          hough_lines.push_back(lines);
        }

        // Convert to points
        for(int i = 0; i < input_sequence.size(); i++){
          for(int j = 0; j < hough_lines[i].size(); j++){
            circle(input_sequence[i], Point(hough_lines[i][j][0], hough_lines[i][j][1]), 5, Scalar(255, 255, 255));
            circle(input_sequence[i], Point(hough_lines[i][j][2], hough_lines[i][j][3]), 5, Scalar(255, 255, 255));
            cout << "I: " << i << endl;
          }
        }

        output_sequence = input_sequence;

        }
        break;
    case THICKLINE_INPUT_HARD:
        {
        // LOAD DATA
        String corny_path_hard("../sequences/marker_thickline/*.png");
        load_data(input_sequence, corny_path_hard, HSV);

        vector<Mat> Black_white = color_segmentation(input_sequence, B_W);

        output_sequence = Black_white;

        }
        break;
    case TEST:
        cout << "-- Test case --" << endl;

        Mat image;
        Mat dst;
        image = imread("../sequences/marker_thickline/marker_thickline_01.png", 1);

        cvtColor(image, dst, CV_BGR2HSV);
        inRange(dst, Scalar(0, 0, 0), Scalar(255, 255, 30), dst);

        Canny( dst, dst, 100, 150, 3 );

        vector<Vec4i> lines;
        HoughLinesP( dst, lines, 1, CV_PI/180, 20, 175, 250 );

        for(int i = 0; i < lines.size(); i++){
            Point first(lines[i][0], lines[i][1]);
            Point second(lines[i][2], lines[i][3]);
            line( image, first, second, Scalar(0,0,255), 1, CV_AA);
        }

        imshow("Test", dst);

        waitKey(0);

        return 0;
        break;
  }

  // --------- Output the video --------
  /*VideoWriter output_vid;
  output_vid.open("./output_vid.avi", CV_FOURCC('M','J','P','G'), 5, Size(1024, 768));
  for(int i = 0; i < output_sequence.size(); i++)
  {
    output_vid.write(output_sequence[i]);
  }
  output_vid.release();*/

  namedWindow( window_name, CV_WINDOW_NORMAL );

  // ------- Show results -----
  for(int i = 0; i < output_sequence.size(); i++)
  {
    //src_threshold = output_sequence[i];

    //createTrackbar( "Max Threshold:", window_name, &highThreshold, max_highThreshold, CannyThreshold );
    //createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    //createTrackbar( "Smooth amount:", window_name, &kernel_size, max_kernel_size, CannyThreshold );
    //CannyThreshold(0,0);

    /*createTrackbar( "Hue_min:", window_name, &low_hueThreshold, max_colorThreshold, ColorThreshold );
    createTrackbar( "Hue_max:", window_name, &high_hueThreshold, max_colorThreshold, ColorThreshold );
    createTrackbar( "Sat_min:", window_name, &low_satThreshold, max_colorThreshold, ColorThreshold );
    createTrackbar( "Sat_max:", window_name, &high_satThreshold, max_colorThreshold, ColorThreshold );
    createTrackbar( "Val_min:", window_name, &low_valThreshold, max_colorThreshold, ColorThreshold );
    createTrackbar( "Val_max:", window_name, &high_valThreshold, max_colorThreshold, ColorThreshold );
    ColorThreshold(0,0);*/

    if(i == output_sequence.size() - 1){
      i = 0;
    }
    //cvtColor(output_sequence[i], output_image, CV_HSV2BGR);
    imshow(window_name, output_sequence[i]);
    if(waitKey(250) >= 0) break; // Increase x of waitKey(x) to slow down the video
  }

  waitKey(0);

  return 0;
}

//____________________ FUNCTIONS ___________________
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

       if(type == 2){
         cvtColor(im_in, im_out, CV_BGR2GRAY);
       }
       else if(type == 3){
         cvtColor(im_in, im_out, CV_BGR2HSV);
       }
       else{
         im_out = im_in;
       }

       input.push_back(im_out);
  }
}

// *** Find Intersections ***
// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(Point o1, Point p1, Point o2, Point p2,
                      Point &r)
{
    Point x = o2 - o1;
    Point d1 = p1 - o1;
    Point d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = (o1 + d1 * t1);
    return true;
}

// *** Color segmentation ***
// Example on syntax for function
vector<Mat> color_segmentation(vector<Mat> &input, int type)
{
  vector<Mat> output;

  int Sat_lower = 30;
  int Sat_upper = 220;
  int Val_lower = 30;
  int Val_upper = 220;
  int Hue_lower = 0;
  int Hue_upper = 255;

  if(type == RED){
    Hue_lower = 0;
    Hue_upper = 15;
    Sat_lower = 150;

    for(int i = 0; i < input.size(); i++)
    {
       Mat lower_temp;
       Mat upper_temp;
       output.push_back(input[i]);
       inRange(input[i], Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output[i]);
    }
  }
  else if(type == BLUE){
    Hue_lower = 110;
    Hue_upper = 120;

    for(int i = 0; i < input.size(); i++)
    {
       output.push_back(input[i]);
       inRange(input[i], Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output[i]);
    }
  }
  else if(type == B_W){
    int Val_lower_1 = 0;
    int Val_upper_1 = 10;
    int Val_lower_2 = 245;
    int Val_upper_2 = 255;
    int Sat_lower1 = 0;
    int Sat_upper1 = 255;

    for(int i = 0; i < input.size(); i++)
    {
       Mat lower_temp;
       Mat upper_temp;
       output.push_back(input[i]);
       inRange(input[i], Scalar(Hue_lower, Sat_lower, Val_lower_1), Scalar(Hue_upper, Sat_upper, Val_upper_1), lower_temp);
       inRange(input[i], Scalar(Hue_lower, Sat_lower, Val_lower_2), Scalar(Hue_upper, Sat_upper, Val_upper_2), upper_temp);
       addWeighted( lower_temp, 1, upper_temp, 1, 0.0, output[i]);
    }
  }
  else{
    for(int i = 0; i < input.size(); i++)
    {
       output.push_back(input[i]);
       inRange(input[i], Scalar(Hue_lower, Sat_lower, Val_lower), Scalar(Hue_upper, Sat_upper, Val_upper), output[i]);
    }
  }

  return output;
}

// *** Find Circle Contours ***
// Example on syntax for function
vector<vector<vector<Point> > > find_circle_contours(vector<Mat> &input, int perimeter_thresh, int circle_thresh)
{
  vector<vector<vector<Point> > > result_contours;

  for(int i = 0; i < input.size(); i++)
  {
    vector<vector<Point> > contours;
    vector<vector<Point> > circle_contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( input[i], contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

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

    result_contours.push_back(circle_contours);
  }

  return result_contours;

}

// *** Find Centers ***
// Example on syntax for function
vector<vector<Point> > find_centers(vector<vector<vector<Point> > > &input_contours)
{
  vector<vector<Point> > circle_centers(input_contours.size());

  for(int i = 0; i < input_contours.size(); i++) // For every frame
  {
    for(int j = 0; j < input_contours[i].size(); j++) // For every contour in frame
    {
      Moments circle_moments = moments(input_contours[i][j], false);
      int center_u = floor(circle_moments.m10/circle_moments.m00);
      int center_v = floor(circle_moments.m01/circle_moments.m00);

      circle_centers[i].push_back(Point(center_u, center_v));
    }
  }

  return circle_centers;
}

// *** Color Threshold ***
// Example on syntax for function
void ColorThreshold(int, void*)
{
  inRange(src_threshold, Scalar(low_hueThreshold, low_satThreshold, low_valThreshold), Scalar(high_hueThreshold, high_satThreshold, high_valThreshold), output_threshold);
  dst = Scalar::all(0);

  src_threshold.copyTo( dst, output_threshold);
  imshow( window_name, dst );
}

// *** Canny Threshold ***
// Example on syntax for function
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  if (kernel_size > 0)
    blur( src_threshold, output_threshold, Size(kernel_size,kernel_size) );
  else
    blur( src_threshold, output_threshold, Size(1,1) );


  /// Canny detector
  Canny( output_threshold, output_threshold, lowThreshold, highThreshold, 3 );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src_threshold.copyTo( dst, output_threshold);
  imshow( window_name, dst );
}
