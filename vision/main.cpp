//______________ FINAL PROJECT ROVI | COMPUTER VISION ________________________
// Description
//
//
// Made by: Mathias Thor               mthor13@student.sdu.dk
//          Christian Koed Pedersen    chped13@student.sdu.dk
//______________________________________________________________________________

//____________________ INCLUDE FILES ____________________
#include "setup.h"
#include "color_detector.h"

// _____________ GLOBAL VARIABLES ____________________
char* window_name = "Output Sequence";

//______________ FUNCTION DECLARATIONS ________________
// See explanations of functions below the function, further down in the code.
//void load_data(vector<Mat> &input, String &path, int type = 1);
bool intersection(Point o1, Point p1, Point o2, Point p2, Point &r);
//vector<Mat> color_segmentation(vector<Mat> &input, int type);
//vector<vector<vector<Point> > > find_circle_contours(vector<Mat> &input, int perimeter_thresh, int circle_thresh);
//vector<vector<Point> > find_centers(vector<vector<vector<Point> > > &input_contours);
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
          //_________ DETECTOR __________
          output_sequence = color_detector();
        }
        break;
    case COLOR_INPUT_HARD:
        {
          // LOAD DATA
          String color_path_hard("./sequences/marker_color_hard/*.png");
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
        String line_path("./sequences/marker_thinline/*.png");
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
        String line_path_hard("./sequences/marker_thinline/*.png");
        load_data(input_sequence, line_path_hard, GRAY);

        output_sequence = input_sequence;
        }
        break;
    case THICKLINE_INPUT:
        {
        // LOAD DATA
        String thickline_path("./sequences/marker_thickline/*.png");
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
        String corny_path_hard("./sequences/marker_thickline/*.png");
        load_data(input_sequence, corny_path_hard, HSV);

        vector<Mat> Black_white = color_segmentation(input_sequence, B_W);

        output_sequence = Black_white;

        }
        break;
    case TEST:
        cout << "-- Test case --" << endl;

        Mat image;
        Mat dst;
        image = imread("./sequences/marker_thickline/marker_thickline_01.png", 1);

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
