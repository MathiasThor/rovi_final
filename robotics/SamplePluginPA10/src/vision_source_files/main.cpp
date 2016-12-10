//______________ FINAL PROJECT ROVI | COMPUTER VISION ________________________
// Description
//
//
// Made by: Mathias Thor               mthor13@student.sdu.dk
//          Christian Koed Pedersen    chped13@student.sdu.dk
//______________________________________________________________________________

//____________________ INCLUDE FILES ____________________
#include "setup.h"
#include "general_functions.h"
#include "color_detector.h"
#include "corny_detector.h"

// _____________ GLOBAL VARIABLES ____________________
char* window_name = "Output Sequence";

//____________________ MAIN PROGRAM ____________________
int main( int argc, char** argv)
{
  // color_input = 1, color_input_hard = 2, corny_input = 3, corny_input_hard = 4;

  int sequence_number;
  cout << "Final Project ROVI - Computer vision:" << endl << endl;
  cout << "Choose a sequence of images..." << endl;
  cout << endl;
  cout << "Color_input\t = 1\nColor_input_hard = 2\nCorny_input\t = 3\nCorny_input_hard = 4" << endl;
  cout << endl;
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
          String color_path("./../sequences/marker_color/*.png");
          load_data(input_sequence, color_path, HSV);

          for(int i = 0; i < input_sequence.size(); i++){
            // Start timer
            clock_t start = clock();

            // Find marker points
            vector<Point> marker_points;
            color_detector(input_sequence[i], marker_points);

            // Convert to RGB and draw circles
            cvtColor(input_sequence[i], input_sequence[i], CV_HSV2BGR);
            draw_circles(input_sequence[i], marker_points);

            // End timer
            clock_t end = clock();
            double secs = double(end - start) / CLOCKS_PER_SEC;
            cout << "Time for frame " << i << ": " << secs << " Seconds" << endl;
          }
        }
        break;
    case COLOR_INPUT_HARD:
        {
          //_________ DETECTOR __________
          String color_path("./../sequences/marker_color_hard/*.png");
          load_data(input_sequence, color_path, HSV);

          for(int i = 0; i < input_sequence.size(); i++){
            // Start timer
            clock_t start = clock();

            // Find marker points
            vector<Point> marker_points;
            color_detector(input_sequence[i], marker_points);

            // Convert to RGB and draw circles
            cvtColor(input_sequence[i], input_sequence[i], CV_HSV2BGR);
            draw_circles(input_sequence[i], marker_points);

            // End timer
            clock_t end = clock();
            double secs = double(end - start) / CLOCKS_PER_SEC;
            cout << "Time for frame " << i << ": " << secs << " Seconds" << endl;
          }
        }
        break;
    case CORNY_INPUT:
        {
          String color_path("./../sequences/marker_corny/*.png");
          load_data(input_sequence, color_path, GRAY);

          SIFT_parameters marker;
          init_corny(marker);

          for(int i = 0; i < input_sequence.size(); i++){
            clock_t start = clock();

            vector<Point2f> marker_points;
            corny_detector(input_sequence[i], marker_points, marker);
            draw_object(input_sequence[i], marker_points);

            clock_t end = clock();
            double secs = double(end - start) / CLOCKS_PER_SEC;
            cout << "Time for frame " << i << ": " << secs << " Seconds" << endl;
          }
        }
        break;
    case CORNY_INPUT_HARD:
        {
          String color_path("./../sequences/marker_corny_hard/*.png");
          load_data(input_sequence, color_path, GRAY);

          SIFT_parameters marker;
          init_corny(marker);

          for(int i = 0; i < input_sequence.size(); i++){
            clock_t start = clock();

            vector<Point2f> marker_points;
            corny_detector(input_sequence[i], marker_points, marker);
            draw_object(input_sequence[i], marker_points);

            clock_t end = clock();
            double secs = double(end - start) / CLOCKS_PER_SEC;
            cout << "Time for frame " << i << ": " << secs << " Seconds" << endl;
          }
        }
        break;
    case TEST:
        {
          String color_path("./../sequences/marker_color_hard/*.png");
          load_data(input_sequence, color_path, HSV);

          for(int i = 0; i < 3; i++){
            vector<Point> marker_points;
            if(i == 0)
            {
              //color_detector(input_sequence[0], marker_points);
              Mat test1 = color_segmentation(input_sequence[0], RED);
              Mat test2 = color_segmentation(input_sequence[0], BLUE);

              Mat test;
              addWeighted(test1, 1, test2, 1, 1, test);
              //cvtColor(input_sequence[0], input_sequence[0], CV_HSV2BGR);
              //draw_circles(input_sequence[0], marker_points);
              imshow("Test", test);
              waitKey(0);
              waitKey(0);
            }
            if(i == 1){
              //color_detector(input_sequence[0], marker_points);
              Mat test1 = color_segmentation(input_sequence[34], RED);
              Mat test2 = color_segmentation(input_sequence[34], BLUE);

              vector<vector<Point> > blue_circles = find_circle_contours(blue_output, 100, 0.7);
              vector<vector<Point> > red_circles  = find_circle_contours(red_output, 100, 0.7);

              for(int i = 0; i < blue_circles.size(); i++){
                for(int j = 0; j < blue_circles[i].size(); j++){
                  
                }
              }
              //cvtColor(input_sequence[0], input_sequence[0], CV_HSV2BGR);
              //draw_circles(input_sequence[0], marker_points);
              imshow("Test", test);
              waitKey(0);
              waitKey(0);
            }
            if(i == 2){
              //color_detector(input_sequence[0], marker_points);
              Mat test1 = color_segmentation(input_sequence[49], RED);
              Mat test2 = color_segmentation(input_sequence[49], BLUE);

              Mat test;
              addWeighted(test1, 1, test2, 1, 1, test);
              //cvtColor(input_sequence[0], input_sequence[0], CV_HSV2BGR);
              //draw_circles(input_sequence[0], marker_points);
              imshow("Test", test);
              waitKey(0);
              waitKey(0);
            }
          }

        }
        break;
  }

  // ------- Show results -----
  namedWindow( window_name, CV_WINDOW_NORMAL );
  for(int i = 0; i < input_sequence.size(); i++)
  {
    if(i == input_sequence.size() - 1){
      i = 0;
    }
    imshow(window_name, input_sequence[i]);
    waitKey(0);
  }

  return 0;
}
