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

// _____________ GLOBAL VARIABLES ____________________
char* window_name = "Output Sequence";

//____________________ MAIN PROGRAM ____________________
int main( int argc, char** argv)
{
  // color_input = 1, color_input_hard = 2, corny_input = 3, corny_input_hard = 4;

  int sequence_number;
  cout << "____________ Final Project ROVI - Computer vision ______________" << endl << endl;
  cout << "Choose a sequence of images..." << endl;
  cout << "-----------------------------------------------------------------------------" << endl;
  cout << "Color_input = 1, Color_input_hard = 2, Corny_input = 3, Corny_input_hard = 4" << endl;
  cout << "-----------------------------------------------------------------------------" << endl;
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
          String color_path("./sequences/marker_color/*.png");
          load_data(input_sequence, color_path, HSV);

          for(int i = 0; i < input_sequence.size(); i++){
            // Find marker points
            vector<Point> marker_points;
            color_detector(input_sequence[i], marker_points);

            // Convert to RGB and draw circles
            cvtColor(input_sequence[i], input_sequence[i], CV_HSV2BGR);
            draw_circles(input_sequence[i], marker_points);
          }
        }
        break;
    case COLOR_INPUT_HARD:
        {
          //_________ DETECTOR __________
          String color_path("./sequences/marker_color_hard/*.png");
          load_data(input_sequence, color_path, HSV);

          for(int i = 0; i < input_sequence.size(); i++){
            // Find marker points
            vector<Point> marker_points;
            color_detector(input_sequence[i], marker_points);

            // Convert to RGB and draw circles
            cvtColor(input_sequence[i], input_sequence[i], CV_HSV2BGR);
            draw_circles(input_sequence[i], marker_points);
          }
        }
        break;
    case CORNY_INPUT:
        {
        }
        break;
    case CORNY_INPUT_HARD:
        {
        }
        break;
    case TEST:
        {
          // For test purposes if needed...
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
    if(waitKey(250) >= 0) break; // Increase x of waitKey(x) to slow down the video
  }

  waitKey(0);

  return 0;
}
