// COLOR DETECTOR
// These functions are used for detection of color marker
//______________________________________________________________________________

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
