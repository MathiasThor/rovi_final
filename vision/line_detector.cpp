// LINE_DETECTOR
// These functions are used for line detection
//______________________________________________________________________________

void line_detector(Mat &input, vector<Point> &marker_points)
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

  //// THICKLINE //////////
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



// *** Find Intersections ***
// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool intersection(Point o1, Point p1, Point o2, Point p2, Point &r)
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
