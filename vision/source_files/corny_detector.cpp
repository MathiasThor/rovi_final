#include "corny_detector.h"

// http://docs.opencv.org/3.1.0/d7/dff/tutorial_feature_homography.html
void sift(Mat &input_image, vector<Point> &marker_points)
{
  Mat img_object = imread( "./../sequences/marker_corny.png", IMREAD_GRAYSCALE );

  //-- Step 1: Detect the keypoints and extract descriptors using SIFT
  int minHessian = 400;
  Ptr<SIFT> detector = SIFT::create( minHessian );
  std::vector<KeyPoint> keypoints_object, keypoints_scene;
  Mat descriptors_object, descriptors_scene;
  detector->detectAndCompute( img_object, Mat(), keypoints_object, descriptors_object );
  detector->detectAndCompute( input_image, Mat(), keypoints_scene, descriptors_scene );

  //-- Step 2: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );
  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_object, keypoints_object, input_image, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(1),
               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;
  for( size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }
  Mat H = findHomography( obj, scene, RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);
  perspectiveTransform( obj_corners, scene_corners, H);

  for (int i = 0; i < scene_corners.size(); i++) {
    scene_corners[i] += Point2f( img_object.cols, 0);
  }

  Point2f scene_midt_point = (scene_corners[0] + scene_corners[1] + scene_corners[2] +scene_corners[3]) / 4;

  cout << "SCENE CORNER  1: " << (scene_corners[0])-scene_midt_point << endl;
  cout << "SCENE CORNER  2: " << (scene_corners[1])-scene_midt_point << endl;
  cout << "SCENE CORNER  3: " << (scene_corners[2])-scene_midt_point << endl;
  cout << "SCENE CORNER  4: " << (scene_corners[3])-scene_midt_point << endl;
  cout << "SCENE MIDTPOINT: " << scene_midt_point-scene_midt_point << endl << endl;
  // Draw centroid
  cv::circle(img_matches, scene_midt_point, 5, cv::Scalar(0, 0, 255), 5);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0], scene_corners[1], Scalar(0, 0, 255), 2 );
  line( img_matches, scene_corners[1], scene_corners[2], Scalar(0, 0, 255), 2 );
  line( img_matches, scene_corners[2], scene_corners[3], Scalar(0, 0, 255), 2 );
  line( img_matches, scene_corners[3], scene_corners[0], Scalar(0, 0, 255), 2 );

  for(int i=0; i < scene_corners.size(); i++)
  {
      circle(img_matches, scene_corners[i], 3, Scalar(0,255,0), -1);
  }

  //-- Show detected matches
  //imshow( "Good Matches & Object detection", img_matches );

  input_image = img_matches.clone();

  if (false) {

    // 2D image points
    std::vector<cv::Point2d> image_points;
    image_points.push_back( cv::Point2d( (double)scene_midt_point.x, (double)scene_midt_point.y ) );    // MIDT
    image_points.push_back( cv::Point2d( (double)scene_corners[0].x, (double)scene_corners[0].y ) );    // TOP-LEFT
    image_points.push_back( cv::Point2d( (double)scene_corners[1].x, (double)scene_corners[1].y ) );    // TOP-RIGHT
    image_points.push_back( cv::Point2d( (double)scene_corners[2].x, (double)scene_corners[2].y ) );    // BOT-LEFT
    image_points.push_back( cv::Point2d( (double)scene_corners[3].x, (double)scene_corners[3].y ) );    // BOT-LEFT

    // 3D model points.
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(   0.0f,    0.0f, 0.0f));     // MIDT
    model_points.push_back(cv::Point3d( 200.0f, -200.0f, 0.0f));     // TOP-LEFT
    model_points.push_back(cv::Point3d( 200.0f,  200.0f, 0.0f));     // TOP-RIGHT
    model_points.push_back(cv::Point3d(-200.0f,  200.0f, 0.0f));     // BOT-RIGHT
    model_points.push_back(cv::Point3d(-200.0f, -200.0f, 0.0f));     // BOT-LEFT

    // Camera internals
    double focal_length = input_image.cols;//input_image.cols*5; // Approximate focal length.
    Point2d center = cv::Point2d(input_image.cols/2,input_image.rows/2);
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion

    cout << "Camera Matrix " << endl << camera_matrix << endl ;
    // Output rotation and translation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    // Solve for pose
    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

    // Project a 3D point (0, 0, 1000.0) onto the image plane.
    // We use this to draw a line sticking out of the nose
    vector<Point3d> midpoint3D;
    vector<Point2d> midpoint2D;
    midpoint3D.push_back(Point3d(0,0,1000));

    projectPoints(midpoint3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, midpoint2D);

    cv::line(input_image, image_points[0], midpoint2D[0], cv::Scalar(255,0,0), 2);

    cout << "Rotation Vector " << endl << rotation_vector << endl;
    cout << "Translation Vector" << endl << translation_vector << endl;

    cout <<  midpoint2D << endl;

    // Display image.
    //cv::imshow("Output", input_image);
    //waitKey(0);

  }
}
