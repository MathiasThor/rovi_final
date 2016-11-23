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

  // Calculate centroid of best matches
  cv::Point corny_center = (scene_corners[0] + Point2f( img_object.cols, 0) + scene_corners[1] + Point2f( img_object.cols, 0) + scene_corners[2] + Point2f( img_object.cols, 0) + scene_corners[3] + Point2f( img_object.cols, 0)) / 4;
  cout << "SCENE CORNOR: " << scene_corners[0] << endl;
  // Draw centroid
  cv::circle(img_matches, corny_center, 5, cv::Scalar(0, 0, 255), 5);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 0, 255), 2 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar(0, 0, 255), 2 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar(0, 0, 255), 2 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar(0, 0, 255), 2 );

  //-- Show detected matches
  //imshow( "Good Matches & Object detection", img_matches );

  input_image = img_matches.clone();
}
