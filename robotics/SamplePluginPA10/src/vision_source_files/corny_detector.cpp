
#include "../vision_header_files/corny_detector.h"

// ******* CORNY DETECTOR **********
// Find the corny marker based on SURF or SIFT feature extraction.
// Inspired by: http://docs.opencv.org/3.1.0/d7/dff/tutorial_feature_homography.html
void corny_detector(Mat &input_image, vector<Point2f> &marker_points, SIFT_parameters &object)
{
  // Generate SIFT class object and parameters needed for the scene
  cv::Ptr<SURF> detector = SURF::create( 500 ); // 500 - We want more points on the scene than on object image.
  SIFT_parameters scene;

  // Clone input image to scene, and detect and compute keypoints and descriptors.
  scene.image = input_image.clone();
  detector->detectAndCompute( scene.image, Mat(), scene.keypoints, scene.descriptors );

  // Generate Flann Based Matcher
  FlannBasedMatcher matcher;

  // Convert types if not correct
  if(object.descriptors.type()!=CV_32F) {
    object.descriptors.convertTo(object.descriptors, CV_32F);
  }
  if(scene.descriptors.type()!=CV_32F) {
      scene.descriptors.convertTo(scene.descriptors, CV_32F);
  }

  // Match the scene with the corny marker.
  matcher.match( object.descriptors, scene.descriptors, scene.matches );
  double min_dist = 100;

  // Calculate the smallest distance match
  for( int i = 0; i < object.descriptors.rows; i++ )
  {
    double dist = scene.matches[i].distance;
    if( dist < min_dist ){
      min_dist = dist;
    }
  }

  // Collect the good matches (Those who are below 6 times the smallest distance)
  vector< DMatch > good_matches;
  for( int i = 0; i < object.descriptors.rows; i++ ){
    if( scene.matches[i].distance < 6*min_dist ){
      good_matches.push_back( scene.matches[i]);
    }
  }
  scene.matches = good_matches;

  // Find the keypoints of both the object and the scene, that belongs to the good matches.
  vector<Point2f> obj_points;
  vector<Point2f> scene_points;
  for( size_t i = 0; i < scene.matches.size(); i++ )
  {
    obj_points.push_back( object.keypoints[ scene.matches[i].queryIdx ].pt );
    scene_points.push_back( scene.keypoints[ scene.matches[i].trainIdx ].pt );
  }

  // Find Homography based on the object points and scene points using RANSAC.
  Mat H = findHomography( obj_points, scene_points, RANSAC );

  // Specify the corners of the object image.
  vector<Point2f> object_corners(4);
  object_corners[0] = cvPoint(0,0);
  object_corners[1] = cvPoint( object.image.cols, 0 );
  object_corners[2] = cvPoint( object.image.cols, object.image.rows );
  object_corners[3] = cvPoint( 0, object.image.rows );

  // Find the corners of the marker on the scene, based on the object image and the homography.
  vector<Point2f> scene_corners(4);
  perspectiveTransform( object_corners, scene_corners, H);

  // Generate center point and push back reference points (corners + center)
  Point2f marker_center_point = (scene_corners[0] + scene_corners[1] + scene_corners[2] +scene_corners[3]) / 4;
  marker_points.push_back(marker_center_point);

  for(int i = 0; i < scene_corners.size(); i++) {
    marker_points.push_back(scene_corners[i]);
  }
}

// ******** DRAW SIFT MATCHES ***********
// Used to generate an image containing the matches of either SIFT or SURF.
Mat draw_sift_matches(SIFT_parameters &object, SIFT_parameters &scene)
{
  Mat img_matches;
  drawMatches( object.image, object.keypoints, scene.image, scene.keypoints,
               scene.matches, img_matches, Scalar::all(-1), Scalar::all(1),
               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  return img_matches;
}

// *********** DRAW OBJECT **********
// Draw the object found in the scene
void draw_object(Mat &input, vector<Point2f> &marker_points)
{
  if(marker_points.size() == 5){
    // Draw centroid
    circle(input, marker_points[0], 5, cv::Scalar(255, 0, 0), 5);

    // Draw corners
    for(int i = 1; i < marker_points.size() - 1; i++)
    {
        circle(input, marker_points[i], 3, Scalar(0,255,0), -1);
    }

    // Draw lines between the corners
    line( input, marker_points[1], marker_points[2], Scalar(255, 0, 0), 10 );
    line( input, marker_points[2], marker_points[3], Scalar(0, 255, 0), 10 );
    line( input, marker_points[3], marker_points[4], Scalar(0, 0, 255), 10 );
    line( input, marker_points[4], marker_points[1], Scalar(0, 255, 255), 10 );
  }
}


// **** INIT OBJECT ****
// Initiliaze the SIFT/SURF parameters for the marker. (NOT USED ANYMORE)
void init_corny(SIFT_parameters &marker)
{
  marker.image;
  cv::Ptr<SURF> object_detector = SURF::create( 300 ); // MinHessian = 400;

  object_detector->detectAndCompute( marker.image, Mat(), marker.keypoints, marker.descriptors );
}
