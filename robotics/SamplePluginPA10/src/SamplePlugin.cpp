#include "SamplePlugin.hpp"
#include "./vision_header_files/setup.h"
#include "./vision_source_files/color_detector.cpp"
#include "./vision_source_files/general_functions.cpp"
#include "./vision_source_files/corny_detector.cpp"

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0,              SIGNAL(pressed()),            this, SLOT(btnPressed()) );
  connect(_startStopMovement, SIGNAL(pressed()),            this, SLOT(btnPressed()) );
  connect(_comboBox,          SIGNAL(activated(int)),       this, SLOT(btnPressed()) );
  connect(_followMarker,      SIGNAL(pressed()),            this, SLOT(btnPressed()) );
  connect(_resetSim,          SIGNAL(pressed()),            this, SLOT(resetSim()) );
  connect(_DT,                SIGNAL(valueChanged(double)), this, SLOT(set_dt()) );
  connect(_testRun,           SIGNAL(pressed()),            this, SLOT(testRun()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::set_dt(){
  DT = _DT->value();
  log().info() << "DT: " << _DT->value() << "\n";
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(path + "rovi_final/robotics/PA10WorkCell/ScenePA10RoVi1.wc.xml");
  if (wc == NULL) {
    cerr << "WorkCell: not found!" << endl;
  }
	getRobWorkStudio()->setWorkCell(wc);
  load_motion();
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();
  _PA10 = _wc->findDevice("PA10");
  _Marker = (MovableFrame*) _wc->findFrame("Marker");
  _Camera = _wc->findFrame("Camera");

  if (_PA10 == NULL)
    cerr << "Device: PA10 not found!" << endl;

  vel_limits = _PA10->getVelocityLimits();

	log().info() << workcell->getFilename() << "\n";

	if (_wc != NULL) {
		// Add the texture render to this workcell if there is a frame for texture
		Frame* textureFrame = _wc->findFrame("MarkerTexture");
		if (textureFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
		}
		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
		}

		// Create a GLFrameGrabber if there is a camera frame with a Camera property set
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		if (cameraFrame != NULL) {
			if (cameraFrame->getPropertyMap().has("Camera")) {
				// Read the dimensions and field of view
				double fovy;
				int width,height;
				string camParam = cameraFrame->getPropertyMap().get<string>("Camera");
				istringstream iss (camParam, istringstream::in);
				iss >> fovy >> width >> height;
				// Create a frame grabber
				_framegrabber = new GLFrameGrabber(width,height,fovy);
				SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
				_framegrabber->init(gldrawer);
			}
		}
	}
  target2 = {-(PT0[0]*f)/z,(PT0[1]*f)/z,-(PT1[0]*f)/z,(PT1[1]*f)/z,-(PT2[0]*f)/z,(PT2[1]*f)/z};
  _startStopMovement->setText("Start movement");
  resetSim();
}

void SamplePlugin::close() {
	log().info() << "CLOSE" << "\n";

	// Stop the timer
	_timer->stop();
	// Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

void SamplePlugin::resetSim(){
  _timer->stop();
  _startStopMovement->setText("Start movement");
  move_marker(marker_motion[0]);
  Q init_position(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
  _PA10->setQ(init_position, _state);
  getRobWorkStudio()->setState(_state);

  uv.clear();
  for (int row = 0; row < numOfPoints*2; row++)
    uv.push_back(0);

  stop_start_motion = false;
  test_runner = false;
  current_motion_position = 0;
  cam_update( false );
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Change Texture\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
    string marker_type = _markTex->currentText().toUtf8().constData();
    string mk_file = path + "rovi_final/robotics/SamplePluginPA10/markers/" + marker_type;
		image = ImageLoader::Factory::load(mk_file.c_str());
		_textureRender->setImage(*image);
    string back_type = _backTex->currentText().toUtf8().constData();
    string bg_file = path + "rovi_final/robotics/SamplePluginPA10/backgrounds/" + back_type;
		image = ImageLoader::Factory::load(bg_file.c_str());
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
    cam_update( false );
	}
  else if(obj==_startStopMovement){
    stop_start_motion = !stop_start_motion;
    if (stop_start_motion){
      log().info() << "Stop Motion\n";
      _startStopMovement->setText("Stop movement");
      _timer->start(100); // run 10 Hz
    }
    else{
      log().info() << "Start Motion\n";
      _startStopMovement->setText("Start movement");
      _timer->stop();
    }
	}
  else if(obj==_followMarker){
    log().info() << "Follow Marker\n";
    timer();
  } else if(obj==_comboBox){
    log().info() << "load_motion\n";
    load_motion();
    resetSim();
  }
}

void SamplePlugin::timer() {
  log().info() << "Stop Motion\n";
  if (current_motion_position == marker_motion.size()){
    if (test_runner){
      jointPos_file.close();
      toolPos_file.close();
      trackErr_file.close();
      cout << "Closed the files" << endl;
      DT *= 0.5;
      cout << "DT: " << DT << endl;
      testRun();
    } else{
      resetSim();
      _timer->start(100);
    }
  }
  else {
    move_marker(marker_motion[current_motion_position]);
    getRobWorkStudio()->setState(_state);
    vector<double> tmp_points = cam_update( true );
    getRobWorkStudio()->setState(_state);
    follow_marker( tmp_points, cvOrFile );
    getRobWorkStudio()->setState(_state);
    tracking_error_image_space();
    cam_update( false );
    current_motion_position++;
    if (test_runner){
      writeToFile();
    }
  }

}

void SamplePlugin::testRun(){
  jointPos_file.open (path + "rovi_final/robotics/SamplePluginPA10/data/joint_positions_" + to_string(DT) + ".txt");
  toolPos_file.open (path + "rovi_final/robotics/SamplePluginPA10/data/tool_positions_" + to_string(DT) + ".txt");
  trackErr_file.open (path + "rovi_final/robotics/SamplePluginPA10/data/tracking_error_" + to_string(DT) + ".txt");
  cout << "Opened the files" << endl;

  jointPos_file << DT << "\n";

  resetSim();
  test_runner=true;
  _timer->start(100);
  cout << "Collecting data..." << endl;
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

void SamplePlugin::move_marker( rw::math::VelocityScrew6D<> p_6D ){
  _Marker->setTransform(Transform3D<double>( p_6D.linear(), RPY<double>(p_6D(3),	p_6D(4),	p_6D(5)).toRotation3D() ), _state);
}

void SamplePlugin::load_motion( ){
  marker_motion.clear();
  string motion_type = _comboBox->currentText().toUtf8().constData();
  string move_file_path = path + "rovi_final/robotics/SamplePluginPA10/motions/" + motion_type;
  ifstream motion_file(move_file_path.c_str());
  VelocityScrew6D<> pos_6D;
  string input;

  // Inspired by http://stackoverflow.com/questions/14516915/read-numeric-data-from-a-text-file-in-c
  if (motion_file.is_open()) {
    while ( getline (motion_file, input,'\n') ) {
      motion_file >> pos_6D(0) >> pos_6D(1) >> pos_6D(2) >> pos_6D(3) >> pos_6D(4) >> pos_6D(5);
      marker_motion.push_back(pos_6D);
    }
    motion_file.close();
    log().info() << "Loaded: " << motion_type << "\n";
  }
}

void SamplePlugin::follow_marker( vector<double> uv_points, bool use_cv){
  //
  // Calculate u, uv[1], du and dv
  //
  // TODO - PLACE MIDTPOINT AS THE FIRST ONE
  if( use_cv ){
    for (int i = 0; i < numOfPoints; i++) {
      uv[i*2]   = uv_points[i*2 + 2];
      uv[i*2 + 1] = uv_points[i*2 + 3];

    }

    if (current_motion_position==0) {
      target2=uv;
      /*for (int i = 0; i < numOfPoints; i++) {
         target2[i*2]   -= uv_points[0];
         target2[i*2+1] -= uv_points[1];
      }*/
    }
  }
  else{
    // Get the transform of CAMARA frame relative to the MARKER frame. -OK
    Transform3D<> camara_to_marker = inverse(_Marker->fTf(_Camera, _state));
    vector< Vector3D<> > points;

    if ( numOfPoints != 1) {
         points.push_back(camara_to_marker.R() * Vector3D<>(PT0[0],PT0[1],PT0[2]) + camara_to_marker.P());
         points.push_back(camara_to_marker.R() * Vector3D<>(PT1[0],PT1[1],PT1[2]) + camara_to_marker.P());
         points.push_back(camara_to_marker.R() * Vector3D<>(PT2[0],PT2[1],PT2[2]) + camara_to_marker.P());
    }
    points.push_back(camara_to_marker.P());

    if (numOfPoints == 1) {
      uv[0] = ( points[0][0] * f ) / z;
      uv[1] = ( points[0][1] * f ) / z;
    }
    else {
      for (int i = 0; i < points.size()-1; i++) {
        uv[i*2]   = ( points[i][0] * f ) / z;
        uv[i*2+1] = ( points[i][1] * f ) / z;
      }
    }

    if (current_motion_position==0) {
      target2=uv;
      for (int i = 0; i < numOfPoints; i++) {
        target2[i*2]   -= (points[points.size()-1][0] *f) / z;
        target2[i*2+1] -= (points[points.size()-1][1] *f) / z;
      }
    }
  }

  Jacobian d_uv(numOfPoints*2,1);
  for (int i = 0; i < numOfPoints; i++) {
    d_uv(i*2,0)   = target2[i*2]   -uv[i*2];
    d_uv(i*2+1,0) = target2[i*2+1] -uv[i*2+1];
  }
  log().info() << "Frame:\t" << current_motion_position << "\n";

  log().info() << "uv:\t" << uv[0] << "\t" << uv[1] << "\t";
  if ( numOfPoints > 1)
      log().info() << uv[2] << "\t" << uv[3] << "\t" << uv[4] << "\t" << uv[5] << "\n";
  else log().info() << "\n";
  //log().info() << "targ:\t" << target[0] << "\t" << target[1] << "\t" << target[2] << "\t" << target[3] << "\t" << target[4] << "\t" << target[5] << "\n";
  log().info() << "targ2:\t" << target2[0] << "\t" << target2[1] << "\t" << target2[2] << "\t" << target2[3] << "\t" << target2[4] << "\t" << target2[5] << "\n";
  log().info() << "d_uv:\t" << d_uv(0,0) << "\t" << d_uv(1,0) << "\t";
  if ( numOfPoints > 1)
    log().info() << d_uv(2,0) << "\t" << d_uv(3,0) << "\t" << d_uv(4,0) << "\t" << d_uv(5,0) << "\n";
  else log().info() << "\n";

  //
  // Calculate the jacobian for PA10 -Ok
  //
	Jacobian J_PA10 = _PA10->baseJframe(_Camera, _state);

  //
  // Calculate the image jacobian
  //
  Jacobian J_image(numOfPoints*2,6);   // Create 6*2 Jacobian
  for (int i = 0; i < numOfPoints; i++) {
    J_image(i*2, 0)   = -(f / z);
    J_image(i*2, 1)   = 0;
    J_image(i*2, 2)   = uv[i*2]/z;
    J_image(i*2, 3)   = uv[i*2]*uv[i*2+1]/f;
    J_image(i*2, 4)   = -(((f*f)+(uv[i*2]*uv[i*2]))/(f));
    J_image(i*2, 5)   = uv[i*2+1];
    J_image(i*2+1, 0) = 0;
    J_image(i*2+1, 1) = -(f / z);
    J_image(i*2+1, 2) = (uv[i*2+1]/z);
    J_image(i*2+1, 3) = (((f*f)+(uv[i*2+1]*uv[i*2+1]))/(f));
    J_image(i*2+1, 4) = -((uv[i*2]*uv[i*2+1])/(f));
    J_image(i*2+1, 5) = -uv[i*2];
  }

  //
  // Calculate Sq
  //
  Transform3D<> base2cam = inverse(_PA10->baseTframe(_Camera, _state));
  Jacobian J_sq = Jacobian(base2cam.R());

  //
  // Calculate Z_image
  //
  Jacobian z_image = J_image * J_sq * J_PA10;
  Jacobian z_image_T(7,numOfPoints*2);
  for(int i=0; i < 7; i++){
    for(int j=0; j < 2*numOfPoints; j++){
      z_image_T(i,j) = z_image(j,i);
    }
  }

  Eigen::MatrixXd U;
  Eigen::VectorXd SIGMA;
  Eigen::MatrixXd V;
  LinearAlgebra::svd(J_image.e(),U,SIGMA,V);
  cout << "SIGMA:\n" << SIGMA << endl;
  cout << "=====================" << endl;

  double kappa = 1.2;
  Jacobian damper(numOfPoints*2,numOfPoints*2);
  for (int i = 0; i < numOfPoints*2; i++) {
    for (int j = 0; j < numOfPoints*2; j++) {
      if (i == j)
        damper(i,j) = kappa*kappa;
      else
        damper(i,j) = 0;
    }
  }
  Jacobian damped( z_image.e()*z_image_T.e() );

  //
  // Calculate dq
  //
  Jacobian pseudoinverse( z_image_T.e() * (damped).e().inverse() );


  // TODO: By adding a damper of 0.5 we can get through more frames (from 67 --> 108)
  // TODO: BY ADDING A 0.05 DAMPER IT ACTURALLY RUNS (0.05*pseudoinverse)
  Jacobian J_dq( pseudoinverse.e() * d_uv.e() );
  Q dq(J_dq.e());
  //log().info() << "dq:\t" << dq << "\n";
  Q new_q(_PA10->getQ(_state));
  velocityLimit(dq,new_q);
  _PA10->setQ(new_q, _state);
  log().info() << "========================================================================" << "\n";
}

void SamplePlugin::tracking_error_image_space(){
  // TODO MAKE FOR CV AS WELL
  Transform3D<> camara_to_marker = inverse(_Marker->fTf(_Camera, _state));
  vector< Vector3D<> > points;
  vector< double > current_uv;

  if ( numOfPoints != 1) {
       points.push_back(camara_to_marker.R() * Vector3D<>(PT0[0],PT0[1],PT0[2]) + camara_to_marker.P());
       points.push_back(camara_to_marker.R() * Vector3D<>(PT1[0],PT1[1],PT1[2]) + camara_to_marker.P());
       points.push_back(camara_to_marker.R() * Vector3D<>(PT2[0],PT2[1],PT2[2]) + camara_to_marker.P());
  }
  points.push_back(camara_to_marker.P());

  if (numOfPoints == 1) {
    current_uv.push_back(( points[0][0] * f ) / z);
    current_uv.push_back(( points[0][1] * f ) / z);
  } else {
    for (int i = 0; i < points.size()-1; i++) {
      current_uv.push_back(( points[i][0] * f ) / z);
      current_uv.push_back(( points[i][1] * f ) / z);
    }
  }

  double euclidean_dist;
  double sum;
  for (int i = 0; i < uv.size()/2; i++) {
    euclidean_dist = sqrt( pow( current_uv[i*2]-target2[i*2],2) + pow( current_uv[i*2+1]-target2[i*2+1] ,2) );
    log().info() << "Tracking error - point " << i << ": " << euclidean_dist <<"\n";
    sum += abs(euclidean_dist);
  }
  log().info() << "Tracking error - Total: " << sum <<"\n";
  last_tracking_error = sum;
}

void SamplePlugin::tracking_error_task_space(){
  // TODO MAKE FOR CV AS WELL
  // Where the actural points are:
  Transform3D<> world_to_marker = inverse(_Marker->wTf(_state)); // world to marker --> inverse = marker to world
  vector < Vector3D<> > marker_points;

  marker_points.push_back(world_to_marker.R() * Vector3D<>(PT0[0],PT0[1],PT0[2]) + world_to_marker.P());
  if ( numOfPoints > 1) {
       marker_points.push_back(world_to_marker.R() * Vector3D<>(PT1[0],PT1[1],PT1[2]) + world_to_marker.P());
       marker_points.push_back(world_to_marker.R() * Vector3D<>(PT2[0],PT2[1],PT2[2]) + world_to_marker.P());
  }

  // Where we are looking (The cam is flipped 180 degree around y when compared to marker)
  Transform3D<> world_to_camera = inverse(_Camera->wTf(_state));
  vector < Vector3D<> > camera_points;

  camera_points.push_back(world_to_camera.R() * Vector3D<>(PT0[0],PT0[1],PT0[2]) + world_to_camera.P());
  if ( numOfPoints > 1) {
       camera_points.push_back(world_to_camera.R() * Vector3D<>(PT1[0],PT1[1],PT1[2]) + world_to_camera.P());
       camera_points.push_back(world_to_camera.R() * Vector3D<>(PT2[0],PT2[1],PT2[2]) + world_to_camera.P());
  }

  double euclidean_dist;
  for (int i = 0; i < marker_points.size(); i++) {
    euclidean_dist = sqrt( pow( -camera_points[i][0]-marker_points[i][0],2) + pow( camera_points[i][1]-marker_points[i][1] ,2) );
    log().info() << "Tracking error - point " << i << ": " << euclidean_dist <<"\n";
  }
}

void SamplePlugin::velocityLimit( Q dq, Q &q ){
  for (int i = 0; i < 7; i++) {
    if (dq[i] > vel_limits[i] * DT){
        dq[i] = vel_limits[i] * DT;
        log().info() << "[Velocity limited]" << "\n";
    }
    else if (dq[i] < -vel_limits[i] * DT){
        dq[i] = -vel_limits[i] * DT;
        log().info() << "[Velocity limited]" << "\n";
    }
    q[i] += dq[i];
  }
}

void SamplePlugin::predictor( ){
  vector< double > dudv;
  prediction.clear();


  // for (size_t i = 0; i < uv.size(); i++) {
  //   cout << uv[i] << " ";
  // }
  // cout << "\n\n";
  //
  // for (size_t i = 0; i < uv_old.size(); i++) {
  //   cout << uv_old[i] << " ";
  // }
  // cout << "\n" << "###############";

  if (!uv_old.empty()) {
    for (int i = 0; i < numOfPoints; i++) {
      dudv.push_back(uv[i*2]   - uv_old[i*2]  );
      dudv.push_back(uv[i*2+1] - uv_old[i*2+1]);
      prediction.push_back( uv[i*2]  + dudv[i*2]   );
      prediction.push_back( uv[i*2+1]+ dudv[i*2+1] );
    }
  }

  uv_old.clear();
  for (int i = 0; i < uv.size(); i++) {
    uv_old.push_back(uv[i]);
  }
}

void SamplePlugin::writeToFile( ){
  if (toolPos_file.is_open()){
    Rotation3D<> tmp_tool_rot = _PA10->baseTframe(_Camera, _state).R();
    Vector3D<> tmp_tool_pos = _PA10->baseTframe(_Camera, _state).P();
    for (int i = 0; i < tmp_tool_pos.size(); i++) {
      toolPos_file << tmp_tool_pos[i] << "\t";
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        toolPos_file << tmp_tool_rot(j,i) << "\t";
      }
    }
    toolPos_file << "\n";
  } else cout << "can't open tool position file" << endl;

  if (jointPos_file.is_open()){
    Q tmp_joint =_PA10->getQ(_state);
    for (int i = 0; i < tmp_joint.size(); i++) {
      jointPos_file << tmp_joint[i] << "\t";
    }
    jointPos_file << "\n";
  } else cout << "can't open tool position file" << endl;

  if (trackErr_file.is_open())
    trackErr_file << last_tracking_error << "\n";
  else cout << "can't open tool position file" << endl;
}

vector<double> SamplePlugin::cam_update( bool get_points ){
  vector<double> uv_points;
  if (_framegrabber != NULL) {
    // Get the image as a RW image
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    _framegrabber->grab(cameraFrame, _state);
    const Image& image = _framegrabber->getImage();

    // Convert to OpenCV image
    Mat im = toOpenCVImage(image);

    Mat imflip;
    cv::flip(im, imflip, 0);

    if(cvOrFile && get_points){
      double start_time = getUnixTime();
      double stop_time;

      uv_points = marker_detection(imflip);

      stop_time = getUnixTime();
      marker_dt = stop_time - start_time;

      set_dt();
      DT = DT - marker_dt;
      if(DT < 0.01){
        DT = 0.01;
      }
      log().info() << "Time for marker detection: " << marker_dt << " Seconds\n";
    }

    cv::circle(imflip, cv::Point(imflip.cols/2+target2[0*2], imflip.rows/2+target2[0*2+1]), 20, cv::Scalar(255,60,60), 4);
    if(numOfPoints>1){
      cv::circle(imflip, cv::Point(imflip.cols/2+target2[1*2], imflip.rows/2+target2[1*2+1]), 20, cv::Scalar(60,255,60), 4);
      cv::circle(imflip, cv::Point(imflip.cols/2+target2[2*2], imflip.rows/2+target2[2*2+1]), 20, cv::Scalar(60,60,255), 4);
      if(numOfPoints == 4){
        cv::circle(imflip, cv::Point(imflip.cols/2+target2[3*2],imflip.rows/2+target2[3*2+1]), 20, cv::Scalar(255,60,255), 4);
      }
    }

    cv::circle(imflip, cv::Point(imflip.cols/2+uv[0*2],imflip.rows/2+uv[0*2+1]), 5, cv::Scalar(255,0,0), -1);
    if(numOfPoints>1){
      cv::circle(imflip, cv::Point(imflip.cols/2+uv[1*2],imflip.rows/2+uv[1*2+1]), 5, cv::Scalar(0,255,0), -1);
      cv::circle(imflip, cv::Point(imflip.cols/2+uv[2*2],imflip.rows/2+uv[2*2+1]), 5, cv::Scalar(0,0,255), -1);
      if(numOfPoints == 4){
        cv::circle(imflip, cv::Point(imflip.cols/2+uv[3*2],imflip.rows/2+uv[3*2+1]), 5, cv::Scalar(255,0,255), -1);
      }
    }

    // Show in QLabel
    QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
  }

  return uv_points;
}

vector<double> SamplePlugin::marker_detection(Mat &input){

  // Initiliaze point vector
  vector<double> uv_points;
  vector<Point2f> cv_points;

  // *********** COLOR ****************
  if(cv_choice == 1){
    Mat color_temp;
    cvtColor(input, color_temp, CV_RGB2HSV);

    color_detector(color_temp, cv_points);

    // Z distance
    double p_width = sqrt( pow(cv_points[2].x - cv_points[1].x, 2) + pow(cv_points[2].y - cv_points[1].y, 2) );
    z = (0.15 * f) / p_width;
    log().info() << "Z: " << z << "\n";
  }
  // ************** CORNY ****************
  else if(cv_choice == 2){
    SIFT_parameters marker;

    marker.image = imread( path + "rovi_final/robotics/SamplePluginPA10/markers/Marker3.ppm", IMREAD_GRAYSCALE );
    cv::Ptr<SURF> object_detector = SURF::create( 300 ); // MinHessian = 400;

    object_detector->detectAndCompute( marker.image, Mat(), marker.keypoints, marker.descriptors );

    corny_detector(input, cv_points, marker);
    draw_object(input, cv_points);

    double p_width = sqrt( pow(cv_points[0].x - cv_points[2].x, 2) + pow(cv_points[0].y - cv_points[2].y, 2) );
    z = (0.357 * f) / p_width;
    log().info() << "Z: " << z << "\n";
  }

  draw_circles(input, cv_points);

  log().info() << "Passing on points: " << cv_points.size() << "\n";
  for(int i = 0; i < cv_points.size(); i++){
    log().info() << "Point" << i+1 << " (" << cv_points[i].x << "," << cv_points[i].y << ")\n";
  }


  for(int i = 0; i < cv_points.size(); i++){
    uv_points.push_back(cv_points[i].x - (input.cols/2));
    uv_points.push_back(cv_points[i].y - (input.rows/2));
  }

  return uv_points;

}

double SamplePlugin::getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}


Q_EXPORT_PLUGIN(SamplePlugin);
