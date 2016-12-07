#include "SamplePlugin.hpp"
#include "./vision_header_files/setup.h"
#include "./vision_source_files/color_detector.cpp"


SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0,              SIGNAL(pressed()),            this, SLOT(btnPressed()) );
  connect(_startStopMovement, SIGNAL(pressed()),            this, SLOT(btnPressed()) );
  connect(_comboBox,          SIGNAL(activated(int)),       this, SLOT(testFunc())   );
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
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/mat/7_semester_workspace/rovi_final/robotics/PA10WorkCell/ScenePA10RoVi1.wc.xml");
  if (wc == NULL) {
    cerr << "WorkCell: not found!" << endl;
  }
	getRobWorkStudio()->setWorkCell(wc);
  load_motion("MarkerMotionSlow.txt");
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
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Change Texture\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	}
  else if(obj==_startStopMovement){
    stop_start_motion = !stop_start_motion;
    if (stop_start_motion){
      log().info() << "Start Motion\n";
      _timer->start(100); // run 10 Hz
    }
    else{
      log().info() << "Stop Motion\n";
      _timer->stop();
    }
	}
  else if(obj==_followMarker){
    log().info() << "Follow Marker\n";
    timer();
  }
}

void SamplePlugin::timer() {
  vector<cv::Point> reference_points;
  if (_framegrabber != NULL) {
    // Get the image as a RW image
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    _framegrabber->grab(cameraFrame, _state);
    const Image& image = _framegrabber->getImage();

    // Convert to OpenCV image
    Mat im = toOpenCVImage(image);
    color_detector(im, reference_points);
    for(int i = 0; i < reference_points.size(); i++){
      Point temp = reference_points[i];
      temp.x = reference_points[i].x - (im.cols/2);
      temp.y = reference_points[i].y - (im.rows/2);
      reference_points[i] = temp;
    }

    Mat imflip;
    cv::flip(im, imflip, 0);
    cv::circle(imflip, cv::Point(imflip.cols/2+target[0*2], imflip.rows/2+target[0*2+1]), 15, cv::Scalar(0,255,0), 4);
    cv::circle(imflip, cv::Point(imflip.cols/2+target[1*2], imflip.rows/2+target[1*2+1]), 15, cv::Scalar(0,0,255), 4);
    cv::circle(imflip, cv::Point(imflip.cols/2+target[2*2], imflip.rows/2+target[2*2+1]), 15, cv::Scalar(50,50,50), 4);

    cv::circle(imflip, cv::Point(imflip.cols/2+uv[0*2],imflip.rows/2+uv[0*2+1]), 10, cv::Scalar(0,255,0),   -1);
    cv::circle(imflip, cv::Point(imflip.cols/2+uv[1*2],imflip.rows/2+uv[1*2+1]), 10, cv::Scalar(0,0,255),   -1);
    cv::circle(imflip, cv::Point(imflip.cols/2+uv[2*2],imflip.rows/2+uv[2*2+1]), 10, cv::Scalar(50,50,50),  -1);

    for(int i = 0; i < reference_points.size(); i++){
      cv::circle(imflip, reference_points[i], 10, cv::Scalar(255,0,0), -1);
    }

    // for (int i = 0; i < 3; i++) { // TODO 3 = numOfPoints....
    //   cv::circle(imflip, cv::Point(imflip.cols/2+uv[i*2],imflip.rows/2+uv[i*2+1]), 10, cv::Scalar(255,0,0), -1);
    // }

    // Show in QLabel
    QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
  }

  if (current_motion_position == marker_motion.size()){
    resetSim();
    if (current_motion_position == marker_motion.size()){
      test_runner = false;
      jointPos_file.close();
      toolPos_file.close();
      log().info() << "Closed the files" << "\n";
    }
  }
  else {
    follow_marker( reference_points, false );
    move_marker(marker_motion[current_motion_position]);
    current_motion_position++;
    if (test_runner)
      writeToFile();
  }

}

void SamplePlugin::testRun(){
  jointPos_file.open ("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/data/joint_positions.txt");
  toolPos_file.open ("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/data/tool_positions.txt");
  log().info() << "Opened the files" << "\n";
  jointPos_file << DT << "\n";
  if (!test_runner) {
    test_runner=true;
    resetSim();
    _timer->start(100);
    log().info() << "Collecting data..." << "\n";
  }
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

void SamplePlugin::move_marker( rw::math::VelocityScrew6D<> p_6D ){
  _Marker->setTransform(Transform3D<double>( p_6D.linear(), RPY<double>(p_6D(3),	p_6D(4),	p_6D(5)).toRotation3D() ), _state);
  getRobWorkStudio()->setState(_state);
}

void SamplePlugin::testFunc() {
  resetSim();
  switch (_comboBox->currentIndex()) {
    case 0:
      load_motion("MarkerMotionSlow.txt");
      break;
    case 1:
      load_motion("MarkerMotionMedium.txt");
      break;
    case 2:
      load_motion("MarkerMotionFast.txt");
      break;
  }
}

void SamplePlugin::load_motion( string move_file ){
  marker_motion.clear();
  string move_file_path = "/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/motions/" + move_file;
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
    log().info() << "Loaded: " << move_file << "\n";
  }
}

void SamplePlugin::follow_marker( vector<Point> &reference_points, bool cv){
  //
  // Calculate u, uv[1], du and dv
  //
  if(cv){
    for (int i = 0; i < reference_points.size(); i++) { // TODO 3 = numOfPoints
      //cout << points[i][0] << "\t" << points[i][1] << endl;
      uv[i]   = ( reference_points[i].x * f ) / z;
      uv[i+1] = ( reference_points[i].y * f ) / z;
    }
  }
  else{
    // Get the transform of CAMARA frame relative to the MARKER frame. -OK
    Transform3D<> camara_to_marker = inverse(_Marker->fTf(_Camera, _state));
    vector< Vector3D<> > points;

    points.push_back(camara_to_marker.R() * Vector3D<>(PT0[0],PT0[1],PT0[2]) + camara_to_marker.P());
    if ( numOfPoints > 1) {
         points.push_back(camara_to_marker.R() * Vector3D<>(PT1[0],PT1[1],PT1[2]) + camara_to_marker.P());
         points.push_back(camara_to_marker.R() * Vector3D<>(PT2[0],PT2[1],PT2[2]) + camara_to_marker.P());
    }

    for (int i = 0; i < points.size(); i++) { // TODO 3 = numOfPoints
      uv[i*2]   = ( points[i][0] * f ) / z;
      uv[i*2+1] = ( points[i][1] * f ) / z;
    }

    if (current_motion_position==0) {
      target=uv;
      // for(double& element : target)
      //   element = element - uv[1];
    }
  }

  vector<double> target2 = {(PT0[0]*f)/z,(PT0[1]*f)/z,(PT1[0]*f)/z,(PT1[1]*f)/z,(PT2[0]*f)/z,(PT2[1]*f)/z};


  Jacobian d_uv(numOfPoints*2,1);
  for (int i = 0; i < numOfPoints; i++) {
    d_uv(i*2,0)   = target[i*2]-uv[i*2];
    d_uv(i*2+1,0) = target[i*2+1]-uv[i*2+1];
  }
  log().info() << "Frame:\t" << current_motion_position << "\n";

  log().info() << "uv:\t" << uv[0] << "\t" << uv[1] << "\t";
  if ( numOfPoints > 1)
      log().info() << uv[2] << "\t" << uv[3] << "\t" << uv[4] << "\t" << uv[5] << "\n";
  log().info() << "targ:\t" << target[1] << " " << target[1] << "\t" << target[2] << " " << target[3] << "\t" << target[4] << " " << target[5] << "\n";
  log().info() << "targ2:\t" << target2[1] << " " << target2[1] << "\t" << target2[2] << " " << target2[3] << "\t" << target2[4] << " " << target2[5] << "\n";
  log().info() << "d_uv:\t" << d_uv(0,0) << "\t" << d_uv(1,0) << "\t";
  if ( numOfPoints > 1)
    log().info() << d_uv(2,0) << "\t" << d_uv(3,0) << "\t" << d_uv(4,0) << "\t" << d_uv(5,0) << "\n";


  //
  // Calculate the jacobian for PA10 -Ok
  //
	Jacobian J_PA10 = _PA10->baseJframe(_Camera, _state);
  //log().info() << "j_pa10:\n" << J_PA10 << "\n";

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
  //log().info() << "J_img:\n" << J_image << "\n";

  //
  // Calculate Sq
  //
  Transform3D<> base2cam = inverse(_PA10->baseTframe(_Camera, _state));
  Jacobian J_sq = Jacobian(base2cam.R());
  //log().info() << "sq:\n" << J_sq << "\n";

  //
  // Calculate Z_image
  //
  Jacobian z_image = J_image * J_sq * J_PA10;
  //log().info() << "z:\n" << z_image << "\n";
  Jacobian z_image_T(7,numOfPoints*2);
  for(int i=0; i < 7; i++){
    for(int j=0; j < 2*numOfPoints; j++){
      z_image_T(i,j) = z_image(j,i);
    }
  }

  //
  // Calculate dq
  //
  Jacobian J_dq(z_image_T.e()*((z_image*z_image_T).e().inverse()*d_uv.e()));
  Q dq(J_dq.e());
  //log().info() << "dq:\t" << dq << "\n";
  Q new_q(_PA10->getQ(_state));
  velocityLimit(dq,new_q);
  _PA10->setQ(new_q, _state);
  getRobWorkStudio()->setState(_state);
  log().info() << "===========================================================" << "\n";
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
}


Q_EXPORT_PLUGIN(SamplePlugin);
