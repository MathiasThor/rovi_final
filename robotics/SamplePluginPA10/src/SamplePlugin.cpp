#include "SamplePlugin.hpp"

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0,              SIGNAL(pressed()),      this, SLOT(btnPressed()) );
  connect(_startStopMovement, SIGNAL(pressed()),      this, SLOT(btnPressed()) );
  connect(_comboBox,          SIGNAL(activated(int)), this, SLOT(testFunc())   );
  connect(_followMarker,      SIGNAL(pressed()),      this, SLOT(btnPressed()) );


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
  timer();
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();
  _PA10 = _wc->findDevice("PA10");

  if (_PA10 == NULL) {
    cerr << "Device: PA10 not found!" << endl;
  }

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
		image = ImageLoader::Factory::load("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/markers/Marker3.ppm");
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	}
  else if(obj==_startStopMovement){
    stop_start_motion = !stop_start_motion;
    if (stop_start_motion){
      _timer->start(100); // run 10 Hz
      log().info() << "Start Motion\n";
    }
    else{
      _timer->stop();
      log().info() << "Stop Motion\n";
    }
	}
  else if(obj==_followMarker){
    /// TMP
    move_marker(marker_motion[current_motion_position]);
    if (current_motion_position == marker_motion.size())
      current_motion_position = 0;
    else
      current_motion_position++;
    /// TMP

    follow_marker();
    log().info() << "Follow Marker\n";
  }
}

void SamplePlugin::timer() {
	if (_framegrabber != NULL) {
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);

		// Show in QLabel
		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400;
		unsigned int maxH = 800;
		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}

  if ( stop_start_motion && !marker_motion.empty() ) {
    move_marker(marker_motion[current_motion_position]);
    if (current_motion_position == marker_motion.size())
      current_motion_position = 0;
    else
      current_motion_position++;
  }
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

void SamplePlugin::move_marker( rw::math::VelocityScrew6D<> p_6D ){
  MovableFrame* marker_frame = static_cast<MovableFrame*>(_wc->findFrame("Marker"));
  marker_frame->setTransform(Transform3D<double>( p_6D.linear(), RPY<double>(p_6D(3),	p_6D(4),	p_6D(5)).toRotation3D() ), _state);
  getRobWorkStudio()->setState(_state);
}

void SamplePlugin::testFunc() {
  current_motion_position = 0;
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

void SamplePlugin::follow_marker( ){
  double focal_length = 823;
  double z = 0.5;

  //
  // Get the transform of CAMARA frame relative to the MARKER frame. -OK
  //
  Transform3D<> camara_to_marker = _wc->findFrame("Marker")->fTf(_wc->findFrame("Camera"), _state);

  //
  // Calculate u, v, du and dv
  //
  Vector3D<> marker_midpoint = inverse(camara_to_marker) * Vector3D<>(0,0,0);
  const double u = ( marker_midpoint(0) * focal_length ) / z;
  const double v = ( marker_midpoint(1) * focal_length ) / z;

  Jacobian d_uv(2,1);
  d_uv(0,0) = u - u_old;
  d_uv(1,0) = v - v_old;
  u_old = u;
  v_old = v;
  log().info() << "v:\t" << v << "\n";
  log().info() << "u:\t" << u << "\n";
  log().info() << "d_uv:\n" << d_uv << "\n";

  //
  // Calculate the jacobian for PA10 -Ok
  //
	Jacobian J_PA10 = _PA10->baseJframe(_wc->findFrame("Camera"), _state);
  //log().info() << "j_pa10:\n" << J_PA10 << "\n";

  //
  // Calculate the image jacobian
  //
  Jacobian J_image(2,6);   // Create 6*2 Jacobian

  // Fill the jacobian
  J_image(0, 0) = -(focal_length / z);
  J_image(0, 1) = 0;
  J_image(0, 2) = u/z;
  J_image(0, 3) = u*v/focal_length;
  J_image(0, 4) = -(((focal_length*focal_length)+(u*u))/(focal_length));
  J_image(0, 5) = v;
  J_image(1, 0) = 0;
  J_image(1, 1) = -(focal_length / z);
  J_image(1, 2) = (v/z);
  J_image(1, 3) = (((focal_length*focal_length)+(v*v))/(focal_length));
  J_image(1, 4) = -((u*v)/(focal_length));
  J_image(1, 5) = -u;
  //log().info() << "J_img:\n" << J_image << "\n";

  //
  // Calculate Sq
  //
  Rotation3D<> cam_rotation = inverse(_PA10->baseTframe(_wc->findFrame("Camera"), _state)).R();
  Jacobian J_sq = Jacobian(cam_rotation);

  //
  // Calculate Z_image
  //
  Jacobian z_image = J_image * J_sq * J_PA10;
  //log().info() << "z:\n" << z_image << "\n";
  Jacobian z_image_T(7,2);
  for(int i=0; i < 7; i++){
    for(int j=0; j < 2; j++){
      z_image_T(i,j) = z_image(j,i);
    }
  }

  //
  // Calculate dq
  // TODO: Is it the use of .e() that fucked us up?
  Jacobian Y = (Jacobian)(z_image*z_image_T).e().inverse()*d_uv;
  Jacobian J_dq = z_image_T*Y;
  Q dq(7, J_dq(0,0) , J_dq(0,1) ,J_dq(0,2),J_dq(0,3),J_dq(0,4),J_dq(0,5),J_dq(0,6));

  // auto ytt = z_image.e().inverse() * d_uv.e();
  // Q dqqq(z_image.e().transpose()*ytt);
  // log().info() << "q:\t" << dqqq << "\n";
  // log().info() << "dq:\t" << dq << "\n";

  Q new_q(_PA10->getQ(_state)+dq);
  _PA10->setQ(new_q, _state);
  getRobWorkStudio()->setState(_state);

}

Q_EXPORT_PLUGIN(SamplePlugin);
