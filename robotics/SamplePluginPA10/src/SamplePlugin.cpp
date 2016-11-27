#include "SamplePlugin.hpp"

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox ,SIGNAL(valueChanged(int)), this, SLOT(testFunc()) );

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

void SamplePlugin::move_marker( rw::math::VelocityScrew6D<> p_6D ){
  MovableFrame* marker_frame = static_cast<MovableFrame*>(_wc->findFrame("Marker"));
  // cout << p_6D(3) << " : " <<	p_6D(4) << " : " <<	p_6D(5) << endl;
  // cout << p_6D.angular() << endl;
  // cout << p_6D.linear() << endl;
  marker_frame->setTransform(Transform3D<double>( p_6D.linear(), RPY<double>(p_6D(3),	p_6D(4),	p_6D(5)).toRotation3D() ), _state);
  getRobWorkStudio()->setState(_state);
}

void SamplePlugin::testFunc() {
  switch (_spinBox->value()) {
    case 1:
      cout << "NUMBER 1" << endl;
      move_marker(VelocityScrew6D<> (0, -0.819, 1.649, 0,	0,	-1.571));
      break;
    case 2:
      cout << "NUMBER 2" << endl;
      move_marker(VelocityScrew6D<> (0.5, -0.819, 1.649, 0,	0,	-1.571));
      break;
    case 3:
      cout << "NUMBER 3" << endl;
      move_marker(VelocityScrew6D<> (-0.5, -0.819, 1.649, 0,	0,	-1.571));
      break;
    case 4:
      cout << "NUMBER 4" << endl;
      move_marker(VelocityScrew6D<> (0, -0.819, 0.5, 0,	0,	-1.571));
      load_motion("MarkerMotionFast.txt");
      break;
  }
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

	// Load Lena image
	Mat im, image;
	im = imread("/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/markers/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();

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
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(100); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
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
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

void SamplePlugin::load_motion( string move_file ){

  string move_file_path = "/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/motions/" + move_file;
  ifstream motion_file(move_file_path.c_str()); // the input file
  string input;
  VelocityScrew6D<> pos_6D;
  int tabs = 1;

  if (motion_file.is_open()) {
    while ( getline (motion_file, input,'\t') ) {
      switch (tabs) {
        case 1:
          pos_6D(0) = atof(input.c_str());
          break;
        case 2:
          pos_6D(1) = atof(input.c_str());
          break;
        case 3:
          pos_6D(2) = atof(input.c_str());
          break;
        case 4:
          pos_6D(3) = atof(input.c_str());
          break;
        case 5:
          pos_6D(4) = atof(input.c_str());
          break;
        case 6:
          pos_6D(5) = atof(input.c_str());
          tabs = 0;
          marker_motion.push_back(pos_6D);
          cout << pos_6D << endl;
          break;
      }
      //cout << input << endl;
      tabs ++;
    }
    motion_file.close();
    cout << "Loaded: " << move_file << endl;
  }

}

Q_EXPORT_PLUGIN(SamplePlugin);
