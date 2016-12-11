#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"

#include <opencv2/opencv.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/rw.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include <QPushButton>
#include <QTimer>
#include <QtPlugin>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <string>

using namespace rw::math;
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;
using namespace cv;
using namespace std;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin {

Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )

public:
	SamplePlugin();
	virtual ~SamplePlugin();
	virtual void open(rw::models::WorkCell* workcell);
	virtual void close();
	virtual void initialize();

private slots:
	void btnPressed();
	void load_motion( );
	void resetSim();
	void testRun();
	void timer();
	void set_dt( );
	void stateChangedListener(const rw::kinematics::State& state);

private:

	// Private funcs
	void move_marker( rw::math::VelocityScrew6D<> v6D );
	void follow_marker( vector<double> uv_points, bool cv );
	void velocityLimit( Q dq, Q &q );
	void writeToFile( );
	vector<double> cam_update( );
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
	vector<double> marker_detection(Mat &input);
	void tracking_error_task_space();
	void tracking_error_image_space();

	vector<double> target2{0,0,0,0,0,0};

	// Global variables
	vector< rw::math::VelocityScrew6D<double> > marker_motion;
	int current_motion_position = 0;
	bool stop_start_motion = false;
	bool test_runner = false;
	bool test_bool = true;
	vector<double> uv;
	double f = 823;
	double z = 0.5;
	double DT = 1;
	double last_tracking_error = 0;

	Q vel_limits;
	ofstream jointPos_file;
	ofstream toolPos_file;
	ofstream trackErr_file;

	//const string path = "/home/christian/Github_projects/";
	const string path = "/home/mat/7_semester_workspace/";

	int cv_choice = 1; 	// COLOR
	//int cv_choice = 2; 	// CORNY

	int numOfPoints = 1;
	bool cvOrFile = false;

	// vector<double> PT0{ 0.0,		0.0,		0};
	// vector<double> PT1{-0.1,		0.0,		0};
	// vector<double> PT2{ 0.0,	 -0.1,		0};

	//vector<double> PT0{0.054,		0.054,		0};
	//vector<double> PT1{-0.054,		0.054,		0};
  //vector<double> PT2{0.054,		-0.054,		0};

	vector<double> PT0{0.15,		0.15,		0};
	vector<double> PT1{-0.15,		0.15,		0};
	vector<double> PT2{0.15,		-0.15,		0};

	Device::Ptr _PA10;
	MovableFrame* _Marker;
	Frame* _Camera;

	QTimer* _timer;
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;

};

#endif /*SAMPLEPLUGIN_HPP*/
