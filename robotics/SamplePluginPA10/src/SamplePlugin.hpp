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
#include <ctime>

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
	void writeToFile();
	vector<double> cam_update( bool get_points );
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
	vector<double> marker_detection(Mat &input);
	void tracking_error_task_space();
	void tracking_error_image_space( vector<double> uv_points );
	double getUnixTime(void);
	void predictor();

	// Global variables
	vector< rw::math::VelocityScrew6D<double> > marker_motion;
	int current_motion_position = 0;
	bool stop_start_motion = false;
	bool test_runner = false;
	vector<double> uv;
	vector<double> uv_old;
	vector<double> prediction{0,0,0,0,0,0};
	vector<double> desired{0,0,0,0,0,0};
	double f = 823;
	double z = 0.50;
	double DT = 1;
	double marker_dt = 0;
	double highest_tracking_error = 0;

	Q vel_limits;
	ofstream jointPos_file;
	ofstream toolPos_file;
	ofstream trackErr_file;

//const string path = "/home/christian/Github_projects/";
	const string path = "/home/mat/7_semester_workspace/";
//const string path = "/your_path_to_the_folder_w._samplepluginpa10/";

	int cv_choice = 1; 			// FOLLOW MARKER 1 (COLOR)
//int cv_choice = 2; 			// TRACK MARKER 4 (CORNY)

//int numOfPoints = 1;		// TRACK MIDTPOINT
	int numOfPoints = 3;		// TRACK PT0, PT1, PT2

//bool cvOrFile = true;	  // TRUE = USE POINTS FROM CV
	bool cvOrFile = false;	// FALSE = USE PT0, PT1, PT2 or MIDPOINT

//double kappa = 0.0;			// KAPPA FOR DAMPED LEAST SQUARE - WITHOUT CV AND NEW POINTS
	double kappa = 1.2;			// KAPPA FOR DAMPED LEAST SQUARE - WITHOUT CV
//double kappa = 3.0;			// KAPPA FOR DAMPED LEAST SQUARE - FOR COLOR MARKER TRACKING

	// ORIGINAL POINTS TO TRACK (needs kappa = 1.2)
	vector<double> PT0{ 0.0,		0.0,		0};
	vector<double> PT1{-0.1,		0.0,		0};
	vector<double> PT2{ 0.0,	 -0.1,		0};

	// NEW POINTS TO TRACK (can have kappa = 0)
	// vector<double> PT0{0.15,		0.15,		0};
	// vector<double> PT1{-0.15,	0.15,		0};
	// vector<double> PT2{0.15,		-0.15,	0};

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
