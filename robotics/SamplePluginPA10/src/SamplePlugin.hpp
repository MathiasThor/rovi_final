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

	void move_marker( rw::math::VelocityScrew6D<> v6D );
	//void load_motion( std::string motion_type );
	vector< rw::math::VelocityScrew6D<double> > marker_motion;

private slots:
	void btnPressed();
	void testFunc();
	void timer();
	void stateChangedListener(const rw::kinematics::State& state);

private:
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
	QTimer* _timer;
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;

};

#endif /*SAMPLEPLUGIN_HPP*/
