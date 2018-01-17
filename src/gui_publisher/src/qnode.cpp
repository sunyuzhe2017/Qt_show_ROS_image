/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/gui_publisher/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

double PI = 3.1415926;
namespace gui_publisher {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui_publisher");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  //自己加的一行信息
  chatter_publisher = n.advertise<std_msgs::Float64>("chatter", 1000);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"gui_publisher");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  chatter_publisher = n.advertise<std_msgs::Float64>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(5);
  std_msgs::Float64 velocity;//创建一个Float64类型的变量
  double amplitude = 1.0;
  double frequency = 5.0;
  int dt = 0;
  //int count = 0;
	while ( ros::ok() ) {
  velocity.data = amplitude*sin(frequency*PI/180*dt);
  dt++;
  //ROS_INFO("Sending data %f",velocity.data);
  chatter_publisher.publish(velocity);
    //std_msgs::String msg;
    std::stringstream ss;
    //ss << "hello world " << count;
    ss << "Velocity " << velocity.data;
    //msg.data = ss.str();
    //chatter_publisher.publish(msg);
    chatter_publisher.publish(velocity);
    std::string s(ss.str());
    //log(Info,std::string("I sent: ")+msg.data);
    log(Info,std::string("I sent: ")+s);

		ros::spinOnce();
		loop_rate.sleep();
    //++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace gui_publisher
