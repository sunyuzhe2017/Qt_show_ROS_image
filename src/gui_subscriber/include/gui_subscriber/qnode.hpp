/**
 * @file /include/gui_subscriber/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_subscriber_QNODE_HPP_
#define gui_subscriber_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include "main_window.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_subscriber {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void myCallback(const std_msgs::Float64& message_holder);
  void myCallback_img(const sensor_msgs::ImageConstPtr& msg);
  cv::Mat img;
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std_msgs::Float64 &msg);
Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void imageSignal(cv::Mat);

private:
	int init_argc;
	char** init_argv;
  //ros::Subscriber chatter_subscriber;
  image_transport::Subscriber image_sub;
  QStringListModel logging_model;
};

}  // namespace gui_subscriber

#endif /* gui_subscriber_QNODE_HPP_ */
