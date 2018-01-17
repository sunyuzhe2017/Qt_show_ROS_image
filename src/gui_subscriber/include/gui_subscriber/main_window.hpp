/**
 * @file /include/gui_subscriber/main_window.hpp
 *
 * @brief Qt based gui for gui_subscriber.
 *
 * @date November 2010
 **/
#ifndef gui_subscriber_MAIN_WINDOW_H
#define gui_subscriber_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui_subscriber {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class QNode;
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing
	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void on_pushButton_back_clicked(bool checked);
    void on_pushButton_clicked();
    void displayMat(cv::Mat image);
private:
	Ui::MainWindowDesign ui;
  QNode qnode;
  cv::Mat image;
};

}  // namespace gui_subscriber

#endif // gui_subscriber_MAIN_WINDOW_H
