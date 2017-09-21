// ============================================================================
// Name        : image_viewer_gui.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Creates a GUI to view two different images.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "image_viewer_gui.hpp"


#define button_height 30
#define button_width 100

#define line_edit_height 30
#define line_edit_width 200

static image_transport::Subscriber sub1, sub2;
static cv::Mat image1, image2;

using namespace std;

ImageViewGUI::ImageViewGUI(QWidget *parent) : QWidget(parent), shutdown_required(false), thread(&ImageViewGUI::spin, this) {


  image_label = new QLabel(this);
  image_label->setGeometry(200, 0, 100, 100);
  image_label->setText(QString::fromStdString("image label"));

  text1 = new QLineEdit(this);
  text1->setGeometry(0, 0, line_edit_width, line_edit_height);
  text1->setText("text1");

  text2 = new QLineEdit(this);
  text2->setGeometry(0, line_edit_height, line_edit_width, line_edit_height);
  text2->setText("text2");


  this->setMinimumSize(500, 500);
  this->setWindowTitle(QString::fromStdString(ros::this_node::getName()));
}

ImageViewGUI::~ImageViewGUI() {
  delete text1;
  delete text2;

  shutdown_required = true;
  thread.join();
}

void ImageViewGUI::spin() {
  ros::Rate loop(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }
  ROS_INFO("[%s] Shutdown this node.", ros::this_node::getName().c_str());
  ros::shutdown();
  QApplication::quit();
}


void imageCallback1(const sensor_msgs::ImageConstPtr &msg) {
  image1 = cv_bridge::toCvShare(msg, "bgr8")->image;
}

void imageCallback2(const sensor_msgs::ImageConstPtr &msg) {
  image2 = cv_bridge::toCvShare(msg, "bgr8")->image;
}

int main(int argc, char *argv[]) {
  ROS_INFO_STREAM("Start: " << ros::this_node::getName());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

  image_transport::ImageTransport it(node);
  sub1 = it.subscribe("camera/image", 1, imageCallback1);
  sub2 = it.subscribe("camera/image", 1, imageCallback2);


  QApplication app(argc, argv);
  ImageViewGUI gui;
  gui.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  return app.exec();
}
