// ============================================================================
// Name        : image_selecter_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Creates a GUI to select a image for publishing it.
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

#include "image_selecter_gui.hpp"


#define button_height 30
#define button_width 100

using namespace std;

static image_transport::Publisher imagePublisher;
static int loadAsBlueChannel;
static int convertToRedChannel;
static int chargeAsCurrent;

GUI::GUI(QWidget *parent) : QWidget(parent), shutdown_required(false), thread(&GUI::spin, this) {

  image_label = new QLabel(this);
  image_label->setGeometry(200, 0, 100, 100);
  image_label->setText(QString::fromStdString("image label"));

  image_selecter = new QPushButton("select image", this);
  image_selecter->setGeometry(0, 0, button_width, button_height);
  QObject::connect(image_selecter, &QPushButton::clicked, this, &GUI::selectImage);

  publish_image = new QPushButton("publish image", this);
  publish_image->setGeometry(100, 0, button_width, button_height);
  QObject::connect(publish_image, &QPushButton::clicked, this, &GUI::publishImage);

  checkboxLoadAsGray = new QCheckBox("as blue ch.", this);
  checkboxLoadAsGray->setGeometry(200, 0, button_width, button_height);
  if (loadAsBlueChannel) {
    checkboxLoadAsGray->setChecked(true);
  } else {
    checkboxLoadAsGray->setChecked(false);
  }

  checkboxInvGray = new QCheckBox("to red ch.", this);
  checkboxInvGray->setGeometry(300, 0, button_width, button_height);
  if (convertToRedChannel) {
    checkboxInvGray->setChecked(true);
  } else {
    checkboxInvGray->setChecked(false);
  }

  checkboxSendAsCurrent = new QCheckBox("as current", this);
  checkboxSendAsCurrent->setGeometry(400, 0, button_width, button_height);
  if (chargeAsCurrent) {
    checkboxSendAsCurrent->setChecked(true);
  } else {
    checkboxSendAsCurrent->setChecked(false);
  }

  image_label->setBackgroundRole(QPalette::Base);
  image_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  image_label->setScaledContents(true);

  this->setMinimumSize(500, 500);
  this->setWindowTitle(QString::fromStdString(ros::this_node::getName()));

  label = new QLabel(this);
}

GUI::~GUI() {
  delete label;
  delete checkboxSendAsCurrent;
  delete checkboxLoadAsGray;
  delete checkboxInvGray;
  delete publish_image;
  delete image_selecter;
  delete image_label;

  shutdown_required = true;
  thread.join();
}

void GUI::spin() {
  ros::Rate loop(10);
  cout << "debug 1" << endl;
  while (ros::ok()) {
    cout << "debug 2" << endl;
    ros::spinOnce();
    loop.sleep();
  }
  ROS_INFO("[%s] Shutdown this node.", ros::this_node::getName().c_str());
  ros::shutdown();
  QApplication::quit();
}

void GUI::selectImage() {
  QString fileName = QFileDialog::getOpenFileName(this, QString::fromStdString("Open image"), QString::fromStdString("../patter/"), QString::fromStdString("Image Files (*.png *.jpg *.jpeg *.bmp)"));

  // Check if loading as grayscale
  if (checkboxLoadAsGray->checkState() == Qt::CheckState::Checked) {
    cv_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
  } else {
    cv_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_COLOR);
  }

  // Invert the color channel if necessary
  if (cv_image.channels() == 1) {
    ROS_INFO_STREAM(ros::this_node::getName() << " loaded gray scale image");
    std::vector<cv::Mat> array_to_merge(3);
    cv::Mat dummy(cv_image.size(), cv_image.type(), cv::Scalar(uchar(0)));
    if (checkboxInvGray->checkState() == Qt::CheckState::Checked) {
      cv::Mat dummy(cv_image.size(), cv_image.type(), cv::Scalar(uchar(0)));
      dummy.copyTo(array_to_merge.at(0));
      dummy.copyTo(array_to_merge.at(1));
      cv_image.copyTo(array_to_merge.at(2));
    } else {
      dummy.copyTo(array_to_merge.at(2));
      dummy.copyTo(array_to_merge.at(1));
      cv_image.copyTo(array_to_merge.at(0));
    }
    cv::merge(array_to_merge, cv_image);
  } else if (cv_image.channels() == 3) {
    ROS_INFO_STREAM(ros::this_node::getName() << " loaded BGR image");
  } else {
    ROS_ERROR_STREAM(ros::this_node::getName() << " Unsupported number of channels");
    return;
  }

  // Display the image
  cv::Mat rgb;
  cv::cvtColor(cv_image, rgb, CV_BGR2RGB);
  qt_image = mat2QImage(rgb);

  label->setPixmap(QPixmap::fromImage(qt_image));
  image_label = label;
  this->setMinimumSize(qt_image.width(), qt_image.height() + button_height);
  image_label->setGeometry(0, button_height, qt_image.width(), qt_image.height() + button_height);
  image_label->show();
  imageSelected = true;
}

void GUI::publishImage() {
  if (!imageSelected) {
    ROS_WARN("[%s] Please selected an image first.", ros::this_node::getName().c_str());
    return;
  }
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = cv_image;
  if (checkboxSendAsCurrent->checkState() == Qt::CheckState::Checked) {
    std::cerr << "Send as current" << std::endl;
    cvImage.header.frame_id = "current";
  } else {
    cvImage.header.frame_id = "charge";
    std::cerr << "Send as charge" << std::endl;
  }
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO_STREAM("Start: " << ros::this_node::getName());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

// Ros Topics
  string rosPublisherTopic;

  node.param<string>("image_publisher_topic", rosPublisherTopic, "/image");
  node.param<int>("load_as_blue_channel", loadAsBlueChannel, 1);
  node.param<int>("convert_to_red_channel", convertToRedChannel, 0);
  node.param<int>("charge_as_current", chargeAsCurrent, 0);
  ROS_INFO("[%s] image_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopic.c_str());

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(rosPublisherTopic, 1);

  QApplication app(argc, argv);
  GUI gui;
  gui.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  return app.exec();
}
