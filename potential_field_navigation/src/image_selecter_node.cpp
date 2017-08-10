// ============================================================================
// Name        : image_selecter_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
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

// Qt5 Widgets
#include <QWidget>
#include <QMainWindow>
#include <QFileDialog>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include "image_selecter_gui.hpp"

#define button_height 30
#define button_width 100

using namespace std;


// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;
static image_transport::Publisher imageRGBAPublisher;

// program name
const string programName = "image_selecter_node";

cv::Mat cv_image;
QImage qt_image;

bool imageSelected = false;

GUI::GUI(QWidget *parent) : QWidget(parent) {

  QLabel *image_label = new QLabel(this);
  image_label->setGeometry(200, 0, 100, 100);
  image_label->setText(QString::fromStdString("image label"));

  QPushButton *image_selecter = new QPushButton("select image", this);
  image_selecter->setGeometry(0, 0, button_width, button_height);
  QObject::connect(image_selecter, &QPushButton::clicked, this, &GUI::selectImage);

  QPushButton *publish_image = new QPushButton("publish image", this);
  publish_image->setGeometry(100, 0, button_width, button_height);
  QObject::connect(publish_image, &QPushButton::clicked, this, &GUI::publishImage);


  image_label->setBackgroundRole(QPalette::Base);
  image_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  image_label->setScaledContents(true);

  this->setMinimumSize(500, 500);
  this->setWindowTitle(QString::fromStdString(programName));
}

void GUI::selectImage() {
  QString fileName = QFileDialog::getOpenFileName(this, QString::fromStdString("Open image"), QString::fromStdString("../patter/"), QString::fromStdString("Image Files (*.png *.jpg *.jpeg *.bmp)"));

  cv_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);

  qt_image.load(fileName);
  QLabel *label = new QLabel(this);
  label->setPixmap(QPixmap::fromImage(qt_image));
  image_label = label;
  this->setMinimumSize(qt_image.width(), qt_image.height() + button_height);
  image_label->setGeometry(0, button_height, qt_image.width(), qt_image.height() + button_height);
  image_label->show();
  imageSelected = true;
}

void GUI::publishImage() {
  if (!imageSelected) {
    ROS_WARN("[%s] Please seletect an image first.", programName.c_str());
    return;
  }
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::MONO8;
  cvImage.image = cv_image;
  imagePublisher.publish(cvImage.toImageMsg());

  //TODO
  // Publish image aswell as RGB for visualization
  cv::Mat rgbaImage;
  cv::cvtColor(cv_image,rgbaImage, CV_GRAY2RGBA);
  cv_bridge::CvImage cvImageRgba;
//  cvImageRgba.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
  cvImageRgba.encoding = sensor_msgs::image_encodings::BGRA8;
  cvImageRgba.image = rgbaImage;
  imageRGBAPublisher.publish(cvImageRgba.toImageMsg());
}

QImage mat2QImage(const cv::Mat &mat) {
  return QImage((const unsigned char *) (mat.data), mat.cols, mat.rows, QImage::Format_Mono);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

// Ros Topics
  string rosPublisherTopic;
  string rosPublisherTopicRgba;

  node.param<string>("image_publisher_topic", rosPublisherTopic, "/image");
  node.param<string>("image_publisher_topic_rgba", rosPublisherTopicRgba, "/image/rgba");
  ROS_INFO("[%s] image_publisher_topic: %s", programName.c_str(), rosPublisherTopic.c_str());
  ROS_INFO("[%s] image_publisher_topic_rgb: %s", programName.c_str(), rosPublisherTopicRgba.c_str());

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(rosPublisherTopic, 1);
  imageRGBAPublisher = imageTransport.advertise(rosPublisherTopicRgba, 1);

  QApplication app(argc, argv);

  GUI gui;
  gui.show();

  return app.exec();
}
