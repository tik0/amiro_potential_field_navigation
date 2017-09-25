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
#define button_width 120
#define text_label_width 180
#define slider_width 150
#define slider_label_info_width 50

#define line_edit_height 30
#define line_edit_width 200

static image_transport::Subscriber sub1, sub2;
static cv::Mat image1, image2;

using namespace std;

ImageViewGUI::ImageViewGUI(QWidget *parent) : QWidget(parent), shutdown_required(false), thread(&ImageViewGUI::spin, this) {


  textBox1 = new QLineEdit(this);
  textBox1->setGeometry(0, 0, line_edit_width, line_edit_height);
  textBox1->setText("text1");

  loadImageButton1 = new QPushButton(this);
  loadImageButton1->setGeometry(textBox1->pos().x()+textBox1->width(), 0, button_width, button_height);
  loadImageButton1->setText("loadImageButton1");
  QObject::connect(loadImageButton1, &QPushButton::clicked, this, &ImageViewGUI::clickedLoadImageButton1);

  transparencySliderLabel1 = new QLabel(this);
  transparencySliderLabel1->setGeometry(line_edit_width + button_width, 0, text_label_width, button_height);
  transparencySliderLabel1->setText("Transparency 0% to 100%");

  transparencySlider1 = new QSlider(Qt::Horizontal, this);
  transparencySlider1->setGeometry(line_edit_width + button_width + text_label_width, 0, slider_width, button_height);
  transparencySlider1->setMinimum(0);
  transparencySlider1->setMaximum(100);
  transparencySlider1->setSliderPosition(70);
  QObject::connect(transparencySlider1, &QSlider::sliderMoved, this, &ImageViewGUI::transparencySliderValueChanged1);

  transparencySliderLabelInfo1 = new QLabel(this);
  transparencySliderLabelInfo1->setGeometry(line_edit_width + button_width + text_label_width + slider_width, 0, slider_label_info_width, button_height);
  transparencySliderLabelInfo1->setText(QString::number(transparencySlider1->value()));

  textBox2 = new QLineEdit(this);
  textBox2->setGeometry(0, line_edit_height, line_edit_width, line_edit_height);
  textBox2->setText("text2");

  loadImageButton2 = new QPushButton(this);
  loadImageButton2->setGeometry(line_edit_width, line_edit_height, button_width, button_height);
  loadImageButton2->setText("loadImageButton2");
  QObject::connect(loadImageButton2, &QPushButton::clicked, this, &ImageViewGUI::clickedLoadImageButton2);

  transparencySliderLabel2 = new QLabel(this);
  transparencySliderLabel2->setGeometry(line_edit_width + button_width, button_height, text_label_width, button_height);
  transparencySliderLabel2->setText("Transparency 0% to 100%");

  transparencySlider2 = new QSlider(Qt::Horizontal, this);
  transparencySlider2->setGeometry(line_edit_width + button_width + text_label_width, button_height, slider_width, button_height);
  transparencySlider2->setMinimum(0);
  transparencySlider2->setMaximum(100);
  transparencySlider2->setSliderPosition(70);
  QObject::connect(transparencySlider2, &QSlider::sliderMoved, this, &ImageViewGUI::transparencySliderValueChanged2);

  transparencySliderLabelInfo2 = new QLabel(this);
  transparencySliderLabelInfo2->setGeometry(line_edit_width + button_width + text_label_width + slider_width, button_height, slider_label_info_width, button_height);
  transparencySliderLabelInfo2->setText(QString::number(transparencySlider2->value()));


  imageLabel1 = new QLabel(this);
  imageLabel1->setGeometry(0, 2 * line_edit_height, 1000, 1000);
  imageLabel1->setText(QString::fromStdString("image label1"));

  imageLabel2 = new QLabel(this);
  imageLabel2->setGeometry(0, 2 * line_edit_height, 1000, 1000);
  imageLabel2->setText(QString::fromStdString("image label2"));

  this->setWindowTitle(QString::fromStdString(ros::this_node::getName()));
}

ImageViewGUI::~ImageViewGUI() {
  delete textBox1;
  delete textBox2;
  delete loadImageButton1;
  delete loadImageButton2;
  delete transparencySlider1;
  delete transparencySlider2;
  delete transparencySliderLabel1;
  delete transparencySliderLabel2;
  delete transparencySliderLabelInfo1;
  delete transparencySliderLabelInfo2;
  delete imageLabel1;
  delete imageLabel2;

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

void ImageViewGUI::clickedLoadImageButton1() {
  ROS_INFO("[%s] clicked loadImageButton1", ros::this_node::getName().c_str());
}

void ImageViewGUI::clickedLoadImageButton2() {
  ROS_INFO("[%s] clicked loadImageButton2", ros::this_node::getName().c_str());
}

void ImageViewGUI::transparencySliderValueChanged1(int value) {
  this->transparencySliderLabelInfo1->setText(QString::number(value));
}

void ImageViewGUI::transparencySliderValueChanged2(int value) {
  this->transparencySliderLabelInfo2->setText(QString::number(value));
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
