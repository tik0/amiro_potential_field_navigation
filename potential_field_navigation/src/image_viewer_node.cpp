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

static image_transport::Subscriber sub1, sub2;
static cv::Mat image1, image2;

using namespace std;

ImageViewGUI::ImageViewGUI(QWidget *parent) : QWidget(parent), shutdown_required(false), thread(&ImageViewGUI::spin, this) {

  // 1st Row
  groupBox1 = new QGroupBox(this);
  groupBox1->setTitle("groupBox1");

  textBox1 = new QLineEdit(this);
  textBox1->setText("text1");

  loadImageButton1 = new QPushButton(this);
  loadImageButton1->setText("loadImageButton1");
  QObject::connect(loadImageButton1, &QPushButton::clicked, this, &ImageViewGUI::clickedLoadImageButton1);

  transparencySliderLabel1 = new QLabel(this);
  transparencySliderLabel1->setText("Transparency 0% to 100%");

  transparencySlider1 = new QSlider(Qt::Horizontal, this);
  transparencySlider1->setMinimum(0);
  transparencySlider1->setMaximum(100);
  transparencySlider1->setSliderPosition(70);
  QObject::connect(transparencySlider1, &QSlider::sliderMoved, this, &ImageViewGUI::transparencySliderValueChanged1);

  transparencySliderLabelInfo1 = new QLabel(this);
  transparencySliderLabelInfo1->setText(QString::number(transparencySlider1->value()));

  qhBox1 = new QHBoxLayout;
  qhBox1->addWidget(textBox1);
  qhBox1->addWidget(loadImageButton1);
  qhBox1->addWidget(transparencySliderLabel1);
  qhBox1->addWidget(transparencySlider1);
  qhBox1->addWidget(transparencySliderLabelInfo1);
  groupBox1->setLayout(qhBox1);

  // 2nd Row
  groupBox2 = new QGroupBox(this);
  groupBox2->setTitle("groupBox2");

  textBox2 = new QLineEdit(this);
  textBox2->setText("text2");

  loadImageButton2 = new QPushButton(this);
  loadImageButton2->setText("loadImageButton2");
  QObject::connect(loadImageButton2, &QPushButton::clicked, this, &ImageViewGUI::clickedLoadImageButton2);

  transparencySliderLabel2 = new QLabel(this);
  transparencySliderLabel2->setText("Transparency 0% to 100%");

  transparencySlider2 = new QSlider(Qt::Horizontal, this);
  transparencySlider2->setMinimum(0);
  transparencySlider2->setMaximum(100);
  transparencySlider2->setSliderPosition(70);
  QObject::connect(transparencySlider2, &QSlider::sliderMoved, this, &ImageViewGUI::transparencySliderValueChanged2);

  transparencySliderLabelInfo2 = new QLabel(this);
  transparencySliderLabelInfo2->setText(QString::number(transparencySlider2->value()));

  qhBox2 = new QHBoxLayout;
  qhBox2->addWidget(textBox2);
  qhBox2->addWidget(loadImageButton2);
  qhBox2->addWidget(transparencySliderLabel2);
  qhBox2->addWidget(transparencySlider2);
  qhBox2->addWidget(transparencySliderLabelInfo2);
  groupBox2->setLayout(qhBox2);

  // merge row1 and row2
  qvBox1 = new QVBoxLayout;
  qvBox1->addWidget(groupBox1);
  qvBox1->addWidget(groupBox2);
  groupBox0 = new QGroupBox(this);
  groupBox0->setTitle("groupBox0");
  groupBox0->setLayout(qvBox1);
  QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  groupBox0->setSizePolicy(sizePolicy);

//   Image Labels
  imageLabel1 = new QLabel(this);
  imageLabel1->setText(QString::fromStdString("image label1"));

//  imageLabel2 = new QLabel(this);
//  imageLabel2->setText(QString::fromStdString("image label2"));

  // Total Gui
  this->setWindowTitle(QString::fromStdString(ros::this_node::getName()));
  gridLayout1 = new QGridLayout();
  gridLayout1->addWidget(groupBox0, 0, 0);
  gridLayout1->addWidget(imageLabel1, 1, 0);
//  gridLayout1->addWidget(imageLabel2, 1, 0);
  this->setLayout(gridLayout1);
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
  delete groupBox0;
  delete groupBox1;
  delete groupBox2;
  delete gridLayout1;
  delete qvBox1;
  delete qhBox1;
  delete qhBox2;
  delete imageLabel1;
//  delete imageLabel2;

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

void ImageViewGUI::updateImages() {
  // Display the image
  if (image1Loaded && image2Loaded) {
    if(image1.size != image2.size)
      cv::resize(image2, image2, image1.size());
    cv::Mat blended;
    cv::addWeighted(image1, transparencySlider1->value()/100.0, image2, transparencySlider2->value()/100.0, 0.0, blended);
    QImage qtImage1 = mat2QImage(blended);
    imageLabel1->setPixmap(QPixmap::fromImage(qtImage1));
    imageLabel1->show();
  }
}

void ImageViewGUI::clickedLoadImageButton1() {
  image1 = cv::imread("/homes/drudolph/lena.jpg", CV_LOAD_IMAGE_COLOR);
  image1Loaded = true;
  updateImages();
  ROS_INFO("[%s] clicked loadImageButton1", ros::this_node::getName().c_str());
}

void ImageViewGUI::clickedLoadImageButton2() {
  image2 = cv::imread("/homes/drudolph/lena2.jpg", CV_LOAD_IMAGE_COLOR);
  image2Loaded = true;
  updateImages();
  ROS_INFO("[%s] clicked loadImageButton2", ros::this_node::getName().c_str());
}

void ImageViewGUI::transparencySliderValueChanged1(int value) {
  updateImages();
  this->transparencySliderLabelInfo1->setText(QString::number(value));
}

void ImageViewGUI::transparencySliderValueChanged2(int value) {
  updateImages();
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
