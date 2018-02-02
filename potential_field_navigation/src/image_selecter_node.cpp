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

using namespace std;

static image_transport::Publisher imagePublisher;
static int loadAsMono;
static int monoAsRed;
static int chargeAsCurrent;
static bool imageMode;
static int videoDelay;

static cv::VideoCapture cap;

ImageSelecterGUI::ImageSelecterGUI(QWidget *parent) : QWidget(parent), shutdown_required(false), thread(&ImageSelecterGUI::spin, this) {

  // Image Gui Stuff
  image_label = new QLabel(this);
//  image_label->setText(QString::fromStdString("image label"));

  image_selecter = new QPushButton("select image", this);
  QObject::connect(image_selecter, &QPushButton::clicked, this, &ImageSelecterGUI::selectImage);

  publish_image = new QPushButton("publish image", this);
  QObject::connect(publish_image, &QPushButton::clicked, this, &ImageSelecterGUI::publishImage);
  publish_image->setEnabled(false);

  checkboxLoadAsGray = new QCheckBox("image as mono", this);
  if (loadAsMono) {
    checkboxLoadAsGray->setChecked(true);
  } else {
    checkboxLoadAsGray->setChecked(false);
  }

  radioButtonRed = new QRadioButton("mono as red");
  radioButtonBlue = new QRadioButton("mono as blue");
  if (monoAsRed) {
    radioButtonBlue->setChecked(false);
    radioButtonRed->setChecked(true);
  } else {
    radioButtonBlue->setChecked(true);
    radioButtonRed->setChecked(false);
  }

  checkboxSendAsCurrent = new QCheckBox("as current", this);
  if (chargeAsCurrent) {
    checkboxSendAsCurrent->setChecked(true);
  } else {
    checkboxSendAsCurrent->setChecked(false);
  }
  
  checkImageMode = new QCheckBox(this);
  checkImageMode->setChecked(imageMode);
  QObject::connect(checkImageMode, &QCheckBox::clicked, this, &ImageSelecterGUI::toggleModeImage);

  // Image QhBox
  qhBoxImage = new QHBoxLayout;
  qhBoxImage->addWidget(checkImageMode);
  qhBoxImage->addWidget(image_selecter);
  qhBoxImage->addWidget(publish_image);
  qhBoxImage->addWidget(checkboxLoadAsGray);
  qhBoxImage->addWidget(radioButtonRed);
  qhBoxImage->addWidget(radioButtonBlue);
  qhBoxImage->addWidget(checkboxSendAsCurrent);
  groupBoxImage = new QGroupBox(this);
  groupBoxImage->setTitle("Image Parameter");
  groupBoxImage->setLayout(qhBoxImage);
  QSizePolicy sizePolicyImage(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  groupBoxImage->setSizePolicy(sizePolicyImage);

  // Video Gui Stuff
  checkVideoMode = new QCheckBox(this);
  checkVideoMode->setChecked(!imageMode);
  QObject::connect(checkVideoMode, &QCheckBox::clicked, this, &ImageSelecterGUI::toggleModeVideo);

  videoSelecter = new QPushButton("select video", this);
  QObject::connect(videoSelecter, &QPushButton::clicked, this, &ImageSelecterGUI::selectVideo);

  startVideoButton = new QPushButton("start video", this);
  QObject::connect(startVideoButton, &QPushButton::clicked, this, &ImageSelecterGUI::startVideo);
  startVideoButton->setEnabled(false);

//  videoDelayLabel = new QLabel(QString(), this);
  videoDelayLabel = new QLabel(QString("video delay:"), this);

  videoDelayLineEdit = new QLineEdit(this);
  videoDelayLineEdit->setValidator(new QIntValidator(1, 100, this));
  QSizePolicy sizePolicyLineEdit(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  videoDelayLineEdit->setSizePolicy(sizePolicyLineEdit);
  videoDelayLineEdit->setText(QString(videoDelay));

  // Video QhBox
  qhBoxVideo = new QHBoxLayout;
  qhBoxVideo->addWidget(checkVideoMode);
  qhBoxVideo->addWidget(videoSelecter);
  qhBoxVideo->addWidget(startVideoButton);
  qhBoxVideo->addWidget(videoDelayLabel);
  qhBoxVideo->addWidget(videoDelayLineEdit);
  groupBoxVideo = new QGroupBox(this);
  groupBoxVideo->setTitle("Video Parameter");
  groupBoxVideo->setLayout(qhBoxVideo);
  QSizePolicy sizePolicyVideo(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  groupBoxVideo->setSizePolicy(sizePolicyVideo);

  // Total Gui
  this->setWindowTitle(QString::fromStdString(ros::this_node::getName()));
  gridLayout1 = new QGridLayout();
  gridLayout1->addWidget(groupBoxImage, 0, 0);
  gridLayout1->addWidget(groupBoxVideo, 1, 0);
  gridLayout1->addWidget(image_label, 2, 0);
  this->setLayout(gridLayout1);

}

ImageSelecterGUI::~ImageSelecterGUI() {
//  delete label;
  delete checkboxSendAsCurrent;
  delete checkboxLoadAsGray;
  delete radioButtonBlue;
  delete radioButtonRed;
  delete publish_image;
  delete image_selecter;
  delete image_label;
  delete qhBoxImage;
  delete groupBoxVideo;
  delete groupBoxImage;
  delete checkImageMode;
  delete checkVideoMode;
  delete groupBoxVideo;
  delete qhBoxVideo;
  delete videoSelecter;
  delete startVideoButton;
  delete videoDelayLabel;
  delete videoDelayLineEdit;

  shutdown_required = true;
  thread.join();
}

void ImageSelecterGUI::spin() {
  ros::Rate loop(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }
  ROS_INFO("[%s] Shutdown this node.", ros::this_node::getName().c_str());
  ros::shutdown();
  QApplication::quit();
}

void ImageSelecterGUI::toggleModeImage() {
  imageMode = true;
  toggleMode();
}

void ImageSelecterGUI::toggleModeVideo() {
  imageMode = false;
  toggleMode();
}

void ImageSelecterGUI::toggleMode() {
  checkImageMode->setChecked(imageMode);
  checkVideoMode->setChecked(!imageMode);

  groupBoxImage->setEnabled(imageMode);
  groupBoxVideo->setEnabled(!imageMode);
  ROS_INFO("image mode %d", imageMode);
  cout << "checkImageMode->isChecked():" << checkImageMode->isChecked() << endl;
  cout << "checkVideoMode->isChecked():" << checkVideoMode->isChecked() << endl;
}

void ImageSelecterGUI::selectVideo() {
  QString fileName = QFileDialog::getOpenFileName(this, QString::fromStdString("Open video"), QString::fromStdString("../patter/"), QString::fromStdString("Video Files (*.avi *.mp4)"));
  cap.open(fileName.toStdString());
  cap.set(CV_CAP_PROP_XI_FRAMERATE, 1);
  startVideoButton->setEnabled(true);
}

void ImageSelecterGUI::startVideo() {
  while(cap.read(cv_image)) {
    qt_image = mat2QImage(cv_image);
    if(qt_image.size().width() > 1200) {
      image_label->setPixmap(QPixmap::fromImage(qt_image.scaledToWidth(1200)));
    } else if(qt_image.size().height() > 1000) {
      image_label->setPixmap(QPixmap::fromImage(qt_image.scaledToHeight(1000)));
    } else {
      image_label->setPixmap(QPixmap::fromImage(qt_image));
    }
    image_label->show();
    cout << "image_label->show()" << endl;

    publishImage();
    sleep(videoDelay);
  }
  startVideoButton->setEnabled(false);

}

void ImageSelecterGUI::selectImage() {
  QString fileName = QFileDialog::getOpenFileName(this, QString::fromStdString("Open image"), QString::fromStdString("../patter/"), QString::fromStdString("Image Files (*.png *.jpg *.jpeg *.bmp)"));

  // Check if loading as grayscale
  if (checkboxLoadAsGray->checkState() == Qt::CheckState::Checked) {
    cv_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
  } else {
    cv_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_COLOR);
  }

  // Invert the color channel if necessary
  if (cv_image.channels() == 1) {
    ROS_INFO("[%s] loaded gray scale image", ros::this_node::getName().c_str());
    std::vector<cv::Mat> array_to_merge(3);
    cv::Mat dummy(cv_image.size(), cv_image.type(), cv::Scalar(uchar(0)));
    if (radioButtonRed->isChecked()) {
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
    ROS_INFO("[%s] loaded BGR image", ros::this_node::getName().c_str());
  } else {
    ROS_ERROR("[%s] Unsupported number of channels", ros::this_node::getName().c_str());
    return;
  }

  // Display the image
  cv::Mat rgb;
  cv::cvtColor(cv_image, rgb, CV_BGR2RGB);
  qt_image = mat2QImage(rgb);


  if(qt_image.size().width() > 1200) {
    image_label->setPixmap(QPixmap::fromImage(qt_image.scaledToWidth(1200)));
  } else if(qt_image.size().height() > 1000) {
    image_label->setPixmap(QPixmap::fromImage(qt_image.scaledToHeight(1000)));
  } else {
    image_label->setPixmap(QPixmap::fromImage(qt_image));
  }
  image_label->show();
  publish_image->setEnabled(true);
}

void ImageSelecterGUI::publishImage() {
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = cv_image;
  if (checkboxSendAsCurrent->checkState() == Qt::CheckState::Checked) {
    ROS_INFO("[%s] Send as current", ros::this_node::getName().c_str());
    cvImage.header.frame_id = "current";
  } else {
    cvImage.header.frame_id = "charge";
    ROS_INFO("[%s] Send as charge", ros::this_node::getName().c_str());
  }
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());

// Ros Topics
  string rosPublisherTopic;

  node.param<string>("image_publisher_topic", rosPublisherTopic, "/image");
  node.param<int>("load_as_mono", loadAsMono, 0);
  node.param<int>("mono_as_red", monoAsRed, 0);
  node.param<int>("charge_as_current", chargeAsCurrent, 0);
  node.param<bool>("image_mode", imageMode, true);
  node.param<int>("video_delay", videoDelay, 7);
  ROS_INFO("[%s] image_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopic.c_str());

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(rosPublisherTopic, 1, true);

  QApplication app(argc, argv);
  ImageSelecterGUI gui;
  gui.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  return app.exec();
}
