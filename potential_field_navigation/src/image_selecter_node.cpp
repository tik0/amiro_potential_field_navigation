// ============================================================================
// Name        : image_selecter_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Creates a GUI to select an image or video for publishing
// ============================================================================

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

  doPublishVideo = false;
  // Image Gui Stuff
  imageLabel = new QLabel(this);

  imageSelectButton = new QPushButton("select image", this);
  QObject::connect(imageSelectButton, &QPushButton::clicked, this, &ImageSelecterGUI::selectImage);

  publishImageButton = new QPushButton("publish image", this);
  QObject::connect(publishImageButton, &QPushButton::clicked, this, &ImageSelecterGUI::publishImage);
  publishImageButton->setEnabled(false);

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
  
  radioButtonImage = new QRadioButton("Image");
  radioButtonVideo = new QRadioButton("Video");
  radioButtonImage->setChecked(true);
  radioButtonVideo->setChecked(false);

  checkboxSendAsCurrent = new QCheckBox("as current", this);
  if (chargeAsCurrent) {
    checkboxSendAsCurrent->setChecked(true);
  } else {
    checkboxSendAsCurrent->setChecked(false);
  }

  QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);

  // Common QhBox
  qhBoxCommon = new QHBoxLayout;
  qhBoxCommon->addWidget(checkboxSendAsCurrent);
  qhBoxCommon->addWidget(radioButtonImage);
  qhBoxCommon->addWidget(radioButtonVideo);
  groupBoxCommon = new QGroupBox(this);
  groupBoxCommon->setTitle("Common Parameters");
  groupBoxCommon->setLayout(qhBoxCommon);
  groupBoxCommon->setSizePolicy(sizePolicy);
  QObject::connect(radioButtonImage, &QRadioButton::clicked, this, &ImageSelecterGUI::toggleMode);
  QObject::connect(radioButtonVideo, &QRadioButton::clicked, this, &ImageSelecterGUI::toggleMode);
  
  // Image QhBox
  qhBoxImage = new QHBoxLayout;
  qhBoxImage->addWidget(imageSelectButton);
  qhBoxImage->addWidget(publishImageButton);
  qhBoxImage->addWidget(checkboxLoadAsGray);
  qhBoxImage->addWidget(radioButtonRed);
  qhBoxImage->addWidget(radioButtonBlue);
  groupBoxImage = new QGroupBox(this);
  groupBoxImage->setTitle("Image Parameters");
  groupBoxImage->setLayout(qhBoxImage);
  groupBoxImage->setSizePolicy(sizePolicy);

  // Video Gui Stuff
  videoSelectButton = new QPushButton("select video", this);
  QObject::connect(videoSelectButton, &QPushButton::clicked, this, &ImageSelecterGUI::selectVideo);

  publishVideoButton = new QPushButton("start video", this);
  QObject::connect(publishVideoButton, &QPushButton::clicked, this, &ImageSelecterGUI::publishVideo);
  publishVideoButton->setEnabled(false);

  videoDelayLabel = new QLabel(QString("Seconds per Frame:"), this);

  videoDelayLineEdit = new QLineEdit(this);
  videoDelayLineEdit->setValidator(new QIntValidator(1, 100, this));
  QSizePolicy sizePolicyLineEdit(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  videoDelayLineEdit->setSizePolicy(sizePolicy);
  videoDelayLineEdit->setText(QString::number(videoDelay));

  // Video QhBox
  qhBoxVideo = new QHBoxLayout;
  qhBoxVideo->addWidget(videoSelectButton);
  qhBoxVideo->addWidget(publishVideoButton);
  qhBoxVideo->addWidget(videoDelayLabel);
  qhBoxVideo->addWidget(videoDelayLineEdit);
  groupBoxVideo = new QGroupBox(this);
  groupBoxVideo->setTitle("Video Parameters");
  groupBoxVideo->setLayout(qhBoxVideo);
  groupBoxVideo->setSizePolicy(sizePolicy);

  // Total Gui
  this->setWindowTitle(QString::fromStdString(ros::this_node::getName()));
  gridLayout1 = new QGridLayout();
  gridLayout1->addWidget(groupBoxCommon, 0, 0);
  gridLayout1->addWidget(groupBoxImage, 1, 0);
  gridLayout1->addWidget(groupBoxVideo, 2, 0);
  gridLayout1->addWidget(imageLabel, 3, 0);
  this->setLayout(gridLayout1);
  groupBoxImage->setEnabled(true);
  groupBoxVideo->setEnabled(false);

}

ImageSelecterGUI::~ImageSelecterGUI() {
  delete checkboxSendAsCurrent;
  delete checkboxLoadAsGray;
  delete radioButtonBlue;
  delete radioButtonRed;
  delete publishImageButton;
  delete imageSelectButton;
  delete imageLabel;
  delete qhBoxImage;
  delete groupBoxVideo;
  delete groupBoxImage;
  delete groupBoxVideo;
  delete qhBoxVideo;
  delete videoSelectButton;
  delete publishVideoButton;
  delete videoDelayLabel;
  delete videoDelayLineEdit;
  delete radioButtonImage;
  delete radioButtonVideo;
  delete qhBoxCommon;
  delete groupBoxCommon;

  shutdown_required = true;
  thread.join();
}

void ImageSelecterGUI::spin() {
  sleep(3);
  ros::Rate loop(10);
  while (ros::ok()) {
    if (doPublishVideo) {
        if (cap.read(cv_image)) {
            showImage();
            publishImage();
        } else {
            doPublishVideo = false;
        }
        loop = ros::Rate(1/float(videoDelayLineEdit->text().toInt()));
    } else {
        if (videoSelectButton) {
            videoSelectButton->setEnabled(true);
        }
        ros::Rate loop(10); // Reset the rate
    }
    ros::spinOnce();
    loop.sleep();
  }
  ROS_INFO("[%s] Shutdown this node.", ros::this_node::getName().c_str());
  ros::shutdown();
  QApplication::quit();
}

void ImageSelecterGUI::toggleMode() {
    if (groupBoxImage != NULL && groupBoxVideo != NULL) {
        if (radioButtonImage->isChecked()) {
            groupBoxImage->setEnabled(true);
            groupBoxVideo->setEnabled(false);
        } else {
            groupBoxImage->setEnabled(false);
            groupBoxVideo->setEnabled(true);
        }
    }
}

void ImageSelecterGUI::selectVideo() {
  QString fileName = QFileDialog::getOpenFileName(this, QString::fromStdString("Open video"), QString::fromStdString("../patter/"), QString::fromStdString("Video Files (*.avi *.mp4)"));
  cap.open(fileName.toStdString());
  cap.set(CV_CAP_PROP_XI_FRAMERATE, 1);
  publishVideoButton->setEnabled(true);
}

void ImageSelecterGUI::publishVideo() {
  doPublishVideo = true;
  videoSelectButton->setEnabled(false);
  publishVideoButton->setEnabled(false);
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

  showImage();
  publishImageButton->setEnabled(true);
}

void ImageSelecterGUI::showImage() {
  // Display the image
  cv::Mat rgb;
  cv::cvtColor(cv_image, rgb, CV_BGR2RGB);
  qt_image = mat2QImage(rgb);
  if(qt_image.size().width() > 1200) {
    imageLabel->setPixmap(QPixmap::fromImage(qt_image.scaledToWidth(1200)));
  } else if(qt_image.size().height() > 1000) {
    imageLabel->setPixmap(QPixmap::fromImage(qt_image.scaledToHeight(1000)));
  } else {
    imageLabel->setPixmap(QPixmap::fromImage(qt_image));
  }
  imageLabel->show();    
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
