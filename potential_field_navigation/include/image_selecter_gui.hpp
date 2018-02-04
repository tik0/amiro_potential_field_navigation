#ifndef _IMAGE_SELECTER_GUI_HPP_
#define _IMAGE_SELECTER_GUI_HPP_


// Qt5 Widgets
#include <QWidget>
#include <QMainWindow>
#include <QFileDialog>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QRadioButton>
#include <QLineEdit>
#include <QIntValidator>

#include <boost/thread.hpp>
#include <atomic>

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

class ImageSelecterGUI : public QWidget {

//Q_OBJECT

public:
  explicit ImageSelecterGUI(QWidget *parent = 0);

  ~ImageSelecterGUI();

public:
  void selectImage();
  void showImage();
  void selectVideo();
  void publishImage();
  void publishVideo();
  void spin();
  void toggleMode();

  QImage mat2QImage(const cv::Mat &mat) {
    QImage::Format format;
    if (cv_image.channels() == 1) {
      std::cout << "mat2QImage: loaded gray scale image\n";
      format = QImage::Format_Grayscale8;
    } else if (cv_image.channels() == 3) {
      std::cout << "mat2QImage: loaded BGR image\n";
      format = QImage::Format_RGB888;
    } else {
      std::cerr << "mat2QImage: Unsupported number of channels\n";
      return QImage();
    }
    return QImage((const unsigned char *) (mat.data), mat.cols, mat.rows, format);
  }

private:

  bool shutdown_required;
  boost::thread thread;

  cv::Mat cv_image;
  QImage qt_image;

  QGroupBox *groupBoxCommon;
  QHBoxLayout *qhBoxCommon;
  QCheckBox *checkboxSendAsCurrent;
  QRadioButton *radioButtonImage;
  QRadioButton *radioButtonVideo;

  QGroupBox *groupBoxImage;
  QHBoxLayout *qhBoxImage;
  QGridLayout *gridLayout1;
  QPushButton *publishImageButton;
  QPushButton *imageSelectButton;
  QLabel *imageLabel;
  QCheckBox *checkboxLoadAsGray;
  QRadioButton *radioButtonRed;
  QRadioButton *radioButtonBlue;
  
  QGroupBox *groupBoxVideo;
  QHBoxLayout *qhBoxVideo;
  QPushButton *videoSelectButton;
  QPushButton *publishVideoButton;
  QLabel *videoDelayLabel;
  QLineEdit *videoDelayLineEdit;
  
  std::atomic<bool> doPublishVideo;

};

#endif //_IMAGE_SELECTER_GUI_HPP_
