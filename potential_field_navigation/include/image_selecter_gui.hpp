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

/*
 * ROS QT Implementation after this tutorial:
 *
 */

#ifndef _IMAGE_SELECTER_GUI_HPP_
#define _IMAGE_SELECTER_GUI_HPP_

class ImageSelecterGUI : public QWidget {

//Q_OBJECT

public:
  explicit ImageSelecterGUI(QWidget *parent = 0);

  ~ImageSelecterGUI();

public:

  void toggleMode();
  void toggleModeImage();
  void toggleModeVideo();
  void selectImage();
  void selectVideo();
  void publishImage();
  void startVideo();
  void spin();

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

  QGroupBox *groupBoxImage;
  QHBoxLayout *qhBoxImage;
  QGridLayout *gridLayout1;
  QPushButton *publish_image;
  QPushButton *image_selecter;
  QLabel *image_label;
  QCheckBox *checkboxLoadAsGray;
  QCheckBox *checkboxSendAsCurrent;
  QRadioButton *radioButtonRed;
  QRadioButton *radioButtonBlue;

  QCheckBox *checkImageMode;
  QCheckBox *checkVideoMode;

  QGroupBox *groupBoxVideo;
  QHBoxLayout *qhBoxVideo;
  QPushButton *videoSelecter;
  QPushButton *startVideoButton;
  QLabel *videoDelayLabel;
  QLineEdit *videoDelayLineEdit;

};

#endif //_IMAGE_SELECTER_GUI_HPP_
