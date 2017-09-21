// Qt5 Widgets
#include <QWidget>
#include <QMainWindow>
#include <QFileDialog>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>

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

  void selectImage();

  void publishImage();

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

  void spin();

private:

  bool shutdown_required;
  boost::thread thread;

//  QPushButton *image_select;
  QPushButton *publish_image;
  QLabel *image_label;
  QPushButton *image_selecter;
  cv::Mat cv_image;
  QImage qt_image;
  QLabel *label;
  QCheckBox *checkboxLoadAsGray;
  QCheckBox *checkboxInvGray;
  QCheckBox *checkboxSendAsCurrent;
  bool imageSelected = false;

};

#endif //_IMAGE_SELECTER_GUI_HPP_
