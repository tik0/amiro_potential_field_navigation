// Qt5 Widgets
#include <QWidget>
#include <QMainWindow>
#include <QFileDialog>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>

#include <boost/thread.hpp>

/*
 * ROS QT Implementation after this tutorial:
 *
 */

#ifndef _IMAGE_VIEWER_GUI_HPP_
#define _IMAGE_VIEWER_GUI_HPP_

class ImageViewGUI : public QWidget {

//Q_OBJECT

public:
  explicit ImageViewGUI(QWidget *parent = 0);

  ~ImageViewGUI();

public:

  QImage mat2QImage(const cv::Mat &mat) {
    return QImage((const unsigned char *) (mat.data), mat.cols, mat.rows, QImage::Format_RGB888);
  }

  void spin();

private:

  bool shutdown_required;
  boost::thread thread;

  QLineEdit *text1;
  QLineEdit *text2;
  QLabel *image_label;

};

#endif //_IMAGE_VIEWER_GUI_HPP_
