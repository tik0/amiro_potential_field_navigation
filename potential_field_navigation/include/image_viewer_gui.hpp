// ============================================================================
// Name        : image_viewer_gui.hpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Creates a GUI to view two different images.
// ============================================================================

// Qt5 Widgets
#include <QWidget>
#include <QObject>
#include <QMainWindow>
#include <QFileDialog>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QSlider>

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

  QImage mat2QImage(const cv::Mat &mat) {
    return QImage((const unsigned char *) (mat.data), mat.cols, mat.rows, QImage::Format_RGB888);
  }

private:


  void spin();

  void transparencySliderValueChanged1(int value);

  void transparencySliderValueChanged2(int value);

  void clickedLoadImageButton1();

  void clickedLoadImageButton2();

private:

  bool shutdown_required;
  boost::thread thread;

  QLineEdit *textBox1;
  QLineEdit *textBox2;
  QPushButton *loadImageButton1;
  QPushButton *loadImageButton2;
  QSlider *transparencySlider1;
  QSlider *transparencySlider2;
  QLabel *transparencySliderLabel1;
  QLabel *transparencySliderLabel2;
  QLabel *transparencySliderLabelInfo1;
  QLabel *transparencySliderLabelInfo2;
  QLabel *imageLabel1;
  QLabel *imageLabel2;
};

#endif //_IMAGE_VIEWER_GUI_HPP_
