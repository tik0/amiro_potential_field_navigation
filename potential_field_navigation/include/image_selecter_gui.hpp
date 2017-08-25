// Qt5 Widgets
#include <QWidget>

#ifndef _IMAGE_SELECTER_GUI_HPP_
#define _IMAGE_SELECTER_GUI_HPP_

class GUI : public QWidget {

//Q_OBJECT

public:
  explicit GUI(QWidget *parent = 0);
  ~GUI();

public:

  void selectImage();

  void publishImage();

  QImage mat2QImage(const cv::Mat &mat) {
    return QImage((const unsigned char *) (mat.data), mat.cols, mat.rows, QImage::Format_Grayscale8);
  }

private:

//  QPushButton *image_select;
  QPushButton *publish_image;
  QLabel *image_label;
  QPushButton *image_selecter;
  cv::Mat cv_image;
  QImage qt_image;
  QLabel *label;
  bool imageSelected = false;

};

#endif //_IMAGE_SELECTER_GUI_HPP_
