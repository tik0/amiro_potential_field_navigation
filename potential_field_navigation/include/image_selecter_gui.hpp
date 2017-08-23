// Qt5 Widgets
#include <QWidget>

#ifndef _IMAGE_SELECTER_GUI_HPP_
#define _IMAGE_SELECTER_GUI_HPP_

class GUI : public QWidget {

//Q_OBJECT

public:
  explicit GUI(QWidget *parent = 0);

public:

  void selectImage();

  void publishImage();

private:

  QLabel *image_label;
  QPushButton *image_select;
  QPushButton *publish_image;
};

#endif //_IMAGE_SELECTER_GUI_HPP_
