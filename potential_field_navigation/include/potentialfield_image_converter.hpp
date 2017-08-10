// ============================================================================
// Name        : potentialfield_image_converter.hpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve a potentialfield and creates a hsv encoded image.
// ============================================================================
#ifndef _POTENTIALFIELD_IMAGE_CONVERTER_HPP_
#define _POTENTIALFIELD_IMAGE_CONVERTER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Takes a 2D CV_32FC2 Potentialfield and convertes with the hsv image encoding to a rgb mat.
 * @param potentialField Input potentialfield
 * @return Output rgb image
 */
cv::Mat potentialfield_to_rgb_cv_mat(cv::Mat potentialField) {
  // point to hsv
  cv::Mat out(potentialField.size(), CV_8UC3);
  for (int y = 0; y < potentialField.rows; y++) {
    for (int x = 0; x < potentialField.cols; x++) {
      cv::Vec3b vec;
      cv::Point point(potentialField.at<cv::Vec2f>(y, x)[0], potentialField.at<cv::Vec2f>(y, x)[1]);
      double angle = atan2(point.y, point.x) / M_PI * 180;
      angle = fmod(angle + 360.0, 360.0);
      int h = angle / 60;
      double C = sqrt(point.x * point.x + point.y * point.y) * 0.31;
      int X = C * (1 - abs(h % 2 - 1));
      switch (h) {
        case 0:
          vec[2] = C;
          vec[1] = X;
          vec[0] = 0;
          break;
        case 1:
          vec[2] = X;
          vec[1] = C;
          vec[0] = 0;
          break;
        case 2:
          vec[2] = 0;
          vec[1] = C;
          vec[0] = X;
          break;
        case 3:
          vec[2] = 0;
          vec[1] = X;
          vec[0] = C;
          break;
        case 4:
          vec[0] = X;
          vec[1] = 0;
          vec[2] = C;
          break;
        case 5:
          vec[0] = C;
          vec[1] = 0;
          vec[2] = X;
          break;
      }
//      out.at<cv::Vec3b>(y,x)[0] = angle * 0.708;
      out.at<cv::Vec3b>(y, x) = vec;
    }
  }
  return out;
}

#endif //_POTENTIALFIELD_IMAGE_CONVERTER_HPP_
