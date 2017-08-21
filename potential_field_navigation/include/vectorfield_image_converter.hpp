// ============================================================================
// Name        : vectorfield_image_converter.hpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve a potentialfield and creates a hsv encoded image.
// ============================================================================
#ifndef _VECTORFIELD_IMAGE_CONVERTER_HPP_
#define _VECTORFIELD_IMAGE_CONVERTER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Takes a 2D CV_32FC2 Vectorfield and convertes with the hsv image encoding to a bgr mat.
 * @param vectorField Input vectorfield
 * @return Output bgr image
 */
cv::Mat vectorfield_to_bgr_cv_mat(cv::Mat vectorField) {
  // point to hsv
  cv::Mat bgr(vectorField.size(), CV_8UC3);
  cv::Mat hsv(vectorField.size(), CV_8UC3);

  double l_max = 0.0;
  for (int y = 0; y < vectorField.rows; y++) {
    for (int x = 0; x < vectorField.cols; x++) {
      cv::Point2f point(vectorField.at<cv::Vec2f>(y, x)[0], vectorField.at<cv::Vec2f>(y, x)[1]);
      double l = sqrt(point.x * point.x + point.y * point.y);
      l_max = std::max(l, l_max);
    }
  }

  for (int y = 0; y < vectorField.rows; y++) {
    for (int x = 0; x < vectorField.cols; x++) {
      cv::Point2f point(vectorField.at<cv::Vec2f>(y, x)[0], vectorField.at<cv::Vec2f>(y, x)[1]);
      double angle = -atan2(point.y, -point.x) / M_PI * 180 - 90;
      angle = fmod(angle + 360.0, 360.0);
      double l = sqrt(point.x * point.x + point.y * point.y) / l_max;

      hsv.at<cv::Vec3b>(y, x)[0] = angle / 2;
      hsv.at<cv::Vec3b>(y, x)[1] = 255;
      hsv.at<cv::Vec3b>(y, x)[2] = l * 255;
//      hsv.at<cv::Vec3b>(y, x)[2] = 255;

    }
  }
  cv::cvtColor(hsv, bgr, CV_HSV2BGR);
  return bgr;
}

#endif //_VECTORFIELD_IMAGE_CONVERTER_HPP_
