// ============================================================================
// Name        : potential_field_utils.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Helper functions
// ============================================================================

#ifndef POTENTIAL_FIELD_UTILS_HPP_
#define POTENTIAL_FIELD_UTILS_HPP_

#include <omp.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

/**
 * Returns the minimum angle difference between the origin and target
 * @param origin Angle taken as reference
 * @param target Angle taken as target
 * @return Minimum difference
 */
double
getAngleDiff(double origin, double target) {

  // Get the angle between 0 .. 2 PI
  origin = fmod(origin + 2 * M_PI, 2 * M_PI);
  target = fmod(target + 2 * M_PI, 2 * M_PI);

  // Calculate the steering vector
  double angleDiff = target - origin;
  if (angleDiff > M_PI) {
    angleDiff = angleDiff - 2 * M_PI;
  } else if (angleDiff < -M_PI) {
    angleDiff = 2 * M_PI + angleDiff;
  }
  return angleDiff;
}

/**
 * Convert bgr to gray image without scaling
 * @param bgr Input image
 * @return Output gray image
 */
cv::Mat
bgr_to_gray(const cv::Mat &image) {
  cv::Mat gray(image.size(), CV_8UC1);
  for (int idx = 0; idx < image.cols * image.rows; ++idx) {
    gray.at<uchar>(idx) =
        (ushort(image.at<cv::Vec3b>(idx)[0]) +
         ushort(image.at<cv::Vec3b>(idx)[1]) +
         ushort(image.at<cv::Vec3b>(idx)[2])) / 3;

  }
  return gray;
}

/**
 * Convert rgb to gray image without scaling
 * @param rgb Input image
 * @return Output gray image
 */
cv::Mat
rgb_to_gray(const cv::Mat &image) {
  return bgr_to_gray(image);
}

/**
 * Takes a 2D CV_32FC2 Vectorfield and convertes with the hsv image encoding to a bgr mat.
 * @param vectorField Input vectorfield
 * @return Output hsv image
 */
cv::Mat
vectorfield_to_hsv(const cv::Mat &vectorField, std::string id = "") {
  cv::Mat hsv(vectorField.size(), CV_8UC3);

  float l_max_square = 0.0;
  int xMax = 0, yMax = 0;
  for (int y = 0; y < vectorField.rows; y++) {
    for (int x = 0; x < vectorField.cols; x++) {
      cv::Point2f point(vectorField.at<cv::Vec2f>(y, x)[0], vectorField.at<cv::Vec2f>(y, x)[1]);
      const double l_square = point.x * point.x + point.y * point.y;
      if (l_square > l_max_square) {
        xMax = x;
        yMax = y;
        l_max_square = l_square;
      }
//      l_max_square = std::max(l_square, l_max_square);
    }
  }

  std::cerr << "-----" << id << "-------- MAX_VALUE(v,v_x,v_y): " << sqrt(l_max_square) << ", " << vectorField.at<cv::Vec2f>(yMax, xMax)[0] << ", " << vectorField.at<cv::Vec2f>(yMax, xMax)[1] << "\n";

#pragma omp parallel for
  for (int y = 0; y < vectorField.rows; y++) {
    for (int x = 0; x < vectorField.cols; x++) {
      const cv::Point2f point(vectorField.at<cv::Vec2f>(y, x)[0], vectorField.at<cv::Vec2f>(y, x)[1]);
      double angle = atan2(point.y, point.x) / M_PI * 180;
      angle = fmod(angle + 360.0, 360.0);
      const double l = sqrt((point.x * point.x + point.y * point.y) / l_max_square);

      hsv.at<cv::Vec3b>(y, x)[0] = angle / 2; // H
      hsv.at<cv::Vec3b>(y, x)[1] = 255;       // S
      hsv.at<cv::Vec3b>(y, x)[2] = l * 255;   // V
    }
  }
  return hsv;
}

/**
 * Convert hsv to gray image
 * @param hsv Input image
 * @return Output gray image
 */
inline cv::Mat
hsv_to_gray(const cv::Mat &hsv) {
  cv::Mat gray(hsv.size(), CV_8UC1);
  for (int idx = 0; idx < hsv.rows * hsv.cols; idx++) {
    gray.at<uchar>(idx) = hsv.at<cv::Vec3b>(idx)[2]; // Get the value V
  }
  return gray;
}

/**
 * Convert hsv to bgr image
 * @param hsv Input image
 * @return Output bgr image
 */
inline cv::Mat
hsv_to_bgr(const cv::Mat &hsv) {
  cv::Mat bgr(hsv.size(), CV_8UC3);
  cv::cvtColor(hsv, bgr, CV_HSV2BGR);
  return bgr;
}

/**
 * Convert hsv to rgb image
 * @param hsv Input image
 * @return Output rgb image
 */
inline cv::Mat
hsv_to_rgb(const cv::Mat &hsv) {
  cv::Mat rgb(hsv.size(), CV_8UC3);
  cv::cvtColor(hsv, rgb, CV_HSV2RGB);
  return rgb;
}


/**
 * Calculates the common derivatives as 2D CV_32FC2 vector field of a potential field
 * @param potentialField Input potential field
 * @param getPerpendicularVectorfield Flip the derivation
 * @param ddepth Derivative parameter
 * @param anchor Derivative parameter
 * @param delta Derivative parameter
 * @param borderType Derivative parameter
 * @return Output CV_32FC2 vector field
 */
cv::Mat
potentialfield_to_vectorfield(const cv::Mat &potentialField,
                              const bool getPerpendicularVectorfield = false,
                              const int ddepth = -1,
                              const cv::Point anchor = cv::Point( -1, -1 ),
                              const double delta = 0,
                              int borderType = cv::BORDER_REPLICATE) {

  cv::Mat vectorFieldX(potentialField.rows, potentialField.cols, CV_32FC1),
          vectorFieldY(potentialField.rows, potentialField.cols, CV_32FC1),
          vectorField(potentialField.rows, potentialField.cols, CV_32FC2);

  /// Update kernel size for a normalized box filter
  const int kernel_size = 3;
  const cv::Mat kernelY = (cv::Mat_<float>(kernel_size,kernel_size)
                                                << -1, -2, -1,
                                                   0,  0,  0,
                                                   1,  2, 1);
  const cv::Mat kernelX = (cv::Mat_<float>(kernel_size,kernel_size)
                                                << -1,  0,  1,
                                                   -2,  0,  2,
                                                   -1,  0,  1);

  /// Apply filter
  cv::filter2D(potentialField, vectorFieldX, ddepth, kernelX, anchor, delta, borderType);
  cv::filter2D(potentialField, vectorFieldY, ddepth, kernelY, anchor, delta, borderType);

  // merge
  if (getPerpendicularVectorfield) {
    cv::Mat vectorFieldChannels[2] = {vectorFieldY, -vectorFieldX};
    cv::merge(vectorFieldChannels,2,vectorField);
  } else {
    cv::Mat vectorFieldChannels[2] = {vectorFieldX, vectorFieldY};
    cv::merge(vectorFieldChannels,2,vectorField);
  }

  return vectorField;
}

enum RotateFlags {
    ROTATE_90_CLOCKWISE = 1, //Rotate 90 degrees clockwise
    ROTATE_90_COUNTERCLOCKWISE = 2, //Rotate 270 degrees clockwise
    ROTATE_180 = 3, //Rotate 180 degrees clockwise
};

void rot90(cv::Mat &matImage, int rotflag){
  //1=CW, 2=CCW, 3=180
  if (rotflag == 1){
    transpose(matImage, matImage);
    flip(matImage, matImage,1); //transpose+flip(1)=CW
  } else if (rotflag == 2) {
    transpose(matImage, matImage);
    flip(matImage, matImage,0); //transpose+flip(0)=CCW
  } else if (rotflag ==3){
    flip(matImage, matImage,-1);    //flip(-1)=180
  } else if (rotflag != 0){ //if not 0,1,2,3:
    std::cerr  << "Unknown rotation flag(" << rotflag << ")" << std::endl;
  }
}

cv::Point2i pose2pixel(const geometry_msgs::Pose &pose, const int imageWidth, const int imageHeight, const float meterPerPixel) {
  return cv::Point2i(
      (int) (imageWidth/2 - pose.position.y / meterPerPixel),
      (int) (imageHeight/2 - pose.position.x / meterPerPixel));
}


#endif /* POTENTIAL_FIELD_UTILS_HPP_ */
