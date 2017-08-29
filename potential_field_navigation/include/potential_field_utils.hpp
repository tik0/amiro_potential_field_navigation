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
#include <fstream>


/**
 * Stores the image as binary blob
 * @param img Image to store
 * @param name Path
 * @param depth size of variable (char=1, float=4, ...)
 */
void write_image_binary(cv::Mat &img, const std::string &name,int depth = 1 /*4 for float/int, 8 for double, ...*/) {
  std::ofstream file(name, std::ios::out | std::ios::binary);
  file.write((char*)img.data, img.rows * img.cols * img.channels() * depth);
  file.close();
}

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
 * Normalizes the lenght of all vectors to the largest present vector
 * @param vectorfield CV_32FC2 field
 * @return CV_32FC1 field with corresponding absolute values
 */
cv::Mat get_abs_field(cv::Mat &vectorfield) {
  cv::Mat absField(vectorfield.size(), CV_32FC1);
  for (int idx = 0; idx < vectorfield.rows * vectorfield.cols; ++idx) {
    const float x = vectorfield.at<cv::Vec2f>(idx)[0];
    const float y = vectorfield.at<cv::Vec2f>(idx)[1];
    absField.at<float>(idx) = sqrt(x*x + y*y);
  }
  return absField;
}

/**
 * Normalizes the lenght of all vectors to the largest present vector
 * @param vectorfield CV_32FC2 field to normalize
 */
void calc_normalized_field(cv::Mat &vectorfield) {
  float maxAbsValue = 0.0f;
  for (auto it = vectorfield.begin<cv::Vec2f>(); it != vectorfield.end<cv::Vec2f>(); ++it) {
    maxAbsValue = std::max((*it)[0] * (*it)[0] + (*it)[1] * (*it)[1], maxAbsValue);
  }
  maxAbsValue = sqrt(maxAbsValue);
  for (auto it = vectorfield.begin<cv::Vec2f>(); it != vectorfield.end<cv::Vec2f>(); ++it) {
    (*it)[0] /= maxAbsValue;
    (*it)[1] /= maxAbsValue;
  }
}

/**
 * Calculates a gray channel out of a vector field
 * @param vectorfield CV_32FC2 field
 * @return The gray channel
 */
cv::Mat vectorfield_to_gray(const cv::Mat &vectorfield) {
  cv::Mat gray(vectorfield.size(), CV_8UC1);
  cv::Mat vectorfieldCopy;
  vectorfield.copyTo(vectorfieldCopy);
  calc_normalized_field(vectorfieldCopy);
  cv::Mat vectorfieldAbs = get_abs_field(vectorfieldCopy);
  for (int idx = 0; idx < vectorfield.rows * vectorfield.cols; ++idx) {
    gray.at<uchar>(idx) = uchar(vectorfieldAbs.at<float>(idx) * 255.0f);
  }
  return gray;
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
  for (int y = 0; y < vectorField.rows; y++) {
    for (int x = 0; x < vectorField.cols; x++) {
      const cv::Point2f point(vectorField.at<cv::Vec2f>(y, x)[0], vectorField.at<cv::Vec2f>(y, x)[1]);
      const float l_square = point.x * point.x + point.y * point.y;
      l_max_square = std::max(l_square, l_max_square);
    }
  }

#pragma omp parallel for
  for (int y = 0; y < vectorField.rows; y++) {
    for (int x = 0; x < vectorField.cols; x++) {
      const cv::Point2f point(vectorField.at<cv::Vec2f>(y, x)[0], vectorField.at<cv::Vec2f>(y, x)[1]);
      double angle = atan2(point.y, point.x) / M_PI * 180;
      angle = fmod(angle + 360.0, 360.0);
      const float l = sqrt((point.x * point.x + point.y * point.y) / l_max_square);

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
  cv::Mat gray(hsv.rows, hsv.cols, CV_8UC1);
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
                              const double delta = 0,
                              int borderType = cv::BORDER_REPLICATE) {

  cv::Mat vectorFieldX(potentialField.rows, potentialField.cols, CV_32FC1, cv::Scalar(0.0f)),
          vectorFieldY(potentialField.rows, potentialField.cols, CV_32FC1, cv::Scalar(0.0f)),
          vectorField(potentialField.rows, potentialField.cols, CV_32FC2, cv::Scalar(0.0f));

  if (getPerpendicularVectorfield) {
    cv::Scharr(potentialField, vectorFieldY, ddepth, 1, 0, 1.0, delta, borderType);
    cv::Scharr(potentialField, vectorFieldX, ddepth, 0, 1, -1.0, delta, borderType);
  } else {
    cv::Scharr(potentialField, vectorFieldX, ddepth, 1, 0, 1.0, delta, borderType);
    cv::Scharr(potentialField, vectorFieldY, ddepth, 0, 1, 1.0, delta, borderType);
  }
  cv::Mat vectorFieldChannels[2] = {vectorFieldX, vectorFieldY};
  cv::merge(vectorFieldChannels,2,vectorField);

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
