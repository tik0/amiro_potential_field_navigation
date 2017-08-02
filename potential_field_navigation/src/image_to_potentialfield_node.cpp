// ============================================================================
// Name        : image_to_potentialfield_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de
// Description : Recieve a image and creates a potentialfield.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <omp.h>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

// Ros Topics
string rosListenerTopic;
string rosPublisherTopic;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;


// program name
const string programName = "image_to_potentialfield_node";

cv::Mat toMat(cv::Mat potentialField) {
  // point to hsv
  cv::Mat out(potentialField.size(), CV_8UC3);
  for (int y = 0; y < potentialField.rows; y++) {
    for (int x = 0; x < potentialField.cols; x++) {
      cv::Vec3b vec;
      cv::Point point(potentialField.at<cv::Vec2f>(y, x)[0], potentialField.at<cv::Vec2f>(y, x)[1]);
      double angle = atan2(point.y,point.x) / M_PI * 180;
      angle = fmod(angle + 360.0, 360.0);
      int h = angle / 60;
//      int C = 255;
      double C = sqrt(point.x * point.x + point.y * point.y) * 0.31;
//      cout << C << endl;
      int X = C * (1 - abs(h % 2 - 1));
//      cout << h << "\t" << X << "\t" << angle << endl;
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
      out.at<cv::Vec3b>(y,x) = vec;
    }
  }
  return out;
}

void process() {
  cv::Mat image = cv::imread("patter/a.png", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat potentialField(image.size(), CV_32FC2);

  vector<cv::Point> blackPixel;
  vector<cv::Point> whitePixel;
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (image.at<uchar>(x, y) == 0)
        blackPixel.emplace_back(cv::Point(y, x));
      else
        whitePixel.emplace_back(cv::Point(y, x));
    }
  }
  cout << "blackPixel size:" << blackPixel.size() << " whitePixel size:" << whitePixel.size() << endl;


#pragma omp parallel for
  for (int i = 0; i < whitePixel.size(); i++) {
    cv::Point wPoint = whitePixel[i];
    cv::Point potential;
    for (cv::Point bPoint : blackPixel) {
      potential += bPoint - wPoint;
//      double angle = atan2(wPoint.y - bPoint.y, wPoint.x - bPoint.x);
    }
    potentialField.at<cv::Vec2f>(wPoint.y, wPoint.x)[0] = potential.x / (float) blackPixel.size();
    potentialField.at<cv::Vec2f>(wPoint.y, wPoint.x)[1] = potential.y / (float) blackPixel.size();
  }

//  cv::imshow("out", toMat(potentialField));
//  cv::waitKey(0);
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImage.image = potentialField;
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");
  node.param<string>("image_listener_topic", rosListenerTopic, "/image");
  ROS_INFO("image_listener_topic: %s", rosListenerTopic.c_str());
  node.param<string>("potentialfield_publisher_topic", rosPublisherTopic, "/potentialfield");
  ROS_INFO("potentialfield_publisher_topic: %s", rosPublisherTopic.c_str());

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(rosPublisherTopic, 1);


  boost::posix_time::ptime before = boost::posix_time::microsec_clock::local_time();
  process();
  boost::posix_time::ptime after = boost::posix_time::microsec_clock::local_time();
  cout << "time: " << (after - before).total_milliseconds() << "ms" << endl;

  ros::spin();
  return 0;

}
