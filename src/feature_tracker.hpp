#ifndef __FEATURE_TRACKER_H
#define __FEATURE_TRACKER_H

#include <ros/ros.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class FeatureTracker {

  ros::NodeHandle _nh; 
  image_transport::ImageTransport _it; // Needed for iamge transport.
  image_transport::Subscriber _image_sub; // Subscription handle.  
  image_transport::Publisher _image_pub;  // Publication handle. 
  cv::Mat _prevImg; 
  cv::Mat _curImg; 
  std::vector<cv::Point2f> _prevPoints; 
  std::vector<cv::Point2f> _curPoints; 
  std::vector<cv::Mat> _prevPyr;
  std::vector<cv::Mat> _curPyr;
  bool _initialized; 

  public: 
    FeatureTracker(ros::NodeHandle* nh); 
    void imageCallback(const sensor_msgs::ImageConstPtr& depth_image); 
}; 


#endif
