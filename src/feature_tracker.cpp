#include "feature_tracker.hpp"

using namespace std;
using namespace ros; 
using namespace cv; 

FeatureTracker::FeatureTracker(ros::NodeHandle* nh):
_nh(*nh), 
_it(*nh),
_initialized(false)
{
     std::string input_topic; 
     std::string output_topic; 
     if (!_nh.getParam("in", input_topic)){
	input_topic = "/camera/ir/image_raw";
     }
     if (!_nh.getParam("out", output_topic)) {
	output_topic = "/feature_tracker/out"; 
     } 
     ROS_INFO("Subscribing to %s", input_topic.c_str());
     ROS_INFO("Publishing to %s", output_topic.c_str());
     _image_sub = _it.subscribe(input_topic, 10, &FeatureTracker::imageCallback, this);
     _image_pub = _it.advertise(output_topic, 10); 
}

void FeatureTracker::imageCallback(const sensor_msgs::ImageConstPtr& image) {

    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    cv_bridge::CvImagePtr image_cv_ptr; 
    const int maxLevel = 3; 
    const int minNumFeatures = 5; 
    bool isColor = false; 
    // Convert to cv::Mat. 
    try{
        image_cv_ptr = cv_bridge::toCvCopy(image);  
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception %s at %s, line %d", e.what(), __FILE__, __LINE__); 
        return; 
    }

    // Move previous images. 

    std::swap(_curPoints, _prevPoints);
    cv::swap(_curImg, _prevImg);

    // Convert to gray. 
    if (image_cv_ptr->image.channels() > 1) {
	isColor = true; 
	cvtColor(image_cv_ptr->image, _curImg, COLOR_BGR2GRAY); 
    }
    // There's no previous frame to look at. 
    if ( (!_initialized) || (_prevImg.empty() ) || (_prevPoints.empty()) || (_prevPyr.empty()) ) {
        _prevImg = _curImg;
        goodFeaturesToTrack(_prevImg, _prevPoints, 100, 0.01, 5.0, Mat(), 10, false, 0.04);
	buildOpticalFlowPyramid(_curImg, _prevPyr, winSize, maxLevel, true);  
        _initialized = true; 
	return; 
    }

//    if (_prevPoints.size() < minNumFeatures) {
//	goodFeaturesToTrack(_prevImg, _prevPoints, 100, 0.01, 10.0, Mat(), 3, false, 0.04);  
//    }

    // _curImg is MONO8. Use image_cv_ptr for display. 
    Mat outImg = image_cv_ptr->image.clone(); 

    vector<uchar> status; 
    vector<float> err; 

    buildOpticalFlowPyramid(_curImg, _curPyr, winSize, maxLevel, true);
    calcOpticalFlowPyrLK(_prevPyr, _curPyr, _prevPoints, _curPoints, status, err, winSize, maxLevel, termcrit, 0, 0.004); 

    size_t i = 0, k = 0; 
    
    for( i = k = 0; i < _curPoints.size(); i++ )
    {
        if( !status[i] )
            continue;

        _curPoints[k++] = _curPoints[i]; // First k elements will contain good ones only. 
        circle( outImg, _curPoints[i], 3, Scalar(0,255,0), -1, 8);
    }
    _curPoints.resize(k); // Resize to k. 
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), isColor ? "rgb8" : "mono8", outImg).toImageMsg(); 
	_image_pub.publish(msg);
	// cv::imshow("debug", _curImg);  
}

int main(int argc, char** argv){
    ros::init(argc, argv, "feature_tracker");  
    ros::NodeHandle nh;
    std::string input_topic;  
    nh.getParam("in", input_topic);
    ROS_INFO("Outside it's %s", input_topic.c_str());
    FeatureTracker ft(&nh);
    ros::spin(); 
    return 0; 
}
