#include "feature_tracker.hpp"

using namespace std;
using namespace ros; 
using namespace cv; 

FeatureTracker::FeatureTracker(String input_topic, String output_topic): 
_it(_nh),
_inititialized(false)
{
    _image_sub = _it.subscribe(input_topic, 10, &FeatureTracker::imageCallback, this);
    _image_pub = _it.advertize(output_topic, 10); 
}

FeatureTracker::imageCallback(const sensor_msgs::ImageConstPtr& image) {

    cv_bridge::CvImagePtr image_cv_ptr; 

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

    _curImg = cvtColor(image_cv_ptr->image, COLOR_BGR2GRAY); 

    // There's no previous frame to look at. 
    if ( (!_inititialized) || (_prevImg.empty() ) || (_prevPoints.empty()) ) {
        _prevImg = curImg;
        goodFeaturesToTrack(curImg, _prevPoints, 100, 0.01, 10, Mat(), 3, 3, 0, 0.04);
        return; 
    }

    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    Mat outImg = curImg.clone(); 

    vector<uchar> status; 
    vector<float> err; 

    calcOpticalFlowPyrLK(_prevImg, curImg, _prevPoints, _curPoints, status, err, winSize, 3, termcrit, 0, 0.001);
    
    for( i = k = 0; i < points[1].size(); i++ )
    {
        if( !status[i] )
            continue;

        _curPoints[k++] = _curPoints[i]; // First k elements will contain good ones only. 
        circle( outImg, _curPoints[i], 3, Scalar(0,255,0), -1, 8);
    }
    _curPoints.resize(k); // Resize to k. 

    _image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImg).toImageMsg();); 
}


int main(char argc, char** argv){
    ros::init(argc, argv, 'feature_tracker'); 
    FeatureTracker ft('/camera/rgb/image_rect_raw', '/feature_tracker/out');
    ros::spin(); 
    return 0; 
}
