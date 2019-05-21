#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <object_marker/ObjectCbMessage.h>



ros::Publisher  object_pub_;
 
using namespace cv;
using namespace std;

 void image_cb(const sensor_msgs::ImageConstPtr& msg)
 {
   std_msgs::Header msg_header = msg->header;
   std::string frame_id = msg_header.frame_id.c_str();

   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
    Mat im = cv_ptr->image;
   // Draw an example crosshair  
    cv::Mat img_filtered(cv_ptr->image);

    cv::cvtColor(img_filtered, img_filtered, CV_BGR2HSV);

    cv::Mat blueOnly;
    cv::inRange(img_filtered, cv::Scalar(100, 140, 50), cv::Scalar(150, 190, 75), blueOnly);
  

    SimpleBlobDetector::Params params;

    params.filterByColor = false;
    params.blobColor = 255;
    params.minDistBetweenBlobs = 0.5;
    params.filterByArea = true;

    params.minArea = 100 * 100 ;

    params.maxArea = 800 * 800 +1;
    params.filterByCircularity = false;
    params.filterByColor = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;


	vector<KeyPoint> keypoints;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	detector->detect( blueOnly, keypoints);


	Mat im_with_keypoints;
	drawKeypoints( blueOnly, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

	// Show blobs
	/*imshow("keypoints", im_with_keypoints );
	waitKey(1);*/
	for(unsigned int i=0; i<keypoints.size();i++)
    {
        object_marker::ObjectCbMessage msg;
        msg.center.x =  keypoints.at(i).pt.x;
        msg.data = "barrel";
        msg.type = "barrel";
        object_pub_.publish(msg);
    }
    
    
 }
 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/robot/camera/rgb/image_raw", 1, image_cb);
  object_pub_ = nh.advertise<object_marker::ObjectCbMessage>("content",1);
  ros::spin();
  cv::destroyWindow("view");
}

