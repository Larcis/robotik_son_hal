#ifndef __OBJECT_MARKER_H__
#define __OBJECT_MARKER_H__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <math.h>
#include <string>
#include <thread>
#include <object_marker/ObjectCbMessage.h>
namespace object_marker{

using namespace std;
class ObjectMarker{
public:
    ObjectMarker();
    bool is_same(tf::Point& old_coor, tf::Point &new_coor);
    void laser_callback(const sensor_msgs::LaserScanPtr& msg); //global
    void laser_camera_adjusting(double, double&,double&,tf::Quaternion&);
    void transform_listener();
    void marker_publisher();

private:
	ros::NodeHandle nh_;
    ros::Publisher object_marker_pub;
    ros::Subscriber laser_sub, object_sub;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

    double camera_width_half_;
    double camera_fov_half_;
    double angle_coeff_;

    vector<tf::Point > old_findings;
    vector<string> old_findings_s;
    vector<string> old_findings_t;
    int f_counter;

    double tolerance;

    sensor_msgs::LaserScan laser_data_;

    visualization_msgs::MarkerArray marker_array;
    tf::TransformListener tf_;
    tf::StampedTransform camera_transform;

    int point2laser(double);
    void object_callback(const object_marker::ObjectCbMessage::ConstPtr&);
    void object_marker(string,string, tf::Point, tf::Quaternion&);
};
}

#endif
