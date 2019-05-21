#include "object_marker/object_marker.h"
#include <time.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "object_marking");
    object_marker::ObjectMarker obj_marker;
    ros::spin();
    return 0;
}

namespace object_marker{

#define OFFSET (0.065)

ObjectMarker::ObjectMarker() : nh_(), f_counter(0)  {
    srand(time(NULL));
    laser_sub = nh_.subscribe("/robot/hokuyo", 1, &ObjectMarker::laser_callback, this);
    object_sub = nh_.subscribe("/content", 1, &ObjectMarker::object_callback, this);

    camera_fov_half_ = nh_.param("camera_fov_degree",80.)/2.;
    camera_width_half_ = nh_.param("camera_width",800.)/2.;
    angle_coeff_ = camera_width_half_ / camera_fov_half_ * 180 / M_PI;
    tolerance = nh_.param("tolerance",1.);

    
    object_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/detected", 1);
    f_counter = 0;
    thread transform_thread(&ObjectMarker::transform_listener, this);
    thread publisher_thread(&ObjectMarker::marker_publisher, this);
    transform_thread.detach();
    publisher_thread.detach();
}

int ObjectMarker::point2laser(double x)
{
    double obj_cam_angle = (x - camera_width_half_) / angle_coeff_;
    double obj_laser_angle = atan2(
        laser_data_.ranges.at(laser_data_.ranges.size()/2) * tan(obj_cam_angle),
        laser_data_.ranges.at(laser_data_.ranges.size()/2) - OFFSET
    );

    return laser_data_.ranges.size()/2 - (int) (obj_laser_angle / laser_data_.angle_increment);
};


bool ObjectMarker::is_same(tf::Point& new_coor, tf::Point& old_coor)
{
    return (fabs(new_coor.x() - old_coor.x()) <= tolerance) &&
           (fabs(new_coor.y() - old_coor.y()) <= tolerance);
}

void ObjectMarker::laser_callback(const sensor_msgs::LaserScanPtr& msg)
{
    laser_data_ = *msg;
}

void ObjectMarker::object_callback(const object_marker::ObjectCbMessage::ConstPtr&  msg)
{
    if(laser_data_.ranges.size() == 0)
        return;
    
    double _x,_y,_z;

    std::tie(_x, _y, _z) = std::make_tuple(msg->center.x, msg->center.y, msg->center.z);

    int laser_index = point2laser(_x);
    double dist_angle = laser_data_.angle_min + laser_index * laser_data_.angle_increment;
    
    double x = cos(dist_angle) * laser_data_.ranges.at(laser_index);
    double y = sin(dist_angle) * laser_data_.ranges.at(laser_index);

    tf::Vector3 coords(x,y,0);
    tf::Point new_coor = camera_transform * coords;
    tf::Quaternion new_ori = camera_transform.getRotation() * -1;

    int i = 0;
    bool isOk = true;
    while(i < old_findings.size() && isOk){
        if(is_same(new_coor, old_findings.at(i))){
           if(old_findings_t.at(i) == msg->type){
               if(old_findings_s.at(i) == msg->data){
                    isOk = false;
               }   
           }else{
                if(old_findings_t.at(i) == "QR"){
                    isOk = false;
                }
           } 
        }  
        i++;
    }
    if(isOk)
        object_marker(msg->data,msg->type, new_coor, new_ori);
}

void ObjectMarker::object_marker(string data,string type,  tf::Point coordinate, tf::Quaternion& orientation)
{
    if(type.length() < 2)
        return;
    f_counter++;
    ROS_INFO("[OBJECT MARKER] : %s FOUND! Number of findings:%d", data.c_str(), f_counter);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = data + to_string(f_counter);
    marker.id = f_counter;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = coordinate.getX();
    marker.pose.position.y = coordinate.getY();
    marker.pose.position.z = 0;
    marker.scale.z = 0.2;
    marker.pose.orientation.x = orientation.getX();
    marker.pose.orientation.y = orientation.getY();
    marker.pose.orientation.z = orientation.getZ();
    marker.pose.orientation.w = orientation.getW();
    
    marker.color.r = marker.color.g = marker.color.b = 1.0;
    if(type =="barrel"){
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }else if(type =="yangin_sondurucu"){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }else if(type != "QR"){
        marker.color.r *= rand() % 255 / 255.;
        marker.color.g *= rand() % 255 / 255.;
        marker.color.b *= rand() % 255 / 255.;
    }

    marker.color.a = 1.0;
    marker.text = data ;//+ to_string(f_counter);
    marker_array.markers.push_back(marker);
    old_findings.push_back(coordinate);
    old_findings_s.push_back(data);
    old_findings_t.push_back(type);
}

void ObjectMarker::transform_listener() {

    tf_.waitForTransform("/map", "/robot/hokuyo_frame", ros::Time::now(), ros::Duration(2));
    while(nh_.ok()) {
        try {
            tf_.lookupTransform("map", "/robot/hokuyo_frame", ros::Time(0), camera_transform);
        } catch(tf::TransformException e) {
            //ROS_INFO("Transform bekleniyor...");
            ros::Duration(0.1).sleep();
        }
    }

}

void ObjectMarker::marker_publisher() {
    while(nh_.ok()) {
        object_marker_pub.publish(marker_array);
        ros::Duration(0.1).sleep();
    }
}

}
