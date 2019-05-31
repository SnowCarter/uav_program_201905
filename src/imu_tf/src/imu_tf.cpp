#include "imu_tf.h"

using namespace imu_tf;
Transform::Transform(ros::NodeHandle nodehandle)
	:nodehandle_(nodehandle)
{
	readparameters();
	initializer();
    // sub_imu_=nodehandle.subscribe("/imu/data",1,&Transform::imuCallback,this);
    sub_imu_=nodehandle.subscribe("/mavros/global_position/local", 10 , &Transform::positionCallback,this);
    sub_hgt=nodehandle.subscribe("/mavros/global_position/raw/fix", 10 , &Transform::hgtCallback,this);
    
}
void Transform::readparameters()
{

}
void Transform::initializer()
{
//    current_time = ros::Time::now();
//    last_time = ros::Time::now();


}

void Transform::hgtCallback(const sensor_msgs::NavSatFix::ConstPtr &hgt_msg){
    
    if (!_is_init){
        _hgt_org = hgt_msg->altitude;
        _is_init = true;
    }
    _hgt = hgt_msg->altitude - _hgt_org;
}

//void Transform::imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
void Transform::positionCallback(const nav_msgs::Odometry::ConstPtr &position_msg){

    //current_time = ros::Time::now();
    //current_time = position_msg->header.stamp;
    if (timeCount<1)
    {
        last_time=current_time;
        timeCount=1;
    }
   // ROS_INFO("time=%lf",double(current_time.toSec()));

//set origin
    tf_t_.setOrigin(tf::Vector3(position_msg->pose.pose.position.x,
            position_msg->pose.pose.position.y,_hgt));

    tf::Quaternion tf_q_(position_msg->pose.pose.orientation.x,position_msg->pose.pose.orientation.y,
            position_msg->pose.pose.orientation.z,position_msg->pose.pose.orientation.w);

    tf_t_.setRotation(tf_q_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_,position_msg->header.stamp,"/world","/uav"));

}
