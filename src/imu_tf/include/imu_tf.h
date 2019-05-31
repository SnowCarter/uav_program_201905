#pragma once

#include <ros/ros.h>
#include <ros/console.h>

// TF
#include <tf/transform_broadcaster.h>


// MSG
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>



namespace imu_tf {

class Transform{

public:
    /*!
     * Constructor.
     */
    Transform(ros::NodeHandle nodehandle);

    /*!
     * Destructor.
     */
    virtual ~Transform() {}

    /*!
     * Read the inertial_tf node parameters.
     */
    void readparameters();

    /*!
     * Initialize the message data.
     */
    void initializer();

    /*!
     * Callback method for the updated IMU data.
     * @param msg: IMU broadcaster data.
     */
    //void imuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    //void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &position_msg);
    void positionCallback(const nav_msgs::Odometry::ConstPtr &position_msg);
    void hgtCallback(const sensor_msgs::NavSatFix::ConstPtr &hgt_msg);
    
    /*!
     * Publish the message data.
     * @param msg_imu_, msg_gps_, msg_pose_.
     */
    //void publish();

private:

    //! ROS nodehandle
    ros::NodeHandle nodehandle_;


    //! ROS subscribers
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_hgt;
    double acc_x,acc_y,acc_z,vx=0,vy=0,vz=0,pose_x=0,pose_y=0,pose_z=0;
    ros::Time current_time;
    ros::Time last_time;

    //TF
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform tf_t_;
    char timeCount=0;
    double _hgt = 0;
    double _hgt_org = 0;
    bool _is_init = false;

};
}

