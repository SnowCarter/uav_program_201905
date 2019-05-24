#include <ros/ros.h>
#include "imu_tf.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "imu_tf");
    ros::NodeHandle node("~");

    imu_tf::Transform transform(node);

    ros::spin();

    return 0;
}

