#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/point_cloud_conversion.h>
//write data into a txt file
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Dense>

//#include <odometry_tf/odometry_tf.h>

// GRID_MAP
#include <grid_map_ros/GridMapRosConverter.hpp>
//filter
#include <pcl/filters/random_sample.h>

//kindr
#include <kindr/rotations/Rotation.hpp>
#include <kindr/phys_quant/PhysicalQuantities.hpp>
//using namespace odometry_tf;
namespace filter_kalman{
class Center_map{

public:
    Center_map(ros::NodeHandle& n);

     ~Center_map();
    void grid_map_initial();
    void publish();
	void readparameters();
    void initializer(const ros::NodeHandle &n);
    void caculatetf();
    void kalmanFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void ESMFdwq(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void estimatexy(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void dataCallback2(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);//good for pointCloud2
    void dataCallback1(const sensor_msgs::PointCloudConstPtr &cloud_msg);//just for pointcloud type data
    // void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &position_msg);
    void filterMap();
    void analyse_ESMF();
    void forget();
    void filter_outliers_(pcl::PointCloud<pcl::PointXYZ>::Ptr &origin_cloud);
    

private:
    ros::NodeHandle nodehandle_;
    ros::Subscriber sub_ ;
    ros::Subscriber sub_2 ;

    //grid map
    //ros::NodeHandle gm;
    grid_map::GridMap map_dwq;//initial the map_dwq {"elevation","variance"}
    grid_map::GridMap fusedMap_;
    ros::Publisher publisher;//advertise it use grid_map
    ros::Publisher fusedMap_pub_;

    //accuracy publish
    ros::Publisher accuracy_pub;

    //tf
    tf::TransformListener transformListener_;

    ros::Time time,last_time ;
	ros::Time stamp_;
	Eigen::Affine3d transform;

    kindr::RotationMatrixPD rotationSensorToWorld_;
    kindr::Position3D translationSensorToWorld_;

    char time_count=0;
    //odometry_tf::odometry_icp odometry_;

    double map_length_,map_resolution_,filter_radius_param_,height_compensate_,fusedMapLength_,point_cloud_type,position1_,position2_;
    std::string sub_points_,SensorFrame_,FixFrame_;
    bool use_forget_,use_KF_,use_ESMF_,use_FilterMap_,use_analyse_ESMF_,use_estimatexy_;
    std::ofstream out_height;
    std::ofstream out_xy;
    std::ofstream out_map;
    double min_height;
    double dwq_min_x,dwq_min_y,dwq_max_x,dwq_max_y;
    unsigned long int out_map_count ;

    Eigen::Matrix3d rotation_matrix_laser;
    Eigen::Vector3f translation_vector_laser; 

    //Eigen::Matrix3d rotation_matrix_uav_world;
    //Eigen::Vector3f translation_vector_uav_world;


   



};


    //一个记录点的类
    class rec_{
        std::vector<std::vector<double>>  storage_;
        public:
            rec_(){}
            void operator() (unsigned row, unsigned column){//row:hang    column:lie
                storage_.resize(row);
                for(unsigned i=0; i<row; i++){
                    storage_[i].resize(column);
                }
            }
            std::vector<double>& operator[] (unsigned i){
                return storage_[i];
            }
    };

}
