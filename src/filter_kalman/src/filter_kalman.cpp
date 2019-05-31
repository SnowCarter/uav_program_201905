#include "filter_kalman.h"
//#include <odometry_tf/odometry_tf.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
using namespace filter_kalman;

Center_map::Center_map(ros::NodeHandle& n)
    :map_dwq({"elevationh","elevationl","sigmah","sigmal","average","pointN"}),
      nodehandle_(n),
      out_height("/program/catkin_ws/src/filter_kalman/include/height.txt"),
      out_xy("/program/catkin_ws/src/filter_kalman/include/out_xy.txt"),
      out_map("/program/catkin_ws/src/filter_kalman/include/out_map.txt")
{

}
Center_map::~Center_map(){}
void Center_map::readparameters() {
    nodehandle_.param("map_length",     map_length_,        double(30.0));
    nodehandle_.param("map_resolution", map_resolution_,    double(1)   );
    nodehandle_.param("filter_radius",  filter_radius_param_,     double(0.5) );
    nodehandle_.param("height_compensate",height_compensate_,      double(0));
    nodehandle_.param("fusedMapLength",   fusedMapLength_,         double(1000.0));
    nodehandle_.param("point_cloud_type", point_cloud_type,         double(1.0));
    nodehandle_.param("position1", position1_,   double(0.0));
    nodehandle_.param("position2", position2_,   double(0.0));

    nodehandle_.param("sub_points",     sub_points_,        std::string("/kitti/velo/pointcloud"));
    nodehandle_.param("SensorFrame",    SensorFrame_,       std::string("/base_link")            );
    nodehandle_.param("FixFrame",       FixFrame_,          std::string("/world")                );

    nodehandle_.param("use_forget",     use_forget_,        bool(false));
    nodehandle_.param("use_KF",         use_KF_,            bool(true));
    nodehandle_.param("use_ESMF",       use_ESMF_,          bool(false));
    nodehandle_.param("use_FilterMap",  use_FilterMap_,     bool(false));
    nodehandle_.param("use_analyse_ESMF",  use_analyse_ESMF_,     bool(true));
    nodehandle_.param("use_estimatexy", use_estimatexy_,    bool(false));

    min_height = 1.6;

//
//    nodehandle_.param("use_setPosition",use_setPosition_,   bool(true));



    ROS_INFO("i'm in readparameters");
}

void Center_map::initializer(const ros::NodeHandle &n)
{
    readparameters();
    //sub_2 = nodehandle_.subscribe("/mavros/local_position/pose", 10 , &Center_map::positionCallback,this);
    sub_  = nodehandle_.subscribe(sub_points_, 1 ,&Center_map::dataCallback2,this);
    

    ROS_INFO("i'm in initializer");
    grid_map_initial();
    out_map_count =0;
    //添加刚体变换矩阵 T=3x1 & R=3x3
    translation_vector_laser << 0.154796,0.019059,0.1760485;
    rotation_matrix_laser<<  0.1575, 0.904, -0.968,
                            -1.903 ,  0.342, -0.083,
                             0.3146, 0.919,  0.238;

}


//first pointCloud2 type process
void Center_map::dataCallback2(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{

    time = ros::Time::now();
    stamp_ = cloud_msg->header.stamp;
    //ROS_INFO("i'm in datacallback and the cloud_msg_stamp is %lf",stamp_.toSec());
    pcl::PCLPointCloud2 pcl_PP2;
    pcl_conversions::toPCL(*cloud_msg, pcl_PP2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_PP2, *origin_cloud);


    //filter statistic outliers
    filter_outliers_(origin_cloud);

    //if(!odometry_.updateEstimate(*origin_cloud)) return;

    caculatetf();

//+++++++++++forget old elevation of map if no update++++++++++++++++++++//
   if(use_forget_) forget();


    if(use_KF_) kalmanFilter(origin_cloud);


//***************new algorithm of ESMF from pf.gu**************//
    if(use_ESMF_) ESMFdwq(origin_cloud);
//*********************估计一个物体的xy坐标***********************//
    if(use_estimatexy_) estimatexy(origin_cloud);

    if(use_FilterMap_) filterMap();//smooth the map similar with forget();
    
    analyse_ESMF();
   // ROS_INFO("After Height Estimate");

    publish();

    //ROS_INFO("i get out buddy");

}

void Center_map::estimatexy(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

}

void Center_map::analyse_ESMF(){//
    Eigen::Vector3d position_point;

    position_point=transform.translation().matrix();//store 无人机所在中心点的位置
    float height_between_land_and_uav;//无人机与地面之间的距离
    double filter_radius_ = 0.5;
    double mean = 0.0;
    double sumOfWeight = 0.0;

    grid_map::Position currentPosition_(position_point(0),position_point(1));//get无人机所在中心点的位置
    for(grid_map::CircleIterator circleIt(map_dwq, currentPosition_,filter_radius_); !circleIt.isPastEnd(); ++circleIt){
        if(map_dwq.isValid(*circleIt,"elevationh")) {
            grid_map::Position currentPositionInCircle;
            map_dwq.getPosition(*circleIt, currentPositionInCircle);

            double distance = (currentPosition_ - currentPositionInCircle).norm();

            double weight = 100.0*pow(filter_radius_ - distance, 2);

            // std::cout <<"distance, weight: " << distance <<"," << weight << std::endl;


            mean += weight * map_dwq.at("elevationh", *circleIt);
            sumOfWeight += weight;      
        }  
    }
    std::cout <<"  height: " << (position_point(2) - (mean / sumOfWeight)) << std::endl;

    if(position_point(2) - (mean / sumOfWeight)>=0)
    {
        height_between_land_and_uav = (float)(position_point(2) - (mean / sumOfWeight));
        
        // std::cout << "mean, sumOfweight, position_point(2), distance"<< mean << ","<< sumOfWeight << ","<< position_point(2) << ","<< height_between_land_and_uav << std::endl;
        

        if(height_between_land_and_uav<100 && height_between_land_and_uav >=0){
            geometry_msgs::PoseStamped geometry_publish_data;
            geometry_publish_data.pose.position.z = height_between_land_and_uav;
            geometry_publish_data.header.stamp = ros::Time::now();
            // there are no changes to other elements.
            accuracy_pub.publish(geometry_publish_data);
            //std::cout << "the x,y,z postion of UAV:"<<position_point(0)<< ","<< position_point(1)<< ","<< position_point(2) << std::endl;
        }
       
    }


}

void Center_map::filter_outliers_(pcl::PointCloud<pcl::PointXYZ>::Ptr &origin_cloud){


}


void Center_map::grid_map_initial()
{
    publisher=nodehandle_.advertise<grid_map_msgs::GridMap>("grid_map",1,true);//advertise it use grid_map
    
    map_dwq.setFrameId("world");
    
//    map_dwq.setGeometry(grid_map::Length(60,60),1);//,grid_map::Position(10,0));//south outdoor 30 30 0.4 is good
    map_dwq.setGeometry(grid_map::Length(map_length_,map_length_),map_resolution_);
   
    accuracy_pub = nodehandle_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1000);
}

void Center_map::caculatetf(){
    /**************  tf begin **************************/
    try{
       
        tf::StampedTransform transformTf;

        //sensor data transform into world frame use the struct of transformTf
        transformListener_.lookupTransform(FixFrame_,SensorFrame_,ros::Time(0),transformTf);
       
        //now i get the rotation and transform matrix
        poseTFToEigen(transformTf, transform);
       
        rotationSensorToWorld_.setMatrix(transform.rotation().matrix());
        translationSensorToWorld_.toImplementation() = transform.translation();


    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        //return false;
    }


    //first we should create a vector3d
    Eigen::Vector3d position_point;
  //  //second get the position from odometry
    position_point=transform.translation().matrix();

    //  //third create the grid_map positon
    grid_map::Position position_grid_map(position_point(0),position_point(1));//3  20
   
    map_dwq.move(position_grid_map);

   // ROS_INFO("position is %d x %d",(int)position2(0),(int)position2(1));


}

void Center_map::forget(){
   
}
void Center_map::kalmanFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

}

//***************new algorithm of ESMF from pf.gu**************//
void Center_map::ESMFdwq(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
   //std::cout<<"ESMF:"<<std::endl;

    float sigmaek=0.0036;//only have variance of velodyne 0.0036 this value is faster than 0.06
    float pk=0.1;//influence of noise from the measurement
    float deltak=0;


    for(unsigned int i=0;i<cloud->size();++i){
        auto &point = cloud->points[i];//get the point cloud from laser

       //change the point to world frame from sensor frame:transform
        Eigen::Vector3f point_in_sensor(point.x,point.y,point.z);

        Eigen::Vector3f point_in_robot;
        Eigen::Vector3f point_in_world;

        // u can get the R and T by test using motion capture.
       // point_in_robot = ((rotation_matrix_laser.cast<float>() * point_in_sensor) + translation_vector_laser);
       point_in_robot = point_in_sensor;
        // translate the point cloud into world coordinate from robotic coordinate using odometry information which can be got by imu_tf.node.
        point_in_world = ((rotationSensorToWorld_.toImplementation().cast<float>())*point_in_robot)
                          +(translationSensorToWorld_.toImplementation().cast<float>());
                       
        grid_map::Index index;
        grid_map::Position position(point_in_world(0),point_in_world(1));
        if (!map_dwq.getIndex(position,index) || sqrt(point.x*point.x + point.y*point.y +point.z*point.z)<0.5) continue;
        auto &elevationh = map_dwq.at("elevationh",index);
        auto &elevationl  = map_dwq.at("elevationl",index);
        auto &sigmah  = map_dwq.at("sigmah",index);
        auto &sigmal  = map_dwq.at("sigmal",index);
        auto &average  = map_dwq.at("average",index);
        auto &pointN  = map_dwq.at("pointN",index);

        double height = point_in_world(2)+height_compensate_;


        if (!map_dwq.isValid(index,"elevationh"))//if have no data,putting the point in it
        {
            elevationh = elevationl = average = height;
            sigmah = sigmal = sigmaek;
            pointN=1;
            continue;
        }
        //**************new algorithm of ESMF  ************************//

        if((height > elevationh)&& (std::abs(height-elevationh)/std::sqrt(sigmah) >1)){
            if(pointN < 20) pointN++;//防止内存溢出
            if(pointN>10){//防止噪声点干扰，效果不错，不去除中心半径球范围内的点的话，会有轨迹点
                pointN = 0;
                elevationh = height;
                sigmah = sigmaek;
                //std::cout<<"hight="<<std::abs(height-elevationh)/std::sqrt(sigmah)<<std::endl;
                continue;
            }
            

        }
        else if((height < elevationh) && (std::abs(height-elevationh)/std::sqrt(sigmah) >1)){

            //std::cout<<"low="<<std::abs(height-elevationh)/std::sqrt(sigmah)<<std::endl;
           continue;

        }
        else{
            auto elevationh_pre = elevationh;
            pk=sqrt(sigmaek)/(sqrt(sigmaek)+sqrt(sigmah));//pk should be update accorading to Japanese article
            elevationh = elevationh_pre + sigmah*(height-elevationh_pre)*(pk-pk*pk)/(pk*sigmah+(1-pk)*sigmaek);
            sigmah = sigmah*sigmaek/(pk*sigmah+(1-pk)*sigmaek);
            deltak = pk*(1-pk)*pow(height - elevationh_pre,2)/(pk*sigmah+(1-pk)*sigmaek);
            sigmah = (1-deltak)*sigmah;
            //std::cout<<sigmah<<std::endl;
        }






    }

}

void Center_map::filterMap(){
    
}

void Center_map::publish(){

    map_dwq.setTimestamp(stamp_.toNSec());
    grid_map_msgs::GridMap message;
    message.info.header.frame_id="world";
    grid_map::GridMapRosConverter::toMessage(map_dwq,message);
    publisher.publish(message);

}

//Second point cloud type data process
void Center_map::dataCallback1(const sensor_msgs::PointCloudConstPtr &cloud_msg)
{
   

}
