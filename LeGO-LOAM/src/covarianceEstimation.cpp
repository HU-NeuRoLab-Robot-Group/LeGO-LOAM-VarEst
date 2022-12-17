#include "covarianceEstimation.h"

CovarianceEstimation::CovarianceEstimation(ros::NodeHandle& node)
                    : nh(node)
{

    allocateMemory();

    pubOdometryWithCovariance = nh.advertise<nav_msgs::Odometry>("/integrated_to_init_with_covariance",2);
    pubTestCloudCorner = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud_corner",2);
    pubTestCloudSurf = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud_surf",2);

    subCovMapCornerCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cov_map_corner_cloud",1, &CovarianceEstimation::CovMapCornerCloudHandler,this);
    subCovMapSurfCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cov_map_surf_cloud",1, &CovarianceEstimation::CovMapSurfCloudHandler,this);
    subLastCornerCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last",2, &CovarianceEstimation::LastCornerCloudHandler,this);
    subLastSurfCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last",2, &CovarianceEstimation::LastSurfCloudHandler,this);
    subLastOdometry = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init",5, &CovarianceEstimation::LastOdometryHandler,this);

};

CovarianceEstimation::~CovarianceEstimation(){};

void CovarianceEstimation::allocateMemory(){

    CovMapCornerCloud.reset(new pcl::PointCloud<PointType>());
    CovMapSurfCloud.reset(new pcl::PointCloud<PointType>());
    LastCornerCloud.reset(new pcl::PointCloud<PointType>());
    LastSurfCloud.reset(new pcl::PointCloud<PointType>());
    TransformedCornerCloud.reset(new pcl::PointCloud<PointType>());
    TransformedSurfCloud.reset(new pcl::PointCloud<PointType>());

};

int CovarianceEstimation::CompareTimestamps(CovarianceEstimation::timestamp T1, CovarianceEstimation::timestamp T2){

    if ((T1.sec == T2.sec) && (T1.nsec == T2.nsec)) return 0;
    else if (T1.sec > T2.sec) return 1;
    else if ((T1.sec == T2.sec) && (T1.nsec > T2.nsec)) return 1;
    else return 2;
} 

bool CovarianceEstimation::ValidateTimestamps(){

    if (CompareTimestamps(T_LastOdometry, T_LastCornerCloud) == 0){
        if (CompareTimestamps(T_LastOdometry, T_LastSurfCloud) == 0){ // O = C = S
            return 1;
        }
        else if (CompareTimestamps(T_LastOdometry, T_LastSurfCloud) == 1){ // O = C > S
            NewLastSurfCloud = 0;
        }
        else if (CompareTimestamps(T_LastOdometry, T_LastSurfCloud) == 2){ // S > O = C 
            NewLastOdometry = 0;
            NewLastCornerCloud = 0;
        }
    }
    else if (CompareTimestamps(T_LastOdometry, T_LastCornerCloud) == 1){
        if (CompareTimestamps(T_LastOdometry, T_LastSurfCloud) == 0){ // O = S > C
            NewLastCornerCloud = 0;
        }
        else if (CompareTimestamps(T_LastOdometry, T_LastSurfCloud) == 1){ // O > C and S
            NewLastCornerCloud = 0;
            NewLastSurfCloud = 0;
        }
        else if (CompareTimestamps(T_LastOdometry, T_LastSurfCloud) == 2){ // S > O > C
            NewLastOdometry = 0;
            NewLastCornerCloud = 0;
        }
    }
    else if (CompareTimestamps(T_LastOdometry, T_LastCornerCloud) == 2){
        if (CompareTimestamps(T_LastCornerCloud, T_LastSurfCloud) == 0){ // C = S > O
            NewLastOdometry = 0;
        }
        else if (CompareTimestamps(T_LastCornerCloud, T_LastSurfCloud) == 1){ // C > O and S
            NewLastOdometry = 0;
            NewLastSurfCloud = 0;
        }
        else if (CompareTimestamps(T_LastCornerCloud, T_LastSurfCloud) == 2){ // S > C > O
            NewLastOdometry = 0;
            NewLastCornerCloud = 0;
        }
    }
    ROS_INFO("\033[1;32m---->\033[0m Timestamp error. Odom = %d, Corner = %d, Surf = %d.\n", NewLastOdometry, NewLastCornerCloud, NewLastSurfCloud);
    return 0;

};

pcl::PointCloud<PointType>::Ptr CovarianceEstimation::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn){
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;
  PointType pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = cos(transformIn->yaw) * pointFrom->x -
               sin(transformIn->yaw) * pointFrom->y;
    float y1 = sin(transformIn->yaw) * pointFrom->x +
               cos(transformIn->yaw) * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
    float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

    pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 +
                transformIn->x;
    pointTo.y = y2 + transformIn->y;
    pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 +
                transformIn->z;
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }
  return cloudOut;
};

void CovarianceEstimation::CalculateCovariance(){

    NewLastCornerCloud = 0;
    NewLastSurfCloud = 0;
    NewLastOdometry = 0;

    float transform[6];

    OdometryToTransform(LastOdometry, transform);

    PointTypePose transformIn;
    
    transformIn.roll = transform[0];
    transformIn.pitch = transform[1];
    transformIn.yaw = transform[2];
    transformIn.x = transform[3];
    transformIn.y = transform[4];
    transformIn.z = transform[5];

    TransformedCornerCloud = transformPointCloud(LastCornerCloud,&transformIn);
    TransformedCornerCloud->header.frame_id = LastOdometry.header.frame_id;
    TransformedCornerCloud->header.seq = LastOdometry.header.seq;
    TransformedSurfCloud = transformPointCloud(LastSurfCloud,&transformIn);
    TransformedSurfCloud->header.frame_id = LastOdometry.header.frame_id;
    TransformedSurfCloud->header.seq = LastOdometry.header.seq;

    pubTestCloudCorner.publish(TransformedCornerCloud);
    pubTestCloudSurf.publish(TransformedSurfCloud);

    ROS_INFO("\033[1;32m---->\033[0m Covariance calculated.\n");

};

void CovarianceEstimation::CovMapCornerCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*CovMapCornerCloud);

};

void CovarianceEstimation::CovMapSurfCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*CovMapSurfCloud);

};

void CovarianceEstimation::LastCornerCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*LastCornerCloud);

    if (NewLastCornerCloud == 1) ROS_INFO("\033[1;32m---->\033[0m New Corner Message Received before using old one.\n");

    NewLastCornerCloud = 1;

    T_LastCornerCloud.sec = message->header.stamp.sec;
    T_LastCornerCloud.nsec = message->header.stamp.nsec;

};

void CovarianceEstimation::LastSurfCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*LastSurfCloud);

    if (NewLastSurfCloud == 1) ROS_INFO("\033[1;32m---->\033[0m New Surf Message Received before using old one.\n");

    NewLastSurfCloud = 1;

    T_LastSurfCloud.sec = message->header.stamp.sec;
    T_LastSurfCloud.nsec = message->header.stamp.nsec;

};

void CovarianceEstimation::LastOdometryHandler(const nav_msgs::OdometryConstPtr& message){

    LastOdometry = *message;

    if (NewLastOdometry == 1) ROS_INFO("\033[1;32m---->\033[0m New Odom Message Received before using old one.\n");

    NewLastOdometry = 1;

    T_LastOdometry.sec = message->header.stamp.sec;
    T_LastOdometry.nsec = message->header.stamp.nsec;

};

void CovarianceEstimation::run() {

    if (NewLastOdometry && NewLastCornerCloud && NewLastSurfCloud){
        if (ValidateTimestamps()){
            CalculateCovariance();
        }
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cov_est");

    ROS_INFO("\033[1;32m---->\033[0m Cov Est. Started.");

    ros::NodeHandle nh("~");

    ros::Rate rate(200);

    CovarianceEstimation CE(nh);

    while (nh.ok()) {
        
        ros::spinOnce();

        CE.run();

        rate.sleep();
        
    }

    return 0;
}