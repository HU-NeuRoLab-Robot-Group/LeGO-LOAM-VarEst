#include "covarianceEstimation.h"

CovarianceEstimation::CovarianceEstimation(ros::NodeHandle& node)
                    : nh(node)
{

    allocateMemory();

    pubOdometryWithCovariance = nh.advertise<sensor_msgs::PointCloud2>("/integrated_to_init_with_covariance",2);

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

}

void CovarianceEstimation::CalculateCovariance(){

    NewLastCornerCloud = 0;
    NewLastSurfCloud = 0;
    NewLastOdometry = 0;

    printf("Covariance calculated.\n");

};

void CovarianceEstimation::CovMapCornerCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*CovMapCornerCloud);

};

void CovarianceEstimation::CovMapSurfCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*CovMapSurfCloud);

};

void CovarianceEstimation::LastCornerCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*LastCornerCloud);

    NewLastCornerCloud = 1;

    T_LastCornerCloud.sec = message->header.stamp.sec;
    T_LastCornerCloud.nsec = message->header.stamp.nsec;

};

void CovarianceEstimation::LastSurfCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message){

    pcl::fromROSMsg(*message,*LastSurfCloud);

    NewLastSurfCloud = 1;

    T_LastSurfCloud.sec = message->header.stamp.sec;
    T_LastSurfCloud.nsec = message->header.stamp.nsec;

};

void CovarianceEstimation::LastOdometryHandler(const nav_msgs::OdometryConstPtr& message){

    LastOdometry = *message;

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