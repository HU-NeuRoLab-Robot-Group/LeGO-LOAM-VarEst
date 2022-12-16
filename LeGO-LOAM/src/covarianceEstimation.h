#ifndef COVARIANCEESTIMATION_H
#define COVARIANCEESTIMATION_H

#include "lego_loam/utility.h"

class CovarianceEstimation{

    public:

        CovarianceEstimation(ros::NodeHandle& node);

        ~CovarianceEstimation();

        void run();
    
    private:

        ros::NodeHandle& nh;

        void allocateMemory();

        ros::Publisher pubOdometryWithCovariance;

        ros::Subscriber subCovMapCornerCloud;
        ros::Subscriber subCovMapSurfCloud;

        ros::Subscriber subLastCornerCloud;
        ros::Subscriber subLastSurfCloud;

        ros::Subscriber subLastOdometry;

        void CovMapCornerCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message);
        void CovMapSurfCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message);

        void LastCornerCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message);
        void LastSurfCloudHandler(const sensor_msgs::PointCloud2ConstPtr& message);

        void LastOdometryHandler(const nav_msgs::OdometryConstPtr& message);

        pcl::PointCloud<PointType>::Ptr CovMapCornerCloud;
        pcl::PointCloud<PointType>::Ptr CovMapSurfCloud;

        pcl::PointCloud<PointType>::Ptr LastCornerCloud;
        pcl::PointCloud<PointType>::Ptr LastSurfCloud;

        bool NewLastCornerCloud = 0;
        bool NewLastSurfCloud = 0;

        nav_msgs::Odometry LastOdometry;

        bool NewLastOdometry = 0;

        struct timestamp {
            int sec;
            int nsec;
        } T_LastCornerCloud, T_LastSurfCloud, T_LastOdometry;

        int CompareTimestamps(CovarianceEstimation::timestamp T1, CovarianceEstimation::timestamp T2);

        bool ValidateTimestamps();
        void CalculateCovariance();



};




#endif