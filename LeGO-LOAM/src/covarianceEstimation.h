#ifndef COVARIANCEESTIMATION_H
#define COVARIANCEESTIMATION_H

#include "lego_loam/utility.h"
#include "lego_loam/nanoflann_pcl.h"
#include <eigen3/Eigen/Dense>

class CovarianceEstimation{

    public:

        CovarianceEstimation(ros::NodeHandle& node);

        ~CovarianceEstimation();

        void run();
    
    private:

        ros::NodeHandle& nh;

        void allocateMemory();

        ros::Publisher pubOdometryWithCovariance;
        ros::Publisher pubTestCloudCorner;
        ros::Publisher pubTestCloudSurf;

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

        pcl::PointCloud<PointType>::Ptr TransformedCornerCloud;
        pcl::PointCloud<PointType>::Ptr TransformedSurfCloud;

        bool NewLastCornerCloud = 0;
        bool NewLastSurfCloud = 0;

        nav_msgs::Odometry LastOdometry;
        nav_msgs::Odometry LastOdometryWithCovariance;

        bool NewLastOdometry = 0;

        struct timestamp {
            int sec;
            int nsec;
        } T_LastCornerCloud, T_LastSurfCloud, T_LastOdometry;

        int CompareTimestamps(CovarianceEstimation::timestamp T1, CovarianceEstimation::timestamp T2);

        bool ValidateTimestamps();
        void CalculateCovariance();

        pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

        float hausdorff_distance = 0;

        nanoflann::KdTreeFLANN<PointType> kdtreeCornerMap;
        nanoflann::KdTreeFLANN<PointType> kdtreeSurfMap;

        void getPointErrors(const pcl::PointCloud<PointType>::Ptr &MapCloud,
                            const pcl::PointCloud<PointType>::Ptr &Cloud,
                            const nanoflann::KdTreeFLANN<PointType> &KdMap);

};




#endif