//
// Created by jacob on 6/23/21.
//
#ifndef FETCHVRBACKEND_MAHALANOBISHELPER_H
#define FETCHVRBACKEND_MAHALANOBISHELPER_H
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <std_msgs/String.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <math.h>
#include <Eigen/Dense>
#include <ctime>
#include "ClusterTracker.h"
using namespace pcl;
using namespace Eigen;

namespace Mahalanobis {
    class MahalanobisHelper {
    private:
    public:
        // Variables
        OcTree *tree = nullptr;

        // Functions for calculating mahalanobis distance
        Matrix3f getCovarianceMatrix(vector<vector<float> > points);
        vector<vector<float> > getMahalanobisDistancesByIndex(PointCloud<PointXYZRGB> cloud_a, PointCloud<PointXYZ> cloud_b,
                                                              vector<vector<float> > cloud_b_spherical_coords,
                                                              Matrix3f covarianceMatrix);
        vector<vector<float> > getMahalanobisDistancesByIndex(PointCloud<PointXYZ> cloud_a, PointCloud<PointXYZRGB> cloud_b,
                                                              vector<vector<float> > cloud_b_spherical_coords,
                                                              Matrix3f covarianceMatrix);

        // Straightforward mathematical helper functions
        float median(vector<float> nums);
        float getStandardMean(vector<vector<float> > points, int dimension);
        float getCircularMean(vector<vector<float> > points, int dimension);
        Vector3f toEigenVector(vector<float> xyz);
        vector<vector<float> > getSphericalCoordinates(PointCloud<PointXYZRGB> pcd);
        vector<vector<float> > getSphericalCoordinates(PointCloud<PointXYZ> pcd);
        template<typename PointT>
        vector<float> getSphericalCoordinates(PointT xyz);

        // Cluster extraction helper functions
        void initializeClusterClouds(MahalanobisPointCloud &mainCloud, vector<MahalanobisPointCloud> &clouds,
                                     vector<PointIndices> &indices);
        void ClusterExtraction(MahalanobisPointCloud &cloud, vector<PointIndices> &indices);
        PointCloud<PointXYZ> extract_voxel_centers(OcTree *octo);
        MahalanobisHelper() = default;
    };
}
#endif //FETCHVRBACKEND_MAHALANOBISHELPER_H
