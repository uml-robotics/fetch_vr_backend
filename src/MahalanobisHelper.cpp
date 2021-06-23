//
// Created by jacob on 6/23/21.
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <octomap/OcTree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <math.h>
#include <Eigen/Dense>
#include "ClusterTracker.h"
#include "MahalanobisHelper.h"

using namespace octomap;
using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace Mahalanobis;

/// straightforward mathematics function implementations
float MahalanobisHelper::median(vector<float> nums) {
    sort(nums.begin(), nums.end());
    int size = (int)nums.size();
    return size % 2 == 0 ? ((float)nums[size/2] + (float)nums[size/2-1]) / 2 : (float)nums[size/2];
}

float MahalanobisHelper::getStandardMean(vector<vector<float>> points, int dimension) {
    if (dimension > points[0].size())
        return 0;
    float mean = 0;
    for (vector<float> point : points) {
        mean += point[dimension];
    }
    mean /= points.size();
    return mean;
}

float MahalanobisHelper::getCircularMean(vector<vector<float>> points, int dimension) {
    if (dimension > points[0].size())
        return 0;
    float circular_mean = 0, x=0, y=0;
    for (vector<float> point : points) {
        y += sin(point[dimension]);
    }
    for (vector<float> point : points) {
        x += cos(point[dimension]);
    }
    x /= (float)points.size();
    y /= (float)points.size();
    circular_mean = atan2(y, x);
    return circular_mean;
}

Vector3f MahalanobisHelper::toEigenVector(vector<float> xyz) {
    return Vector3f (xyz[0], xyz[1], xyz[2]);
}

template<typename PointT>
vector<float> MahalanobisHelper::getSphericalCoordinates(PointT xyz) {
    vector<float> spherical_point;
    float x = xyz.x, y = xyz.y, z = xyz.z;
    float r = sqrt(x*x + y*y + z*z);
    float polar_angle = acos(z/r);
    float azimuthal_angle = atan(y/x);
    spherical_point.push_back(r);
    spherical_point.push_back(polar_angle);
    spherical_point.push_back(azimuthal_angle);
    return spherical_point;
}

vector<vector<float> > MahalanobisHelper::getSphericalCoordinates(PointCloud<PointXYZRGB> pcd) {
    vector<vector<float> > spherical_points;
    for (auto & point : pcd.points) {
        vector<float> spherical_point;
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float r = sqrt(x*x + y*y + z*z);
        int R, G, B;
        R = point.getRGBVector3i()[0];
        G = point.getRGBVector3i()[1];
        B = point.getRGBVector3i()[2];
        float polar_angle = acos(z/r);
        float azimuthal_angle = atan(y/x);
        spherical_point.push_back(r);
        spherical_point.push_back(polar_angle);
        spherical_point.push_back(azimuthal_angle);
        spherical_point.push_back((float)R);
        spherical_point.push_back((float)G);
        spherical_point.push_back((float)B);
        spherical_points.push_back(spherical_point);
    }
    return spherical_points;
}

vector<vector<float> > MahalanobisHelper::getSphericalCoordinates(PointCloud<PointXYZ> pcd) {
    vector<vector<float> > spherical_points;
    for (auto & point : pcd.points) {
        vector<float> spherical_point;
        float x = point.x;
        float y = point.y;
        float z = point.z;
        float r = sqrt(x*x + y*y + z*z);
        float polar_angle = acos(z/r);
        float azimuthal_angle = atan(y/x);
        spherical_point.push_back(r);
        spherical_point.push_back(polar_angle);
        spherical_point.push_back(azimuthal_angle);
        spherical_points.push_back(spherical_point);
    }
    return spherical_points;
}

/// Functions for calculating Mahalanobis distance
Matrix3f MahalanobisHelper::getCovarianceMatrix(vector<vector<float> > points) {
    // Loop through data, get mean in each dimension, using the "circular mean" for the angles
    vector<float> means;
    float mean_radius = this->getStandardMean(points, 0);
    float mean_polar = this->getCircularMean(points, 1);
    float mean_azimuth = this->getCircularMean(points, 2);
    means.push_back(mean_radius);
    means.push_back(mean_polar);
    means.push_back(mean_azimuth);

    // Loop through the matrix, using the proper means and data points to calculate the covariance for each cell
    Matrix3f covarianceMatrix;
    float covarianceIJ = 0;
    for (int i=0; i<3; i++) {
        for (int j= 0; j<3; j++) {
            float mean_i = means[i];
            float mean_j = means[j];
            for (vector<float> point : points) {
                covarianceIJ += (point[i] - mean_i) * (point[j] - mean_j);
            }
            covarianceIJ /= (float) points.size();
            covarianceMatrix(i, j) = covarianceIJ;
            covarianceIJ = 0;
        }
    }
    return covarianceMatrix;
}

vector<vector<float> > MahalanobisHelper::getMahalanobisDistancesByIndex(PointCloud<PointXYZ> cloud_a, PointCloud<PointXYZRGB> cloud_b,
                                                      vector<vector<float> > cloud_b_spherical_coords, Matrix3f covarianceMatrix) {
    // Intialize objects (array to be returned, KD tree for cloud_b)
    vector<vector<float> > distancesByIndex;
    search::KdTree<PointXYZRGB> pcdTree;
    PointCloud<PointXYZRGB>::Ptr bPtr (new PointCloud<PointXYZRGB> ());
    *bPtr = cloud_b;
    pcdTree.setInputCloud(bPtr);
    pcdTree.setEpsilon(0.1);

    // Loop through all points of cloud_a
    // calculate the point's mahalanobis distance to its KNN in cloud_b
    // add each (index, distance) pair to the 2d vector array
    // ROS_INFO("Looping through cloud of %d points", (int)cloud_a.points.size());
    for (int i=0; i<cloud_a.points.size(); i++) {
        PointXYZRGB p;
        p.x = cloud_a.points[i].x;
        p.y = cloud_a.points[i].y;
        p.z = cloud_a.points[i].z;
        std::vector<int> found_indices;
        std::vector<float> k_sqr_distances;
        pcdTree.nearestKSearch(p, 5, found_indices, k_sqr_distances);

        // Get the spherical coordinates of p, calculate mahalanobis distance
        vector<float> p_spherical = getSphericalCoordinates(p);
        Vector3f maxNeighbor (1, 1, 1), p_spher = toEigenVector(p_spherical);
        float maxDist = 0;
        for (int index : found_indices) {
            Vector3f n = toEigenVector(cloud_b_spherical_coords[index]);
            Vector3f V = n - p_spher;
            float dist = V.transpose() * covarianceMatrix * V;
            if (dist > maxDist) {
                maxNeighbor = n;
                maxDist = dist;
            }
        }
        Vector3f p_diff = p_spher - maxNeighbor;
        float r_p = p_spher(0);
        float d_octo = (float)tree->getResolution() / 2 ;
        Vector3f p_corr (d_octo/r_p, d_octo/r_p, d_octo);
        Vector3f p_dist = p_diff * (1 - (p_diff.cwiseAbs() / p_diff(0)).dot(p_corr / p_diff(0)));
        float mahalanobis_distance = p_dist.transpose() * covarianceMatrix * p_dist;

        // store data
        vector<float> index_and_distance;
        index_and_distance.push_back((float)i);
        index_and_distance.push_back(mahalanobis_distance);
        distancesByIndex.push_back(index_and_distance);
    }
    return distancesByIndex;
}

vector<vector<float> > MahalanobisHelper::getMahalanobisDistancesByIndex(PointCloud<PointXYZRGB> cloud_a, PointCloud<PointXYZ> cloud_b,
                                                                         vector<vector<float> > cloud_b_spherical_coords, Matrix3f covarianceMatrix) {
    // Intialize objects (array to be returned, KD tree for cloud_b)
    vector<vector<float> > distancesByIndex;
    search::KdTree<PointXYZ> pcdTree;
    PointCloud<PointXYZ>::Ptr bPtr (new PointCloud<PointXYZ> ());
    *bPtr = cloud_b;
    pcdTree.setInputCloud(bPtr);
    pcdTree.setEpsilon(0.1);

    // Loop through all points of cloud_a
    // calculate the point's mahalanobis distance to its KNN in cloud_b
    // add each (index, distance) pair to the 2d vector array
    // ROS_INFO("Looping through cloud of %d points", (int)cloud_a.points.size());
    for (int i=0; i<cloud_a.points.size(); i++) {
        PointXYZ p (cloud_a[i].x, cloud_a[i].y, cloud_a[i].z);
        std::vector<int> found_indices;
        std::vector<float> k_sqr_distances;
        pcdTree.nearestKSearch(p, 5, found_indices, k_sqr_distances);

        // Get the spherical coordinates of p, calculate mahalanobis distance
        vector<float> p_spherical = getSphericalCoordinates(p);
        Vector3f maxNeighbor (1, 1, 1), p_spher = toEigenVector(p_spherical);
        float maxDist = 0;
        for (int index : found_indices) {
            Vector3f n = toEigenVector(cloud_b_spherical_coords[index]);
            Vector3f V = n - p_spher;
            float dist = V.transpose() * covarianceMatrix * V;
            if (dist > maxDist) {
                maxNeighbor = n;
                maxDist = dist;
            }
        }
        Vector3f p_diff = p_spher - maxNeighbor;
        float r_p = p_spher(0);
        float d_octo = (float)tree->getResolution() / 2 ;
        Vector3f p_corr (d_octo/r_p, d_octo/r_p, d_octo);
        Vector3f p_dist = p_diff * (1 - (p_diff.cwiseAbs() / p_diff(0)).dot(p_corr / p_diff(0)));
        float mahalanobis_distance = p_dist.transpose() * covarianceMatrix * p_dist;

        // store data
        vector<float> index_and_distance;
        index_and_distance.push_back((float)i);
        index_and_distance.push_back(mahalanobis_distance);
        distancesByIndex.push_back(index_and_distance);
    }
    return distancesByIndex;
}

///Helper Functions for pointcloud cluster extraction
void MahalanobisHelper::initializeClusterClouds(MahalanobisPointCloud &mainCloud, vector<MahalanobisPointCloud> &clouds,
                             vector<PointIndices> &indices) {
    for (const auto & indice : indices) {
        MahalanobisPointCloud cluster;
        for (int index : indice.indices) {
            cluster.points.push_back(mainCloud.points[index]);
            cluster.distances.push_back(mainCloud.distances[index]);
            if (mainCloud.isRGB) {
                cluster.isRGB = true;   // not really needed now, but good to have
            }
        }
        clouds.push_back(cluster);
    }
}

void MahalanobisHelper::ClusterExtraction(MahalanobisPointCloud &cloud, vector<PointIndices > &indices) {
    PointCloud<PointXYZRGB>::Ptr pclCloud (new PointCloud<PointXYZRGB>);
    for (vector<float> point : cloud.points) {
        PointXYZRGB pclPoint;
        pclPoint.x = point[0];
        pclPoint.y = point[1];
        pclPoint.z = point[2];
        pclCloud->points.push_back(pclPoint);
    }
    search::KdTree<PointXYZRGB>::Ptr t (new search::KdTree<PointXYZRGB>);
    t->setInputCloud(pclCloud);
    EuclideanClusterExtraction<PointXYZRGB> ec;
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(300000);
    ec.setInputCloud(pclCloud);
    ec.extract(indices);
}

PointCloud<PointXYZ> MahalanobisHelper::extract_voxel_centers(OcTree *octo) {
    PointCloud<PointXYZ> cloud_out;
    for (OcTree::leaf_iterator it = octo->begin_leafs(), end = octo->end_leafs(); it != end; it++) {
        PointXYZ point;
        point.x = (float)it.getX();
        point.y = (float)it.getY();
        point.z = (float)it.getZ();
        cloud_out.points.push_back(point);
    }
    return cloud_out;
}