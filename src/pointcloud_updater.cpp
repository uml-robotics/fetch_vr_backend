//
// Code to find additions and disappearances in the stored cloud on the unity end, sends appropriate updates to unity
// Written by Jacob Epstein, May 2021
//

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
#include <pcl/PointIndices.h>
#include <math.h>
#include <Eigen/Dense>

using namespace octomap;
using namespace pcl;
using namespace std;
using namespace Eigen;

// global variables
OcTree *tree = nullptr;

// helper functions
Vector3f toEigenVector(vector<float> xyz);
PointCloud<PointXYZ> extract_voxel_centers(OcTree *octo);
vector<vector<float> > getSphericalCoordinates(PointCloud<PointXYZ> pcd);
PointCloud<PointXYZ> extract_pointcloud(PointCloud<PointXYZ> pcd, vector<int> indices);
float getMahalanobisDistance(vector<float> p_spherical, vector<int> indices, vector<vector<float> > points,
                             Matrix3f covarianceMatrix);
vector<float> getSphericalCoordinates(PointXYZ xyz);
Matrix3f getCovarianceMatrix(vector<vector<float> > points);
float getStandardMean(vector<vector<float> > points, int dimension);
float getCircularMean(vector<vector<float> > points, int dimension);
vector<vector<float> > getMahalanobisDistancesByIndex(PointCloud<PointXYZ> cloud_a, PointCloud<PointXYZ> cloud_b,
                                                      vector<vector<float> > cloud_b_spherical_coords,
                                                      Matrix3f covarianceMatrix);

void cloud_cb(sensor_msgs::PointCloud2 pcd) {
    // Store current version of tree in a local variable, interesting question for Yuri as to whether this is needed
    OcTree *tree_local = tree;

    // Extract pointcloud of all the centers of the voxels from the octomap into PCL format
    PointCloud<PointXYZ> octomap_cloud = extract_voxel_centers(tree_local);

    // Convert the pcd msg to a PCL format, extract NANs
    PCLPointCloud2::Ptr cloud2 (new PCLPointCloud2 ());
    pcl_conversions::toPCL(pcd, *cloud2);
    PCLPointCloud2 filtered_cloud;
    PassThrough<PCLPointCloud2> filter (true);
    filter.setInputCloud(cloud2);
    filter.setFilterLimits(0, 20);
    filter.filter(filtered_cloud);
    PointCloud<PointXYZ>::Ptr reading_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(filtered_cloud, *reading_cloud);

    // Get spherical coordinates of points in both pointclouds
    vector<vector<float> > octomap_spherical_coords = getSphericalCoordinates(octomap_cloud);
    vector<vector<float> > reading_spherical_coords = getSphericalCoordinates(*reading_cloud);

    // Calculate the covariance matrix of each distribution
    Matrix3f cov_octomap = getCovarianceMatrix(octomap_spherical_coords);
    Matrix3f cov_reading = getCovarianceMatrix(reading_spherical_coords);

    // Run Algorithm 1 from the paper on both clouds to get mahalanobis distance info for each point in both clouds
    vector<vector<float> > disappeared = getMahalanobisDistancesByIndex(octomap_cloud, *reading_cloud,
                                                                        reading_spherical_coords, cov_reading);
    vector<vector<float> > appeared = getMahalanobisDistancesByIndex(*reading_cloud, octomap_cloud,
                                                                     octomap_spherical_coords, cov_octomap);

    // Threshold both clouds to obtain points which have a high probability of appearing or disappearing

    // Run HBDSCAN clustering on both pointclouds to separate them into objects

    // Preform some sort of classification on the objects

    // Publish object data to the Unity end (gonna have to experiment with this a bunch)

    // Will require a converter from octree point data back to voxel data
    // Have to somehow send this data in a message, custom message needed?

    // Need some sort of interface to store all proposed cluster changes, and dynamically report the "support clusters"
    // This interface should send these support clusters over to the unity project
    // Will need a cluster object and a cluster storer object
}

Matrix3f getCovarianceMatrix(vector<vector<float> > points) {
    // Loop through data, get mean in each dimension, using the "circular mean" for the angles
    vector<float> means;
    float mean_radius = getStandardMean(points, 0);
    float mean_polar = getCircularMean(points, 1);
    float mean_azimuth = getCircularMean(points, 2);
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

float getStandardMean(vector<vector<float> > points, int dimension) {
    if (dimension > points[0].size())
        return 0;
    float mean = 0;
    for (vector<float> point : points) {
        mean += point[dimension];
    }
    mean /= points.size();
    return mean;
}

// A more appropriate mean function for angles.
float getCircularMean(vector<vector<float> > points, int dimension) {
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

vector<vector<float> > getMahalanobisDistancesByIndex(PointCloud<PointXYZ> cloud_a, PointCloud<PointXYZ> cloud_b,
                                      vector<vector<float> > cloud_b_spherical_coords, Matrix3f covarianceMatrix) {
    // Intialize objects (array to be returned, KD tree for cloud_b)
    vector<vector<float> > distancesByIndex;
    search::KdTree<PointXYZ> pcdTree;
    PointCloud<PointXYZ>::Ptr bPtr (new PointCloud<PointXYZ> ());
    *bPtr = cloud_b;
    pcdTree.setInputCloud(bPtr);

    // Loop through all points of cloud_a
    // calculate the point's mahalanobis distance to its KNN in cloud_b
    // add each (index, distance) pair to the 2d vector array
    for (int i=0; i<cloud_a.points.size(); i++) {
        PointXYZ p = cloud_a[i];
        std::vector<int> found_indices;
        std::vector<float> k_sqr_distances;
        pcdTree.nearestKSearch(p, 100, found_indices, k_sqr_distances);
        // PointCloud<PointXYZ> neighbors = extract_pointcloud(cloud_a, found_indices);

        // Get the spherical coordinates of p, calculate mahalanobis distance
        vector<float> p_spherical = getSphericalCoordinates(p);
        float mahalanobis_distance = getMahalanobisDistance(p_spherical, found_indices, cloud_b_spherical_coords,
                                                            covarianceMatrix);
        vector<float> index_and_distance;
        index_and_distance.push_back((float)i);
        index_and_distance.push_back(mahalanobis_distance);
        distancesByIndex.push_back(index_and_distance);
    }
    return distancesByIndex;
}

PointCloud<PointXYZ> extract_pointcloud(PointCloud<PointXYZ> pcd, vector<int> indices) {
    PointCloud<PointXYZ> extracted_pcd;
    for (int index: indices) {
        extracted_pcd.points.push_back(pcd.points[index]);
    }
    return extracted_pcd;
}

float getMahalanobisDistance(vector<float> p, vector<int> indices, vector<vector<float> > points,
                             Matrix3f covarianceMatrix) {
    // Following  the calculation steps detailed in the paper
    Matrix3f covarianceInverse = covarianceMatrix.inverse();
    Vector3f maxNeighbor (0, 0, 0), p_spher = toEigenVector(p);
    float maxDist = 0;
    for (int index : indices) {
        Vector3f n = toEigenVector(points[index]);
        Vector3f V = n - p_spher;
        if (V.transpose() * covarianceInverse * V > maxDist) {
            maxNeighbor = n;
        }
    }
    Vector3f p_diff = p_spher - maxNeighbor;
    float r_p = p_spher(0);
    float d_octo = (float)tree->getResolution() / 2 ;
    Vector3f p_corr (d_octo/r_p, d_octo/r_p, d_octo);
    Vector3f p_dist = p_diff * (1 - 1);   // final line to add in tomorrow, magnitude vs norm of vector in spherical coords?
    return p_dist.transpose() * covarianceInverse * p_dist;
}

Vector3f toEigenVector(vector<float> xyz) {
    return Vector3f (xyz[0], xyz[1], xyz[2]);
}

PointCloud<PointXYZ> extract_voxel_centers(OcTree *octo) {
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

vector<float> getSphericalCoordinates(PointXYZ xyz) {
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

vector<vector<float> > getSphericalCoordinates(PointCloud<PointXYZ> pcd) {
    vector<vector<float> > spherical_points;
    for (int i=0; i<pcd.points.size(); i++ ) {
        vector<float> spherical_point;
        float x = pcd.points[i].x;
        float y = pcd.points[i].y;
        float z = pcd.points[i].z;
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

void octomap_cb(octomap_msgs::Octomap ot) {
    // Extract binary from the octomap msg, store it in the variable tree
    AbstractOcTree* my_abstract_map = octomap_msgs::msgToMap(ot);
    tree = (OcTree*) my_abstract_map;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_updater");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber pcd_sub = nh.subscribe("/head_camera/depth_registered/points", 1000, cloud_cb);
    ros::Subscriber octomap_sub = nh.subscribe("/octomap_full", 1000, octomap_cb);

    ros::waitForShutdown(); // or ros::spin() maybe?
}
