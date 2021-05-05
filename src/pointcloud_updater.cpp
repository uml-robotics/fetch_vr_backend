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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <math.h>
#include <Eigen/Dense>
#include <ctime>
#include "ClusterTracker.h"

using namespace octomap;
using namespace pcl;
using namespace std;
using namespace Eigen;

// global variables
OcTree *tree = nullptr;
ClusterTracker *tracker;

// helper functions
float median(vector<float> nums);
void initializeClusterClouds(MahalanobisPointCloud &mainCloud, vector<MahalanobisPointCloud> &clouds,
                             vector<PointIndices> &indices);
void ClusterExtraction(MahalanobisPointCloud &cloud, vector<PointIndices> &indices);
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
    PointCloud<PointXYZ>::Ptr reading_cloud (new pcl::PointCloud<PointXYZ> ());
    pcl::fromPCLPointCloud2(filtered_cloud, *reading_cloud);

    // Get spherical coordinates of points in both pointclouds
    vector<vector<float> > octomap_spherical_coords = getSphericalCoordinates(octomap_cloud);
    vector<vector<float> > reading_spherical_coords = getSphericalCoordinates(*reading_cloud);

    // Calculate the covariance matrix of each distribution
    Matrix3f cov_octomap = getCovarianceMatrix(octomap_spherical_coords);
    Matrix3f octomap_inverse = cov_octomap.inverse();
    Matrix3f cov_reading = getCovarianceMatrix(reading_spherical_coords);
    Matrix3f reading_inverse = cov_reading.inverse();

    // Run Algorithm 1 from the paper on both clouds to get mahalanobis distance info for each point in both clouds
    vector<vector<float> > disappeared = getMahalanobisDistancesByIndex(octomap_cloud, *reading_cloud,
                                                                        reading_spherical_coords, reading_inverse);
    vector<vector<float> > appeared = getMahalanobisDistancesByIndex(*reading_cloud, octomap_cloud,
                                                                     octomap_spherical_coords, octomap_inverse);

    // Threshold both clouds to obtain points which have a high probability of appearing or disappearing
    float mahalanobisThreshold = 5.0;   // make this a param
    MahalanobisPointCloud DisappearedCloud, AppearedCloud;
    for (vector<float> point : disappeared) {
        if (point[1] > mahalanobisThreshold) {
            PointXYZ pclPoint = octomap_cloud.points[point[0]];
            vector<float> cloudPoint {pclPoint.x, pclPoint.y, pclPoint.z};
            DisappearedCloud.points.push_back(cloudPoint);
        }
    }
    for (vector<float> point: appeared) {
        if (point[1] > mahalanobisThreshold) {
            PointXYZ pclPoint = reading_cloud->points[point[0]];
            vector<float> cloudPoint {pclPoint.x, pclPoint.y, pclPoint.z};
            AppearedCloud.points.push_back(cloudPoint);
        }
    }

    // Run Euclidean cluster extraction on both pointclouds to separate them into clusters
    vector<PointIndices> disappearingIndices, appearingIndices;
    ClusterExtraction(DisappearedCloud, disappearingIndices);
    ClusterExtraction(AppearedCloud, appearingIndices);
    vector<MahalanobisPointCloud> disappearingClouds, appearingClouds;
    initializeClusterClouds(DisappearedCloud, disappearingClouds, disappearingIndices);
    initializeClusterClouds(AppearedCloud, appearingClouds, appearingIndices);

    // Preform some sort of classification on the clusters (random forest, thresholding)?
    // Doing simple median thresholding at the moment
    float classifyDistanceThreshold = 12.0; // Make this a param as well of course
    for (const MahalanobisPointCloud& m : appearingClouds) {
        float x = median(m.distances);
        if (x > classifyDistanceThreshold) {
            Cluster c;
            c.isAppearing = true;
            c.score = 1 - classifyDistanceThreshold / x;
            c.pointcloud = m;
            tracker->AddCluster(c);
        }
    }

    for (const MahalanobisPointCloud& m : disappearingClouds) {
        float x = median(m.distances);
        if (x > classifyDistanceThreshold) {
            Cluster c;
            c.isAppearing = false;
            c.score = 1 - classifyDistanceThreshold / x;
            c.pointcloud = m;
            tracker->AddCluster(c);
        }
    }

    // Publish object data to the Unity end (gonna have to experiment with this a bunch)

    // Will require a converter from octree point data back to voxel data
    // Have to somehow send this data in a message, custom message needed?

    // Need some sort of interface to store all proposed cluster changes, and dynamically report the "support clusters"
    // This interface should send these support clusters over to the unity project
    // Will need a cluster object and a cluster storer object
}

void initializeClusterClouds(MahalanobisPointCloud &mainCloud, vector<MahalanobisPointCloud> &clouds,
                             vector<PointIndices> &indices) {
    for (const auto & indice : indices) {
        MahalanobisPointCloud cluster;
        for (int index : indice.indices) {
            cluster.points.push_back(mainCloud.points[index]);
            cluster.distances.push_back(mainCloud.distances[index]);
        }
        clouds.push_back(cluster);
    }
}

void ClusterExtraction(MahalanobisPointCloud &cloud, vector<PointIndices > &indices) {
    PointCloud<PointXYZ>::Ptr pclCloud (new PointCloud<PointXYZ>);
    for (vector<float> point : cloud.points) {
        PointXYZ pclPoint;
        pclPoint.x = point[0];
        pclPoint.y = point[1];
        pclPoint.z = point[2];
        pclCloud->points.push_back(pclPoint);
    }
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    tree->setInputCloud(pclCloud);
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(300000);
    ec.setInputCloud(pclCloud);
    ec.extract(indices);
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
    pcdTree.setEpsilon(0.1);

    // Loop through all points of cloud_a
    // calculate the point's mahalanobis distance to its KNN in cloud_b
    // add each (index, distance) pair to the 2d vector array
    // ROS_INFO("Looping through cloud of %d points", (int)cloud_a.points.size());
    for (int i=0; i<cloud_a.points.size(); i++) {
        PointXYZ p = cloud_a[i];
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

PointCloud<PointXYZ> extract_pointcloud(PointCloud<PointXYZ> pcd, vector<int> indices) {
    PointCloud<PointXYZ> extracted_pcd;
    for (int index: indices) {
        extracted_pcd.points.push_back(pcd.points[index]);
    }
    return extracted_pcd;
}

float getMahalanobisDistance(vector<float> p, vector<int> indices, vector<vector<float> > points,
                             Matrix3f covarianceInverse) {
    // Following  the calculation steps detailed in the paper
    Vector3f maxNeighbor (1, 1, 1), p_spher = toEigenVector(p);
    float maxDist = 0;
    for (int index : indices) {
        Vector3f n = toEigenVector(points[index]);
        Vector3f V = n - p_spher;
        float dist = V.transpose() * covarianceInverse * V;
        if (dist > maxDist) {
            //maxNeighbor = n;
            maxDist = dist;
        }
    }
    Vector3f p_diff = p_spher - maxNeighbor;
    float r_p = p_spher(0);
    float d_octo = (float)tree->getResolution() / 2 ;
    Vector3f p_corr (d_octo/r_p, d_octo/r_p, d_octo);
    Vector3f p_dist = p_diff * (1 - (p_diff.cwiseAbs() / p_diff(0)).dot(p_corr / p_diff(0)));
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

float median(vector<float> nums) {
    sort(nums.begin(), nums.end());
    int size = (int)nums.size();
    return size % 2 == 0 ? ((float)nums[size/2] + (float)nums[size/2-1]) / 2 : (float)nums[size/2];
}

int main(int argc, char** argv) {
    ROS_INFO("starting pointcloud_updater");
    ros::init(argc, argv, "pointcloud_updater");
    ros::NodeHandle nh;
    ClusterTracker newTracker(nh);
    *tracker = newTracker;

    // Subscribers
    ros::Subscriber pcd_sub = nh.subscribe("/head_camera/depth_registered/points", 3, cloud_cb);
    ros::Subscriber octomap_sub = nh.subscribe("/octomap_full", 3, octomap_cb);

    ros::spin(); // or ros::spin() maybe?
}
