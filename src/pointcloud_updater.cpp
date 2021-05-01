//
// Created by jacob on 4/30/21.
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

using namespace octomap;
using namespace pcl;
using namespace std;

// global variables
OcTree *tree = nullptr;

// helper functions
PointCloud<PointXYZ> extract_voxel_centers(OcTree *octo);
vector<vector<float> > getSphericalCoordinates(PointCloud<PointXYZ> pcd);
PointCloud<PointXYZ> extract_pointcloud(PointCloud<PointXYZ> pcd, vector<int> indices);
float getMahalanobisDistance(vector<float> p_spherical, vector<int> indices, vector<vector<float> > spherical_coords);
vector<float> getSphericalCoordinates(PointXYZ xyz);

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

    // Run Algorithm 1 from the paper on both clouds to get mahalanobis distance info for each point in both clouds

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

vector<vector<float> > getMahalanobisDistancesByIndex(PointCloud<PointXYZ> cloud_a, PointCloud<PointXYZ> cloud_b,
                                      vector<vector<float> > cloud_b_spherical_coords) {
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
        float mahalanobis_distance = getMahalanobisDistance(p_spherical, found_indices, cloud_b_spherical_coords);
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

float getMahalanobisDistance(vector<float> p_spherical, vector<int> indices, vector<vector<float> > spherical_coords) {
    // Get covariance matrix somehow, not sure how to do this yet

    // Follow the calculation steps detailed in the paper
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
}
