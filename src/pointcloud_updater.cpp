//
// Code Written by Jacob Epstein, June 2021
// Sends detected changes in the pointcloud scan to the unity end to be added
// Sends an voxels to the unity end - all points within those voxels are to be deleted
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <Eigen/Dense>
#include <ctime>
#include "ClusterTracker.h"
#include "MahalanobisHelper.h"

using namespace octomap;
using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace Mahalanobis;

// global variables
MahalanobisHelper mh;
ClusterTracker *tracker;

void cloud_cb(const sensor_msgs::PointCloud2& pcd) {
    // Store current version of tree in a local variable, interesting question for Yuri as to whether this is needed
    OcTree *tree_local = mh.tree;

    // Extract pointcloud of all the centers of the OCCUPIED voxels from the octomap into PCL format
    vector<vector<float> > octomap_voxel_centers_and_sizes = mh.extract_occupied_voxel_centers_and_sizes(tree_local);
    mh.voxel_centers_and_sizes = octomap_voxel_centers_and_sizes;
    PointCloud<PointXYZ> octomap_cloud;
    for (auto center_and_size : octomap_voxel_centers_and_sizes) {
        PointXYZ p;
        p.x = center_and_size[0];
        p.y = center_and_size[1];
        p.z = center_and_size[2];
        octomap_cloud.points.push_back(p);
    }

    // Convert the pcd msg to a PCL format, extract NANs
    PCLPointCloud2::Ptr cloud2 (new PCLPointCloud2 ());
    pcl_conversions::toPCL(pcd, *cloud2);
    PCLPointCloud2 filtered_cloud;
    PassThrough<PCLPointCloud2> filter (true);
    filter.setInputCloud(cloud2);
    filter.setFilterLimits(0, 20);
    filter.filter(filtered_cloud);
    PointCloud<PointXYZRGB>::Ptr reading_cloud (new pcl::PointCloud<PointXYZRGB> ());
    pcl::fromPCLPointCloud2(filtered_cloud, *reading_cloud);

    // Get spherical coordinates of points in both pointclouds
    vector<vector<float> > octomap_spherical_coords = mh.getSphericalCoordinates(octomap_cloud);
    vector<vector<float> > reading_spherical_coords = mh.getSphericalCoordinates(*reading_cloud);

    // Calculate the covariance matrix of each distribution
    Matrix3f cov_octomap = mh.getCovarianceMatrix(octomap_spherical_coords);
    Matrix3f octomap_inverse = cov_octomap.inverse();
    Matrix3f cov_reading = mh.getCovarianceMatrix(reading_spherical_coords);
    Matrix3f reading_inverse = cov_reading.inverse();

    // Run Algorithm 1 from the paper on both clouds to get mahalanobis distance info for each point in both clouds
    vector<vector<float> > disappeared = mh.getMahalanobisDistancesByIndex(octomap_cloud, *reading_cloud,
                                                                           reading_spherical_coords, reading_inverse);
    vector<vector<float> > appeared = mh.getMahalanobisDistancesByIndex(*reading_cloud, octomap_cloud,
                                                                        octomap_spherical_coords, octomap_inverse);

    // Threshold both clouds to obtain points which have a high probability of appearing or disappearing
    float mahalanobisThreshold = 5.0;   // make this a param
    MahalanobisPointCloud DisappearedCloud, AppearedCloud;
    AppearedCloud.isRGB = true; // IMPORTANT
    for (vector<float> point : disappeared) {
        if (point[1] > mahalanobisThreshold) {
            PointXYZ pclPoint = octomap_cloud.points[(int)point[0]];
            vector<float> cloudPoint {pclPoint.x, pclPoint.y, pclPoint.z};
            DisappearedCloud.points.push_back(cloudPoint);
        }
    }
    for (vector<float> point: appeared) {
        if (point[1] > mahalanobisThreshold) {
            PointXYZRGB pclPoint = reading_cloud->points[(int)point[0]];
            vector<float> cloudPoint {pclPoint.x, pclPoint.y, pclPoint.z, (float)pclPoint.r, (float)pclPoint.g, (float)pclPoint.b};
            AppearedCloud.points.push_back(cloudPoint);
        }
    }

    // Run Euclidean cluster extraction on both pointclouds to separate them into clusters
    vector<PointIndices> disappearingIndices, appearingIndices;
    mh.ClusterExtraction(DisappearedCloud, disappearingIndices);
    mh.ClusterExtraction(AppearedCloud, appearingIndices);
    vector<MahalanobisPointCloud> disappearingClouds, appearingClouds;
    mh.initializeClusterClouds(DisappearedCloud, disappearingClouds, disappearingIndices);
    mh.initializeClusterClouds(AppearedCloud, appearingClouds, appearingIndices);

    // Preform some sort of classification on the clusters (random forest, thresholding)?
    // Doing simple median thresholding at the moment
    float classifyDistanceThreshold = 12.0; // Make this a param as well of course
    tracker->setOctree(*tree_local);
    for (const MahalanobisPointCloud& m : appearingClouds) {
        float x = mh.median(m.distances);
        if (x > classifyDistanceThreshold) {
            Cluster c;
            c.isAppearing = true;
            c.score = 1 - classifyDistanceThreshold / x;
            c.pointcloud = m;
            tracker->AddCluster(c);
        }
    }

    for (const MahalanobisPointCloud& m : disappearingClouds) {
        float x = mh.median(m.distances);
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

void octomap_cb(const octomap_msgs::Octomap& ot) {
    // Extract binary from the octomap msg, store it in the variable tree
    AbstractOcTree* my_abstract_map = octomap_msgs::msgToMap(ot);
    mh.tree = (OcTree*) my_abstract_map;
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

    ros::spin();
}