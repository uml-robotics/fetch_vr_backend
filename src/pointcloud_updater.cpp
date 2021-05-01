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

using namespace octomap;
OcTree *tree = nullptr;
void cloud_cb(sensor_msgs::PointCloud2 pcd) {
    // Store current version of tree in a local variable, interesting question for Yuri as to whether this is needed

    // Extract pointcloud of all the centers of the voxels from the octomap into PCL format

    // Convert the pcd msg to a PCL format

    // Store both clouds in separate K-D trees

    // Run Algorithm 1 from the paper on to get mahalanobis distance info for each point in both clouds

    // Threshold both clouds to obtain points which have a high probability of appearing or disappearing

    // Run HBDSCAN clustering on both pointclouds to separate them into objects

    // Preform some sort of classification on the objects

    // Publish object data to the Unity end (gonna have to experiment with this a bunch)
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
