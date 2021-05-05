//
// Created by jacob on 5/5/21.
// Functions for the ClusterTracker class
//

#include "ClusterTracker.h"
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <flann/flann.hpp>
#include <math.h>

using namespace std;

ClusterTracker::ClusterTracker(ros::NodeHandle nh) {
   this->maxBboxDim = 0;
   this->additionPub = nh.advertise<std_msgs::String>(this->additionTopic, 10); // Will end up publishing a pointcloud here
   this->deletionPub = nh.advertise<std_msgs::String>(this->deletionTopic, 10); // Will end up publishing some sort of voxel message here
   flann::Matrix<float> dataset;
   flann::Index<flann::L2_3D<float> > newTree(dataset, flann::KDTreeIndexParams(4));
   *this->index = newTree;
}

void ClusterTracker::AddCluster(Cluster &c) {
    // Calculate the bounding box of cluster c
    ClusterTracker::calculateBbox(c);

    // Update maxBboxDim
    float m = max(c.bbox.length, c.bbox.width, c.bbox.height);
    if (m > this->maxBboxDim) {
        this->maxBboxDim = m;
    }

    // Insert Bbox center into the Flann KDTree
    flann::Matrix<float> dataset(new float[3], 1, 3);
    dataset[0][0] = c.bbox.center[0];
    dataset[0][1] = c.bbox.center[1];
    dataset[0][2] = c.bbox.center[2];
    this->index->addPoints(dataset);

    // Insert Bbox center : Bbox object into the dictionary
    this->clustersByCenter[c.bbox.center] = c;

    // Call updateSupports method on cluster c
    this->updateSupports(c);
}

void ClusterTracker::updateSupports(Cluster &c) {
    // search in maxBbox + maxCdim radius around the center of c

    // for all found cluster centers, look up the corresponding cluster object

    // call publish if support on each of these clusters
}

void ClusterTracker::calculateBbox(Cluster &c) {
    // Calculate axis aligned Bbox by finding max and min coord in each direction
    // If this is slow, could probably just approximate the Bbox
    float minX = c.pointcloud.points[0][0], maxX = 0, minY = c.pointcloud.points[0][1], maxY = 0
            , minZ = c.pointcloud.points[0][2], maxZ = 0;
    for (vector<float> point : c.pointcloud.points) {
        if (point[0] < minX)
            minX = point[0];
        else if (point[0] > maxX)
            maxX = point[0];
        if (point[1] < minY)
            minY = point[1];
        else if (point[1] > maxY)
            maxY = point[1];
        if (point[2] < minZ)
            minZ = point[2];
        else if (point[2] > maxZ)
            maxZ = point[2];
    }
    c.bbox.length = maxX - minX;
    c.bbox.width = maxY - minY;
    c.bbox.height = maxZ - minZ;
    c.bbox.center = {(maxX + minX) / 2, (maxY + minY) / 2, (maxZ + minZ) / 2};
}