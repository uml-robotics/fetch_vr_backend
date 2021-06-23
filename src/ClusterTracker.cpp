//
// Created by jacob on 5/5/21.
// Functions for the ClusterTracker class
//

#include "ClusterTracker.h"
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <flann/flann.hpp>
#include <octomap_msgs/Octomap.h>
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
    //this->clustersByCenter[c.bbox.center] = c;
    // Trying out a vector of ordered clusters
    this->orderedClusters.push_back(c);

    // Call updateSupports method on cluster c
    this->updateSupports(c);
}

void ClusterTracker::updateSupports(Cluster &c) {
    // Call publishIfSupport on c and its neighbors
    publishIfSupport(c);
    for (Cluster cluster : this->queryClusterCenters(c)) {
        publishIfSupport(cluster);
    }
}

void ClusterTracker::publishIfSupport(Cluster &c) {
    // if cluster has already been marked as a support, do not query
    if (c.isSupport)
        return;

    // Query the kdTree of cluster centers
    vector<Cluster> nearby_clusters = this->queryClusterCenters(c);

    // do not publish if there aren't enough clusters
    if (nearby_clusters.size() < this->n_overlap) {
        return;
    }

    // Count up the number of clusters for which v_intersect / c_volume > 1 - c_score
    float c_volume = c.bbox.length * c.bbox.width * c.bbox.height;
    float v_intersect, c_score = c.score;
    int cluster_count;
    for (const Cluster& nearby_cluster : nearby_clusters) {
        v_intersect = this->intersect(nearby_cluster.bbox, c.bbox);
        if (v_intersect / c_volume > 1 - c_score) {
            cluster_count ++;
        }
        if (cluster_count == n_overlap) {
            // publish the cluster!
            c.isSupport = true;
            if (c.isAppearing) {
                ROS_INFO("Publishing Addition cluster of %zu points", c.pointcloud.points.size());
                // convert the cluster to a sensor_msgs::PointCloud2

                this->additionPub.publish(c);
                return;
            }
            ROS_INFO("Publishing Deletion cluster of %zu voxels", c.pointcloud.points.size());
            this->deletionPub.publish(c);
            // query the octomap to get all deletion voxels, store these in another octomap

            return;
        }
    }
}

vector<Cluster> ClusterTracker::queryClusterCenters(Cluster queryCluster) {
    float searchRadius = this->maxBboxDim + max(queryCluster.bbox.length, queryCluster.bbox.width, queryCluster.bbox.height);
    flann::Matrix<float> query(new float[3], 1, 3);
    query[0][0] = queryCluster.bbox.center[0];
    query[0][1] = queryCluster.bbox.center[1];
    query[0][2] = queryCluster.bbox.center[2];
    vector<vector<int> > indices;
    vector<vector<float> > distances;
    this->index->radiusSearch(query, indices, distances, searchRadius, flann::SearchParams());

    // for all found cluster centers, look up the corresponding cluster object
    // call publish if support on each of these clusters
    vector<Cluster> found_clusters;
    for (int center_index : indices[0]) {
        // Look up the corresponding cluster object
        found_clusters.push_back(this->orderedClusters[center_index]);
    }
    return found_clusters;
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

float ClusterTracker::intersect(BoundingBox a, BoundingBox b) {
    // find min and max coords in x,y,z directions of both boxes
    float a_x_min, a_x_max, b_x_min, b_x_max;
    float a_y_min, a_y_max, b_y_min, b_y_max;
    float a_z_min, a_z_max, b_z_min, b_z_max;
    float x_dim, y_dim, z_dim;
    a_x_min = a.center[0] - a.length / 2;
    a_x_max = a.center[0] + a.length / 2;
    b_x_min = b.center[0] - b.length / 2;
    b_x_max = b.center[0] + b.length / 2;
    a_y_min = a.center[1] - a.width / 2;
    a_y_max = a.center[1] + a.width / 2;
    b_y_min = b.center[1] - b.width / 2;
    b_y_max = b.center[1] + b.width / 2;
    a_z_min = a.center[2] - a.height / 2;
    a_z_max = a.center[2] + a.height / 2;
    b_z_min = b.center[2] - b.height / 2;
    b_z_max = b.center[2] + b.height / 2;

    // Check if the boxes do not intersect
    if (a_x_min > b_x_max || b_x_min > a_x_max || a_y_min > b_y_max ||
        b_y_min > a_y_max || a_z_min > b_z_max || b_z_min > a_z_max) {
        // no intersection
        return 0.0;
    }

    // calculate length of intersection volume in each axial direction
    x_dim = min(a_x_max, b_x_max) - max(a_x_min, b_x_min);
    y_dim = min(a_y_max, b_y_max) - max(a_y_min, b_y_min);
    z_dim = min(a_z_max, b_z_max) - max(a_z_min, b_z_min);

    // return the volume of the intersection
    return x_dim * y_dim * z_dim;
}

void ClusterTracker::setOctree(OcTree &input_tree) {
    this->tree = &input_tree;
}