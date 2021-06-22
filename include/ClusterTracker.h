//
// Created by jacob on 5/4/21.
//

#ifndef FETCHVRBACKEND_CLUSTERTRACKER_H
#define FETCHVRBACKEND_CLUSTERTRACKER_H
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <flann/flann.hpp>

using namespace pcl;
using namespace std;

// A pointcloud which also stores Mahalanobis Distance of each point
typedef struct MahalanobisPointCloud {
  vector<vector<float> > points;
  vector<float> distances;
} MahalanobisPointCloud;

typedef struct BoundingBox {
    vector<float> center;
    float length;   // x - dir
    float width;    // y - dir
    float height;   // z - dir
} BoundingBox;

typedef struct Cluster {
    MahalanobisPointCloud pointcloud;
    BoundingBox bbox;
    float score;
    bool isSupport = false, isAppearing;
} Cluster;

class ClusterTracker {
private:
    // stuff to add: Flann kdTree for storage
//    map<vector<float>, Cluster> clustersByCenter;
    vector<Cluster> orderedClusters;
    string additionTopic, deletionTopic;
    ros::Publisher additionPub, deletionPub;
    flann::Index<flann::L2_3D<float> > *index;
    float maxBboxDim; // used for querying purposes
    int n_overlap; // threshold for min number of overlapping clusters which satisfy the requirements
    void updateSupports(Cluster &c);  // queries a cluster, calls publish if support on all clusters intersected, including itself
    static void calculateBbox(Cluster &c);  // calculates the Bbox of a cluster
    void publishIfSupport(Cluster &c);   // determines if cluster is support, if so, publishes to appropriate topic
    float intersect(BoundingBox a, BoundingBox b); // returns the volume of the intersection of two bboxes
public:
    explicit ClusterTracker(ros::NodeHandle nh);
    void AddCluster(Cluster& c);
    vector<Cluster> queryClusterCenters(Cluster query_cluster);
    ~ClusterTracker();
};
#endif //FETCHVRBACKEND_CLUSTERTRACKER_H
