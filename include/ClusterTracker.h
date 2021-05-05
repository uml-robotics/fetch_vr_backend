//
// Created by jacob on 5/4/21.
//

#ifndef FETCHVRBACKEND_CLUSTERTRACKER_H
#define FETCHVRBACKEND_CLUSTERTRACKER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace pcl;
using namespace std;

// A pointcloud with points which store the mahalanobis distance
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
    PointCloud<PointXYZRGB> pointcloud;
    BoundingBox bbox;
    float score;
    bool isSupport = false;
} Cluster;

class ClusterTracker {
private:
    vector<Cluster> clusters;
public:
    ClusterTracker();
    void AddCluster(PointCloud<PointXYZRGB>);
    ~ClusterTracker();

};

ClusterTracker::ClusterTracker() {

}

void ClusterTracker::AddCluster(PointCloud<PointXYZRGB> pcd) {
    Cluster newCluster;
    newCluster.pointcloud = pcd;
    clusters.push_back(newCluster);
}

#endif //FETCHVRBACKEND_CLUSTERTRACKER_H
