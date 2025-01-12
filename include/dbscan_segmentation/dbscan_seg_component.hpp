#ifndef DBSCAN_H
#define DBSCAN_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

struct Point {
    float x, y, z;
    int cluster_id = -1; // 初期値は未分類
};

namespace dbscan_segmentation
{
class DBScanNode : public rclcpp::Node {
public:    
    explicit DBScanNode(const rclcpp::NodeOptions & options);
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    
    void SetFilterParam();
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::vector<Point> extractPointsFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    void dbscan(std::vector<Point> &points);
    std::vector<int> find_neighbors(const std::vector<Point> &points, int index);
    void expandCluster(std::vector<Point>& points, int& cluster_id, std::vector<int>& neighbors);
    double distance(const Point &p1, const Point &p2);
    void publishClusters(const std::vector<Point> &points);

    // int getTotalPointSize() {return m_pointSize;}
    // int getMinimumClusterSize() {return m_minPoints;}
    // int getEpsilonSize() {return m_epsilon;}
    
public:
    vector<Point> m_points;
    
private:    
    unsigned int pointSize_;
    unsigned int min_points_;
    float epsilon_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
        
};
}

#endif // DBSCAN_H