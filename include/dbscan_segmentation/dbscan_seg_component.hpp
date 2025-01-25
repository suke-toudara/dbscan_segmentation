#ifndef DBSCAN_H
#define DBSCAN_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
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
#include <tbb/tbb.h>
#include <dbscan_segmentation/kd_tree.hpp>

using namespace std;

namespace dbscan_segmentation
{
class DBScanNode : public rclcpp::Node {
public:    
    explicit DBScanNode(const rclcpp::NodeOptions & options);
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;
    // rclcpp::Publisher<geometry_msgs::msg::PolygonArray>::SharedPtr clusters_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void extractPointsFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::vector<Point> &points);
    void downsampling(std::vector<Point> &points,std::vector<Point> &downsampled_points);
    void find_clusters(std::vector<Point> &points);
    void expandCluster(Point& p, int &cluster_id, std::vector<Point> &points);
    void publishClusters(const std::vector<Point> &points);

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