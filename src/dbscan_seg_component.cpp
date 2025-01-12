#include <dbscan_segmentation/dbscan_seg_component.hpp>

namespace dbscan_segmentation
{
DBScanNode::DBScanNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_filter_node", options),
  tf_buffer_(get_clock()),
  listener_(tf_buffer_)

{
    SetFilterParam();
    std::string cloud_topic_name;
    std::string filter_cloud_topic_name;
    declare_parameter<std::string>("cloud_topic_name","/lidar_points");
    declare_parameter<std::string>("clustering_topic_name","/clustering_cloud");
    get_parameter("cloud_topic_name", cloud_topic_name);
    get_parameter("clustering_topic_name", filter_cloud_topic_name);

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_name, 10, std::bind(&DBScanNode::cloud_callback, this, std::placeholders::_1)
    );
    point_cloud_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        filter_cloud_topic_name, 10
    );
}

void DBScanNode::SetFilterParam(){
    declare_parameter("min_points",3);
    declare_parameter("epsilon_",0.5);
    get_parameter("min_points",min_points_);
    get_parameter("epsilon", epsilon_);
}

void DBScanNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        return;
    }
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*msg, transformed_cloud, transform);
    auto transformed_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_cloud);
    std::vector<Point> points = extractPointsFromCloud(transformed_cloud_ptr);
    dbscan(points);
    publishClusters(points);
}

std::vector<Point> DBScanNode::extractPointsFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::vector<Point> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {
            continue;
        }else if (*iter_x < 20.0 && *iter_y < 20.0 && *iter_z < 20.0)
        {
            continue;
        }
        Point point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        points.push_back(point);
    }
    return points;
}

void DBScanNode::dbscan(std::vector<Point> &points)
{
    // -1: 未訪問, 0: ノイズ, >0: クラスタID
    int cluster_id = 1;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].cluster_id != -1)
            continue;

        std::vector<int> neighbors = find_neighbors(points, i);
        if (neighbors.size() < min_points_)
        {
            // ノイズとして処理
            points[i].cluster_id = 0; 
            continue;
        }
        expandCluster(points,cluster_id,neighbors);
        cluster_id++;
    }
}

std::vector<int> DBScanNode::find_neighbors(const std::vector<Point> &points, int index)
{
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (static_cast<std::size_t>(index) != i && distance(points[index], points[i]) <= epsilon_)
        {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

void DBScanNode::expandCluster(std::vector<Point> &points, int &cluster_id, std::vector<int> &neighbors)
{
    for (auto& neighbor : neighbors)
    { 
        points[neighbor].cluster_id = cluster_id;

        for (size_t i = 0; i < points.size(); ++i)
        {
            bool found = std::find(neighbors.begin(), neighbors.end(),i) != neighbors.end();
            if (found) {
                points[i].cluster_id = cluster_id;
                continue;
            }
            else if (points[i].cluster_id != -1 ) {
                continue;
            }
            if (i != static_cast<std::size_t>(neighbor) && distance(points[neighbor], points[i]) <= epsilon_)
            {
                //ノイズ判定なくていい？

                points[i].cluster_id = cluster_id;
            }
        }
    }
}

double DBScanNode::distance(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

void DBScanNode::publishClusters(const std::vector<Point> &points)
{
    // クラスタごとにMarkerを生成する
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;    
    for (const auto &point : points)
    {
        if (point.cluster_id == -1 || point.cluster_id == 0) continue;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "sphepe";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
        marker.color.r = 0.5;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.id = id++;
        markers.markers.push_back(marker);
    }
    point_cloud_pub_->publish(markers);
}
    
} // namespace dbscan_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dbscan_segmentation::DBScanNode)

