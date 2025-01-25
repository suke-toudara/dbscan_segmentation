#include <dbscan_segmentation/dbscan_seg_component.hpp>

namespace dbscan_segmentation
{
DBScanNode::DBScanNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_filter_node", options),
  tf_buffer_(get_clock()),
  listener_(tf_buffer_)

{
    //set_fliter_param
    declare_parameter("min_points",10);
    declare_parameter("epsilon",1.0);
    get_parameter("min_points",min_points_);
    get_parameter("epsilon", epsilon_);
    //set_topic_name
    std::string cloud_topic_name;
    std::string filter_cloud_topic_name;
    std::string clusters_topic_name;
    declare_parameter<std::string>("cloud_topic_name","/lidar_points");
    declare_parameter<std::string>("clustering_topic_name","/clustering_cloud");
    declare_parameter<std::string>("clusters_topic_name","clusters");
    get_parameter("cloud_topic_name", cloud_topic_name);
    get_parameter("clustering_topic_name", filter_cloud_topic_name);
    get_parameter("clusters_topic_name", clusters_topic_name);
    
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_name, 10, std::bind(&DBScanNode::cloud_callback, this, std::placeholders::_1)
    );
    clusters_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        filter_cloud_topic_name, 10
    );
    // clusters_pub_ = create_publisher<geometry_msgs::msg::PolygonArray>(
    //     clusters_topic_name, 10
    // );
}


void DBScanNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("base_link", msg->header.frame_id, msg->header.stamp,rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        return;
    }
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*msg, transformed_cloud, transform);
    auto transformed_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_cloud);
    std::vector<Point> points ;
    extractPointsFromCloud(transformed_cloud_ptr, points);
    std::vector<Point> downsampled_points;
    downsampling(points, downsampled_points);
    find_neighbors(downsampled_points, epsilon_,min_points_);
    find_clusters(downsampled_points);
    publishClusters(downsampled_points);
}

void DBScanNode::downsampling(std::vector<Point> &points,std::vector<Point> &downsampled_points){
    //downsampling
    std::unordered_map<std::string, Point> voxel_map;
    for (auto &p : points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        int x = static_cast<int>(p.x / 0.05);
        int y = static_cast<int>(p.y / 0.05);
        int z = static_cast<int>(p.z / 0.05);
        std::string key = std::to_string(x) + std::to_string(y) + std::to_string(z);
        if (voxel_map.find(key) == voxel_map.end())
        {
            voxel_map[key] = p;
        }
    }
    for (auto &pair : voxel_map)
    {
        downsampled_points.push_back(pair.second);
    }
}
void DBScanNode::extractPointsFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::vector<Point> &points)
{
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) || 
            *iter_x > 40.0 || *iter_x < -40.0 || 
            *iter_y > 40.0 || *iter_y < -40.0 ||
            *iter_z > 5.0 || *iter_z < 0.1)
        {
            continue;
        }
        Point point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.cluster_id = -1;
        point.core = false;
        points.push_back(point);
    }
}


void DBScanNode::find_clusters(std::vector<Point> &points)
{
    // -1: 未訪問, >0: クラスタID
    int cluster_id = 0;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                Point& p = points[i];
                if (p.core && p.cluster_id == -1) {
                    p.cluster_id = cluster_id;
                    expandCluster(p, cluster_id, points);
                    cluster_id++;
                }
            }
        }
    );
}

void DBScanNode::expandCluster(Point& p, int &cluster_id, std::vector<Point> &points)
{   
    std::queue<Point*> qu;
    qu.push(&p);
    while (!qu.empty()) {
        Point* current = qu.front();
        qu.pop();    
        for (Point &q : points) {
            if (q.cluster_id == -1 &&
                current->x >= q.x - epsilon_ && current->x <= q.x + epsilon_ &&
                current->y >= q.y - epsilon_ && current->y <= q.y + epsilon_ &&
                current->z >= q.z - epsilon_ && current->z <= q.z + epsilon_) 
            { 
                q.cluster_id = cluster_id;
                if (q.core){
                    qu.push(&q);
                }
            }
        }
    }        
}

void DBScanNode::publishClusters(const std::vector<Point> &points)
{
    visualization_msgs::msg::MarkerArray cluster_polygons;
    std::map<int, std::vector<Point>> clusters;
    for (const Point &p : points)
    {   
        if(p.cluster_id == -1) continue;
        clusters[p.cluster_id].push_back(p);
    }
    for (const auto &pair : clusters)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "clusters";
        marker.id = pair.first;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action =  visualization_msgs::msg::Marker::ADD;
        marker.color.a = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        double min_x ,max_x,min_y,max_y,min_z,max_z;
        for (const Point &p : pair.second)
        {
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y);
            max_y = std::max(max_y, p.y);
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }
        marker.pose.position.x = (min_x + max_x) / 2;
        marker.pose.position.y = (min_y + max_y) / 2;
        marker.pose.position.z = (min_z + max_z) / 2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = max_x - min_x;
        marker.scale.y = max_y - min_y;
        marker.scale.z = max_z - min_z;
        cluster_polygons.markers.push_back(marker);
    }
    clusters_pub_->publish(cluster_polygons);
}

} // namespace dbscan_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dbscan_segmentation::DBScanNode)
