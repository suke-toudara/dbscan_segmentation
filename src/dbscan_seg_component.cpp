#include <dbscan_segmentation/dbscan_seg_component.hpp>

namespace dbscan_segmentation
{
DBScanNode::DBScanNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_filter_node", options) 
{
    SetFilterParam();
    
    std::string cloud_topic_name;
    std::string filter_cloud_topic_name;
    declare_parameter<std::string>("cloud_topic_name","/lidar_points");
    declare_parameter<std::string>("filter_cloud_topic_name","/filtered_cloud");
    get_parameter("cloud_topic_name", cloud_topic_name);
    get_parameter("filter_cloud_topic_name", filter_cloud_topic_name);

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_name, 10, std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1)
    );
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        filter_cloud_topic_name, 10
    );
}

void DBScanNode::SetFilterParam(){
    declare_parameter("voxel_size",0.1);
    declare_parameter("min_points",5);
    get_parameter("voxel_size", voxel_filter_.voxel_size);
    get_parameter("min_points", voxel_filter_.min_points);
}

void DBScanNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = cloud;  // 同じ型に変更

    
    DBSCAN ds(MINIMUM_POINTS, EPSILON, points);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = msg->header;
    point_cloud_pub_->publish(output_msg);
}
} // namespace dbscan_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dbscan_segmentation::DBScanNode)

