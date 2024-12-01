#include <dbscan_segmentation/dbscan_seg_component.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<dbscan_segmentation::DBScanNode>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}