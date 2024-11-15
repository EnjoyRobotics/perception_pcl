#include "pcl_ros/filters/frustum_culling.hpp"

pcl_ros::FrustumCulling::FrustumCulling(const rclcpp::NodeOptions & options)
: Filter("FrustumCullingNode", options)
{
  rcl_interfaces::msg::ParameterDescriptor vertical_fov_desc;
  vertical_fov_desc.name = "vertical_fov";
  vertical_fov_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  vertical_fov_desc.description =
    "Vertical field of view in radians";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 2 * M_PI;
    vertical_fov_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(vertical_fov_desc.name, rclcpp::ParameterValue(1.0), vertical_fov_desc);

  rcl_interfaces::msg::ParameterDescriptor horizontal_fov_desc;
  horizontal_fov_desc.name = "horizontal_fov";
  horizontal_fov_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  horizontal_fov_desc.description =
    "Horizontal field of view in radians";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 2 * M_PI;
    horizontal_fov_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(horizontal_fov_desc.name, rclcpp::ParameterValue(1.0), horizontal_fov_desc);

  rcl_interfaces::msg::ParameterDescriptor near_plane_distance_desc;
  near_plane_distance_desc.name = "near_plane_distance";
  near_plane_distance_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  near_plane_distance_desc.description =
    "Distance to the near plane";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 1000.0;
    near_plane_distance_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(
    near_plane_distance_desc.name, rclcpp::ParameterValue(
      0.1), near_plane_distance_desc);

  rcl_interfaces::msg::ParameterDescriptor far_plane_distance_desc;
  far_plane_distance_desc.name = "far_plane_distance";
  far_plane_distance_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  far_plane_distance_desc.description =
    "Distance to the far plane";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 1000.0;
    far_plane_distance_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(
    far_plane_distance_desc.name, rclcpp::ParameterValue(
      10.0), far_plane_distance_desc);

  const std::vector<std::string> param_names {
    vertical_fov_desc.name,
    horizontal_fov_desc.name,
    near_plane_distance_desc.name,
    far_plane_distance_desc.name,
  };

  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &FrustumCulling::config_callback, this,
      std::placeholders::_1));

  config_callback(get_parameters(param_names));

  // TODO(daisukes): lazy subscription after rclcpp#2060
  subscribe();
}

void
pcl_ros::FrustumCulling::filter(
  const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*(input), *(pcl_input));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*pcl_input, *pcl_input_xyz);

  impl_.setInputCloud(pcl_input_xyz);
  Eigen::Matrix4f camera_pose;

  // Convention: camera is looking along the z-axis
  Eigen::Matrix4f cam_transform;
  cam_transform << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  impl_.setCameraPose(cam_transform);

  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  impl_.filter(pcl_output);
  pcl::PCLPointCloud2 pcl_output2;
  pcl::toPCLPointCloud2(pcl_output, pcl_output2);
  pcl_conversions::moveFromPCL(pcl_output2, output);
}

//////////////////////////////////////////////////////////////////////////////////////////////

rcl_interfaces::msg::SetParametersResult
pcl_ros::FrustumCulling::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "vertical_fov") {
      impl_.setVerticalFOV(param.as_double() * 180.0 / M_PI);
    }
    if (param.get_name() == "horizontal_fov") {
      impl_.setHorizontalFOV(param.as_double() * 180.0 / M_PI);
    }
    if (param.get_name() == "near_plane_distance") {
      impl_.setNearPlaneDistance(param.as_double());
    }
    if (param.get_name() == "far_plane_distance") {
      impl_.setFarPlaneDistance(param.as_double());
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::FrustumCulling)
