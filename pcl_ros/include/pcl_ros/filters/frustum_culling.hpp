#ifndef PCL_ROS__FILTERS__FRUSTUM_CULLING_HPP_
#define PCL_ROS__FILTERS__FRUSTUM_CULLING_HPP_

// PCL includes
#include <pcl/filters/frustum_culling.h>
#include <vector>
#include <pcl/conversions.h>
#include "pcl_ros/filters/filter.hpp"


namespace pcl_ros
{
// \brief @b FrustumCulling is a filter that allows the user to filter all the data inside of a given frustum.

class FrustumCulling : public Filter
{
protected:
  /** \brief Call the actual filter.
    * \param input the input point cloud dataset
    * \param indices the input set of indices to use from \a input
    * \param output the resultant filtered dataset
    */
  inline void
  filter(
    const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
    PointCloud2 & output) override;

  /** \brief Parameter callback
    * \param params parameter values to set
    */
  rcl_interfaces::msg::SetParametersResult
  config_callback(const std::vector<rclcpp::Parameter> & params);

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

private:
  /** \brief The PCL filter implementation used. */
  pcl::FrustumCulling<pcl::PointXYZ> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FrustumCulling(const rclcpp::NodeOptions & options);
};
}  // namespace pcl_ros

#endif  // PCL_ROS__FILTERS__FRUSTUM_CULLING_HPP_
