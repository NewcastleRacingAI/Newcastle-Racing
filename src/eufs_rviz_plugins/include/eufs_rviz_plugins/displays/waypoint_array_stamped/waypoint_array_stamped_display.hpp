#ifndef EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_WAYPOINT_ARRAY_STAMPED_WAYPOINT_ARRAY_STAMPED_DISPLAY_HPP_  // NOLINT
#define EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_WAYPOINT_ARRAY_STAMPED_WAYPOINT_ARRAY_STAMPED_DISPLAY_HPP_  // NOLINT

#include <memory>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"

namespace eufs_rviz_plugins {
namespace displays {

class WaypointArrayStampedDisplay
    : public rviz_common::RosTopicDisplay<eufs_msgs::msg::WaypointArrayStamped> {
  Q_OBJECT

 public:
  WaypointArrayStampedDisplay();

  void onInitialize() override;

  void load(const rviz_common::Config &config) override;

  void update(float wall_dt, float ros_dt) override;

  void reset() override;

 private:
  void initMarkers();

  void setMarkerArray(const eufs_msgs::msg::WaypointArrayStamped::ConstSharedPtr &msg);

  void processMessage(eufs_msgs::msg::WaypointArrayStamped::ConstSharedPtr msg) override;

  visualization_msgs::msg::Marker CreateTrajectoryMarker(
      const eufs_msgs::msg::WaypointArrayStamped::ConstSharedPtr &msg);

  int id_;

  rviz_common::properties::ColorProperty *color_property_;

  std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> marker_common_;

  visualization_msgs::msg::Marker delete_all_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
};

}  // namespace displays
}  // namespace eufs_rviz_plugins

#endif  // NOLINT(build/header_guard)
