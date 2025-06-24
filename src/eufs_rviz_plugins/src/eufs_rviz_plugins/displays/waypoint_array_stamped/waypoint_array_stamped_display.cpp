#include "eufs_rviz_plugins/displays/waypoint_array_stamped/waypoint_array_stamped_display.hpp"  // NOLINT

#include <Eigen/Eigen>

namespace eufs_rviz_plugins {
namespace displays {

WaypointArrayStampedDisplay::WaypointArrayStampedDisplay()
    : rviz_common::RosTopicDisplay<eufs_msgs::msg::WaypointArrayStamped>(),
      id_(0),
      marker_common_(
          std::make_unique<rviz_default_plugins::displays::MarkerCommon>(
              this)) {
  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(100, 204, 153), "Color of waypoint array", this);
}

void WaypointArrayStampedDisplay::onInitialize() {
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);

  topic_property_->setValue("/trajectory");
  topic_property_->setDescription(
      "eufs_msgs::msg::WaypointArrayStamped topic to subscribe to.");

  initMarkers();
}

void WaypointArrayStampedDisplay::load(const rviz_common::Config &config) {
  Display::load(config);
  marker_common_->load(config);
}

void WaypointArrayStampedDisplay::processMessage(
    eufs_msgs::msg::WaypointArrayStamped::ConstSharedPtr msg) {
  delete_all_marker_.header = msg->header;
  delete_all_marker_.id = id_;
  marker_array_.markers.push_back(delete_all_marker_);

  marker_common_->addMessage(
      std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

  marker_array_.markers.clear();

  setMarkerArray(msg);
  marker_common_->addMessage(
      std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

  marker_array_.markers.clear();
}

void WaypointArrayStampedDisplay::update(float wall_dt, float ros_dt) {
  marker_common_->update(wall_dt, ros_dt);
}

void WaypointArrayStampedDisplay::reset() {
  RosTopicDisplay::reset();
  marker_common_->clearMarkers();
}

void WaypointArrayStampedDisplay::initMarkers() {
  delete_all_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
}

visualization_msgs::msg::Marker
WaypointArrayStampedDisplay::CreateTrajectoryMarker(
    const eufs_msgs::msg::WaypointArrayStamped::ConstSharedPtr &msg) {
  visualization_msgs::msg::Marker trajectory_marker;
  trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
  trajectory_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  trajectory_marker.pose.orientation.x = 0.0;
  trajectory_marker.pose.orientation.y = 0.0;
  trajectory_marker.pose.orientation.z = 0.0;
  trajectory_marker.pose.orientation.w = 1.0;
  trajectory_marker.scale.x = 0.3;
  trajectory_marker.scale.y = 0.3;
  trajectory_marker.scale.z = 0.3;

  QColor color = color_property_->getColor();
  trajectory_marker.color.r = static_cast<float>(color.red()) / 255.0f;
  trajectory_marker.color.g = static_cast<float>(color.green()) / 255.0f;
  trajectory_marker.color.b = static_cast<float>(color.blue()) / 255.0f;
  trajectory_marker.color.a = 1.0;

  trajectory_marker.ns = "waypoint";
  trajectory_marker.header = msg->header;

  return trajectory_marker;
}

void WaypointArrayStampedDisplay::setMarkerArray(
    const eufs_msgs::msg::WaypointArrayStamped::ConstSharedPtr &msg) {
  auto trajectory_marker = CreateTrajectoryMarker(msg);
  for (auto waypoint : msg->waypoints) {
    geometry_msgs::msg::Point p;
    p.x = waypoint.position.x;
    p.y = waypoint.position.y;

    trajectory_marker.points.push_back(p);
  }
  marker_array_.markers.push_back(trajectory_marker);
}

}  // namespace displays
}  // namespace eufs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(eufs_rviz_plugins::displays::WaypointArrayStampedDisplay,
                       rviz_common::Display)
