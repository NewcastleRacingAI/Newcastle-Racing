#include "eufs_rviz_plugins/displays/cone_array_with_covariance/cone_array_with_covariance_display.hpp"  // NOLINT

#include <Eigen/Eigen>

namespace eufs_rviz_plugins {
namespace displays {

ConeArrayWithCovarianceDisplay::ConeArrayWithCovarianceDisplay()
    : rviz_common::RosTopicDisplay<eufs_msgs::msg::ConeArrayWithCovariance>(),
      id_(0),
      marker_common_(
          std::make_unique<rviz_default_plugins::displays::MarkerCommon>(
              this)) {
  color_option_property_ = new rviz_common::properties::EnumProperty(
      "Color Display", "Cone", "Cone colour to use", this,
      SLOT(updateColorOption()));
  color_option_property_->addOption("Cone", ConeColorOption::CONE);
  color_option_property_->addOption("Flat", ConeColorOption::FLAT);

  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(200, 200, 200), "Color of cones to display", this);
  color_property_->hide();
}

void ConeArrayWithCovarianceDisplay::updateColorOption() {
  ConeColorOption color_option =
      static_cast<ConeColorOption>(color_option_property_->getOptionInt());
  cone_color_option_ = color_option;
  switch (color_option) {
    case CONE:
      color_property_->hide();
      break;
    case FLAT:
      color_property_->show();
      break;
  }
}

void ConeArrayWithCovarianceDisplay::onInitialize() {
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);

  topic_property_->setValue("/cones");
  topic_property_->setDescription(
      "eufs_msgs::msg::ConeArrayWithCovariance topic to subscribe to.");

  initMarkers();
}

void ConeArrayWithCovarianceDisplay::load(const rviz_common::Config &config) {
  Display::load(config);
  marker_common_->load(config);
}

void ConeArrayWithCovarianceDisplay::processMessage(
    eufs_msgs::msg::ConeArrayWithCovariance::ConstSharedPtr msg) {
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

void ConeArrayWithCovarianceDisplay::update(float wall_dt, float ros_dt) {
  marker_common_->update(wall_dt, ros_dt);
}

void ConeArrayWithCovarianceDisplay::reset() {
  RosTopicDisplay::reset();
  marker_common_->clearMarkers();
}

void ConeArrayWithCovarianceDisplay::initMarkers() {
  delete_all_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

  blue_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  blue_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  blue_cone_marker_.pose.orientation.x = 0.0;
  blue_cone_marker_.pose.orientation.y = 0.0;
  blue_cone_marker_.pose.orientation.z = 0.0;
  blue_cone_marker_.pose.orientation.w = 1.0;
  blue_cone_marker_.scale.x = 1.0;
  blue_cone_marker_.scale.y = 1.0;
  blue_cone_marker_.scale.z = 1.0;
  blue_cone_marker_.mesh_resource =
      "package://eufs_rviz_plugins/meshes/cone.dae";
  blue_cone_marker_.color.r = 0.0;
  blue_cone_marker_.color.g = 0.0;
  blue_cone_marker_.color.b = 1.0;
  blue_cone_marker_.color.a = 1.0;
  blue_cone_marker_.ns = "cone";

  yellow_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  yellow_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  yellow_cone_marker_.pose.orientation.x = 0.0;
  yellow_cone_marker_.pose.orientation.y = 0.0;
  yellow_cone_marker_.pose.orientation.z = 0.0;
  yellow_cone_marker_.pose.orientation.w = 1.0;
  yellow_cone_marker_.scale.x = 1.0;
  yellow_cone_marker_.scale.y = 1.0;
  yellow_cone_marker_.scale.z = 1.0;
  yellow_cone_marker_.mesh_resource =
      "package://eufs_rviz_plugins/meshes/cone.dae";
  yellow_cone_marker_.color.r = 1.0;
  yellow_cone_marker_.color.g = 1.0;
  yellow_cone_marker_.color.b = 0.0;
  yellow_cone_marker_.color.a = 1.0;
  yellow_cone_marker_.ns = "cone";

  orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  orange_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  orange_cone_marker_.pose.orientation.x = 0.0;
  orange_cone_marker_.pose.orientation.y = 0.0;
  orange_cone_marker_.pose.orientation.z = 0.0;
  orange_cone_marker_.pose.orientation.w = 1.0;
  orange_cone_marker_.scale.x = 1.0;
  orange_cone_marker_.scale.y = 1.0;
  orange_cone_marker_.scale.z = 1.0;
  orange_cone_marker_.mesh_resource =
      "package://eufs_rviz_plugins/meshes/cone.dae";
  orange_cone_marker_.color.r = 1.0;
  orange_cone_marker_.color.g = 0.549;
  orange_cone_marker_.color.b = 0.0;
  orange_cone_marker_.color.a = 1.0;
  orange_cone_marker_.ns = "cone";

  big_orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  big_orange_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  big_orange_cone_marker_.pose.orientation.x = 0.0;
  big_orange_cone_marker_.pose.orientation.y = 0.0;
  big_orange_cone_marker_.pose.orientation.z = 0.0;
  big_orange_cone_marker_.pose.orientation.w = 1.0;
  big_orange_cone_marker_.scale.x = 1.0;
  big_orange_cone_marker_.scale.y = 1.0;
  big_orange_cone_marker_.scale.z = 1.0;
  big_orange_cone_marker_.mesh_resource =
      "package://eufs_rviz_plugins/meshes/big_cone.dae";
  big_orange_cone_marker_.color.r = 1.0;
  big_orange_cone_marker_.color.g = 0.271;
  big_orange_cone_marker_.color.b = 0.0;
  big_orange_cone_marker_.color.a = 1.0;
  big_orange_cone_marker_.ns = "cone";

  unknown_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  unknown_cone_marker_.pose.orientation.x = 0.0;
  unknown_cone_marker_.pose.orientation.y = 0.0;
  unknown_cone_marker_.pose.orientation.z = 0.0;
  unknown_cone_marker_.pose.orientation.w = 1.0;
  unknown_cone_marker_.scale.x = 1.0;
  unknown_cone_marker_.scale.y = 1.0;
  unknown_cone_marker_.scale.z = 1.0;
  unknown_cone_marker_.mesh_resource =
      "package://eufs_rviz_plugins/meshes/cone.dae";
  unknown_cone_marker_.color.r = 0.0;
  unknown_cone_marker_.color.g = 1.0;
  unknown_cone_marker_.color.b = 0.0;
  unknown_cone_marker_.color.a = 1.0;
  unknown_cone_marker_.ns = "cone";

  covariance_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  covariance_marker_.pose.orientation.x = 0.0;
  covariance_marker_.pose.orientation.y = 0.0;
  covariance_marker_.pose.orientation.z = 0.0;
  covariance_marker_.pose.orientation.w = 1.0;
  covariance_marker_.scale.x = 1.0;
  covariance_marker_.scale.y = 1.0;
  covariance_marker_.scale.z = 1.0;
  covariance_marker_.color.r = 1.0;
  covariance_marker_.color.g = 0.271;
  covariance_marker_.color.b = 0.271;
  covariance_marker_.color.a = 0.7;
  covariance_marker_.ns = "covariance";
}

void ConeArrayWithCovarianceDisplay::setConeMarker(
    const eufs_msgs::msg::ConeWithCovariance &cone,
    const std_msgs::msg::Header &header, const int &id,
    visualization_msgs::msg::Marker *marker) {
  marker->id = id;
  marker->header = header;
  marker->pose.position.x = cone.point.x;
  marker->pose.position.y = cone.point.y;
  marker->pose.position.z = cone.point.z;
}

void ConeArrayWithCovarianceDisplay::setCovarianceMarker(
    const eufs_msgs::msg::ConeWithCovariance &cone,
    const std_msgs::msg::Header &header, const int &id) {
  // https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
  covariance_marker_.id = id;
  covariance_marker_.header = header;
  covariance_marker_.pose.position.x = cone.point.x;
  covariance_marker_.pose.position.y = cone.point.y;
  covariance_marker_.pose.position.z = cone.point.z;

  // Convert the covariance message to a matrix
  Eigen::Matrix2d cov_matrix;
  cov_matrix << static_cast<float>(cone.covariance[0]),
    static_cast<float>(cone.covariance[1]),
    static_cast<float>(cone.covariance[2]),
    static_cast<float>(cone.covariance[3]);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov_matrix);
  const Eigen::Vector2d& eigValues(eig.eigenvalues());
  const Eigen::Matrix2d& eigVectors(eig.eigenvectors());
  double x_scale = 2 * std::sqrt(5.991 * eigValues[0]);
  double y_scale = 2 * std::sqrt(5.991 * eigValues[1]);
  covariance_marker_.scale.x = x_scale;
  covariance_marker_.scale.y = y_scale;
  covariance_marker_.scale.z = 0.01;
  // Angle between x-axis and first eigenvector
  double angle = std::atan2(eigVectors(1, 0), eigVectors(0, 0));
  // Rotate ellipse such that x-axis is aligned with first eignevector
  // See: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
  covariance_marker_.pose.orientation.x = 0;
  covariance_marker_.pose.orientation.y = 0;
  covariance_marker_.pose.orientation.z = std::sin(angle * 0.5);
  covariance_marker_.pose.orientation.w = std::cos(angle * 0.5);
}

visualization_msgs::msg::Marker
ConeArrayWithCovarianceDisplay::getColoredMarker(
    visualization_msgs::msg::Marker cone_marker) {
  visualization_msgs::msg::Marker marker = cone_marker;
  switch (cone_color_option_) {
    case FLAT: {
      QColor color = color_property_->getColor();
      marker.color.r = static_cast<float>(color.red()) / 255.0f;
      marker.color.g = static_cast<float>(color.green()) / 255.0f;
      marker.color.b = static_cast<float>(color.blue()) / 255.0f;
    }
    default:
      break;
  }
  return marker;
}

void ConeArrayWithCovarianceDisplay::setMarkerArray(
    const eufs_msgs::msg::ConeArrayWithCovariance::ConstSharedPtr &msg) {
  for (const auto &cone : msg->blue_cones) {
    setConeMarker(cone, msg->header, id_, &blue_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(blue_cone_marker_);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->yellow_cones) {
    setConeMarker(cone, msg->header, id_, &yellow_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(yellow_cone_marker_);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->orange_cones) {
    setConeMarker(cone, msg->header, id_, &orange_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(orange_cone_marker_);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->unknown_color_cones) {
    setConeMarker(cone, msg->header, id_, &unknown_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(unknown_cone_marker_);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->big_orange_cones) {
    setConeMarker(cone, msg->header, id_, &big_orange_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(big_orange_cone_marker_);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
}

}  // namespace displays
}  // namespace eufs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    eufs_rviz_plugins::displays::ConeArrayWithCovarianceDisplay,
    rviz_common::Display)
