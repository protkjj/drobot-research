// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "drobot_costmap_2_5d/elevation_height_layer.hpp"

#include <string>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "drobot_costmap_2_5d/elevation_layer.hpp"

namespace drobot_costmap_2_5d
{

void ElevationHeightLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"ElevationHeightLayer: 노드를 잠글 수 없다"};
  }
  logger_ = node->get_logger();
  const std::string p = name_ + ".";
  nav2_util::declare_parameter_if_not_declared(node, p + "enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "source_layer", rclcpp::ParameterValue(source_name_));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "height_unit", rclcpp::ParameterValue(unit_m_));
  node->get_parameter(p + "enabled", enabled_);
  node->get_parameter(p + "source_layer", source_name_);
  node->get_parameter(p + "height_unit", unit_m_);

  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
  current_ = true;
  RCLCPP_INFO(
    logger_, "ElevationHeightLayer '%s': '%s' 의 높이를 %.3f m 단위로 담는다",
    name_.c_str(), source_name_.c_str(), unit_m_);
}

void ElevationHeightLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  // 갱신 범위는 늘리지 않는다. ElevationLayer 가 자기 범위를 이미 넓힌다.
}

void ElevationHeightLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &, int, int, int, int)
{
  if (!enabled_) {return;}
  ElevationLayer * src = findSource();
  if (src == nullptr) {return;}
  // 창이 아니라 격자 전체를 옮긴다. ElevationLayer 는 지우는 경우(reset) 도 있어서
  // 창만 옮기면 바깥이 낡은 값으로 남는다. 30×20 m / 0.05 m 에서 24 만 칸 — 복사뿐이라 가볍다.
  if (!src->copyHeightsTo(*this, unit_m_) && !warned_) {
    RCLCPP_WARN(
      logger_, "'%s' 와 격자 크기가 다르다 — 높이를 담지 못한다", source_name_.c_str());
    warned_ = true;
  }
}

ElevationLayer * ElevationHeightLayer::findSource()
{
  if (source_ != nullptr) {return source_;}
  for (const auto & layer : *layered_costmap_->getPlugins()) {
    if (layer->getName() != source_name_) {continue;}
    // 같은 라이브러리 안의 형이라 dynamic_cast 가 안전하다
    source_ = dynamic_cast<ElevationLayer *>(layer.get());
    break;
  }
  if (source_ == nullptr && !warned_) {
    RCLCPP_WARN(
      logger_, "'%s' ElevationLayer 를 찾지 못했다 — 높이 격자가 비어 있다 (전부 255)",
      source_name_.c_str());
    warned_ = true;
  }
  return source_;
}

}  // namespace drobot_costmap_2_5d

PLUGINLIB_EXPORT_CLASS(
  drobot_costmap_2_5d::ElevationHeightLayer, nav2_costmap_2d::Layer)
