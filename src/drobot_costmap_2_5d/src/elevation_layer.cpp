// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "drobot_costmap_2_5d/elevation_layer.hpp"

// cpplint 규칙상 C 헤더(.h)가 C++ 헤더(.hpp)보다 앞에 와야 한다.
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace drobot_costmap_2_5d
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace
{
template<typename T>
T getParam(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & name, const T & fallback)
{
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(fallback));
  T out;
  node->get_parameter(name, out);
  return out;
}
}  // namespace


// ---------------------------------------------------------------------------
// 초기화
// ---------------------------------------------------------------------------
void ElevationLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"ElevationLayer: 노드를 잠글 수 없다"};
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = layered_costmap_->getGlobalFrameID();

  const std::string p = name_ + ".";

  enabled_param_ = getParam(node, p + "enabled", true);
  enabled_ = enabled_param_;

  // 높이 임계값
  rover_traversable_max_ =
    getParam(node, p + "height_thresholds.rover_traversable_max", rover_traversable_max_);
  fly_over_max_ = getParam(node, p + "height_thresholds.fly_over_max", fly_over_max_);
  ceiling_height_ = getParam(node, p + "ceiling_height", ceiling_height_);

  // traversability
  max_slope_deg_ = getParam(node, p + "traversability.max_slope_angle", max_slope_deg_);
  max_roughness_ = getParam(node, p + "traversability.max_roughness", max_roughness_);
  max_step_height_ = getParam(node, p + "traversability.max_step_height", max_step_height_);

  // cost 값 (int로 읽어 캐스팅)
  cost_free_ = static_cast<unsigned char>(getParam(node, p + "cost_values.free", 0));
  cost_rover_ =
    static_cast<unsigned char>(getParam(node, p + "cost_values.rover_traversable", 100));
  cost_flyover_ = static_cast<unsigned char>(getParam(node, p + "cost_values.fly_over", 200));
  cost_impassable_ =
    static_cast<unsigned char>(getParam(node, p + "cost_values.impassable", 254));

  // 로봇 형상
  robot_flight_height_ = getParam(node, p + "robot_flight_height", robot_flight_height_);
  ceiling_clearance_ = getParam(node, p + "min_ceiling_clearance", ceiling_clearance_);

  // 센서
  depth_topic_ = getParam(node, p + "depth_topic", depth_topic_);
  scan_topic_ = getParam(node, p + "scan_topic", scan_topic_);
  use_pointcloud_ = getParam(node, p + "use_pointcloud", use_pointcloud_);
  use_laserscan_ = getParam(node, p + "use_laserscan", use_laserscan_);
  min_obstacle_height_ = getParam(node, p + "min_obstacle_height", min_obstacle_height_);
  max_obstacle_height_ = getParam(node, p + "max_obstacle_height", max_obstacle_height_);
  max_sensor_range_ = getParam(node, p + "max_sensor_range", max_sensor_range_);

  publish_elevation_ = getParam(node, p + "publish_elevation_grid", publish_elevation_);
  clear_on_reset_ = getParam(node, p + "clear_on_reset", clear_on_reset_);

  matchSize();
  current_ = true;

  // 구독
  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  if (use_pointcloud_) {
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      depth_topic_, sensor_qos,
      std::bind(&ElevationLayer::pointCloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "ElevationLayer: PointCloud2 구독 '%s'", depth_topic_.c_str());
  }
  if (use_laserscan_) {
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, sensor_qos,
      std::bind(&ElevationLayer::laserScanCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "ElevationLayer: LaserScan 구독 '%s'", scan_topic_.c_str());
  }
  if (publish_elevation_) {
    elevation_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      name_ + "/elevation_grid", rclcpp::QoS(1).transient_local());
  }

  RCLCPP_INFO(
    logger_,
    "ElevationLayer '%s' 초기화: rover<=%.2fm, flyover<=%.2fm, 천장=%.2fm | "
    "경사<=%.1f도, 거칠기<=%.3fm, 단차<=%.3fm",
    name_.c_str(), rover_traversable_max_, fly_over_max_, ceiling_height_,
    max_slope_deg_, max_roughness_, max_step_height_);

  // 설정 검증 — 조용히 잘못 동작하는 것보다 경고가 낫다
  if (rover_traversable_max_ >= fly_over_max_) {
    RCLCPP_WARN(
      logger_,
      "rover_traversable_max(%.2f) >= fly_over_max(%.2f) 이면 fly-over 등급이 "
      "생기지 않는다. 비행이 필요한 장애물을 구분하지 못한다.",
      rover_traversable_max_, fly_over_max_);
  }
  const double max_flyable = ceiling_height_ - robot_flight_height_ - ceiling_clearance_;
  if (fly_over_max_ > max_flyable) {
    RCLCPP_WARN(
      logger_,
      "fly_over_max(%.2fm)가 천장 제약상 실제 통과 가능 높이(%.2fm)를 넘는다. "
      "%.2fm~%.2fm 구간은 fly-over로 분류되지만 실제로는 넘을 수 없다.",
      fly_over_max_, max_flyable, max_flyable, fly_over_max_);
  }
}


void ElevationLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(), master->getOriginX(), master->getOriginY());

  std::lock_guard<std::mutex> lock(data_mutex_);
  size_x_ = master->getSizeInCellsX();
  size_y_ = master->getSizeInCellsY();
  cells_.assign(static_cast<size_t>(size_x_) * size_y_, ElevationCell{});
}


void ElevationLayer::onFootprintChanged()
{
  // footprint 변화는 이 레이어 동작에 영향을 주지 않는다.
  // (인플레이션은 별도 레이어가 담당)
}


// ---------------------------------------------------------------------------
// TF
// ---------------------------------------------------------------------------
bool ElevationLayer::lookupToGlobal(
  const std::string & source_frame, const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & out) const
{
  if (source_frame.empty() || source_frame == global_frame_) {
    // 이미 전역 프레임이면 항등 변환
    out = geometry_msgs::msg::TransformStamped();
    out.transform.rotation.w = 1.0;
    return true;
  }
  try {
    // 센서 타임스탬프 기준으로 조회하되, 짧게 기다린다.
    // costmap 갱신 주기를 막지 않도록 대기 시간을 크게 두지 않는다.
    out = tf_->lookupTransform(
      global_frame_, source_frame, stamp, tf2::durationFromSec(0.1));
    return true;
  } catch (const tf2::TransformException & ex) {
    // 시작 직후에는 TF 가 아직 안 올라와 실패가 잦다. 로그를 조인다.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "TF 변환 실패 %s -> %s: %s",
      source_frame.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }
}


// ---------------------------------------------------------------------------
// 센서 입력
// ---------------------------------------------------------------------------
void ElevationLayer::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_) {return;}

  // 센서 프레임(예: camera)의 점을 costmap 전역 프레임(map)으로 옮긴다.
  // 이 변환이 없으면 로봇이 움직여도 높이가 항상 같은 자리에 쌓여
  // 완전히 틀린 맵이 만들어진다.
  geometry_msgs::msg::TransformStamped tf_st;
  if (!lookupToGlobal(msg->header.frame_id, rclcpp::Time(msg->header.stamp), tf_st)) {
    return;
  }
  const auto & q = tf_st.transform.rotation;
  const auto & t = tf_st.transform.translation;
  // 쿼터니언 -> 회전행렬 (점마다 tf2 변환 호출은 비싸므로 한 번만 만든다)
  const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  const double r00 = 1 - 2 * (yy + zz), r01 = 2 * (xy - wz), r02 = 2 * (xz + wy);
  const double r10 = 2 * (xy + wz), r11 = 1 - 2 * (xx + zz), r12 = 2 * (yz - wx);
  const double r20 = 2 * (xz - wy), r21 = 2 * (yz + wx), r22 = 1 - 2 * (xx + yy);

  std::lock_guard<std::mutex> lock(data_mutex_);

  sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

  bool any = false;
  double lo_x = 0, lo_y = 0, hi_x = 0, hi_y = 0;

  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    const float sx = *ix, sy = *iy, sz = *iz;
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) {continue;}

    // 센서 프레임 -> 전역 프레임
    const double x = r00 * sx + r01 * sy + r02 * sz + t.x;
    const double y = r10 * sx + r11 * sy + r12 * sz + t.y;
    const double z = r20 * sx + r21 * sy + r22 * sz + t.z;

    // 높이 필터는 변환 '후' 값으로 판단해야 한다 (지면 기준이므로)
    if (z < min_obstacle_height_ || z > max_obstacle_height_) {continue;}

    unsigned int mx, my;
    if (!worldToMap(x, y, mx, my)) {continue;}

    cells_[cellIndex(mx, my)].add(z);

    if (!any) {
      lo_x = hi_x = x;
      lo_y = hi_y = y;
      any = true;
    } else {
      lo_x = std::min(lo_x, static_cast<double>(x));
      hi_x = std::max(hi_x, static_cast<double>(x));
      lo_y = std::min(lo_y, static_cast<double>(y));
      hi_y = std::max(hi_y, static_cast<double>(y));
    }
  }

  if (any) {
    if (!has_dirty_) {
      dirty_min_x_ = lo_x; dirty_max_x_ = hi_x;
      dirty_min_y_ = lo_y; dirty_max_y_ = hi_y;
      has_dirty_ = true;
    } else {
      dirty_min_x_ = std::min(dirty_min_x_, lo_x);
      dirty_max_x_ = std::max(dirty_max_x_, hi_x);
      dirty_min_y_ = std::min(dirty_min_y_, lo_y);
      dirty_max_y_ = std::max(dirty_max_y_, hi_y);
    }
  }
}


void ElevationLayer::laserScanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (!enabled_) {return;}

  // 2D LiDAR 는 높이 정보가 없다. 스캔 평면에 장애물이 있다는 사실만 반영한다.
  // 정확한 높이는 RGB-D 쪽이 담당.
  geometry_msgs::msg::TransformStamped tf_st;
  if (!lookupToGlobal(msg->header.frame_id, rclcpp::Time(msg->header.stamp), tf_st)) {
    return;
  }
  const auto & q = tf_st.transform.rotation;
  const auto & t = tf_st.transform.translation;
  // 스캔은 평면이므로 yaw 만 있으면 충분하다
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cy = std::cos(yaw), sy_ = std::sin(yaw);

  std::lock_guard<std::mutex> lock(data_mutex_);

  double angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
    const float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {continue;}
    if (r > max_sensor_range_) {continue;}

    const double lx = r * std::cos(angle);
    const double ly = r * std::sin(angle);
    // 센서 프레임 -> 전역 프레임 (평면 회전 + 평행이동)
    const double x = cy * lx - sy_ * ly + t.x;
    const double y = sy_ * lx + cy * ly + t.y;
    unsigned int mx, my;
    if (!worldToMap(x, y, mx, my)) {continue;}

    // LiDAR가 본 지점은 최소한 rover_traversable_max 를 넘는 장애물이다.
    // 보수적으로 그 경계값을 넣는다.
    cells_[cellIndex(mx, my)].add(static_cast<float>(rover_traversable_max_ + 0.01));
  }
  has_dirty_ = true;
}


// ---------------------------------------------------------------------------
// 지형 판정
// ---------------------------------------------------------------------------
double ElevationLayer::localSlopeDeg(unsigned int mx, unsigned int my) const
{
  // 3x3 이웃에 z = ax + by + c 평면을 최소자승 피팅하고
  // 법선 (-a, -b, 1) 과 수직축 사이 각도를 구한다.
  //   theta = atan(sqrt(a^2 + b^2))
  const double res = resolution_;
  double sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0, sz = 0, sx = 0, sy = 0;
  int n = 0;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int nx = static_cast<int>(mx) + dx;
      const int ny = static_cast<int>(my) + dy;
      if (nx < 0 || ny < 0 ||
        nx >= static_cast<int>(size_x_) || ny >= static_cast<int>(size_y_))
      {
        continue;
      }
      const auto & c = cells_[cellIndex(nx, ny)];
      if (!c.observed) {continue;}

      const double px = dx * res;
      const double py = dy * res;
      const double pz = c.mean();
      sx += px; sy += py; sz += pz;
      sxx += px * px; sxy += px * py; syy += py * py;
      sxz += px * pz; syz += py * pz;
      ++n;
    }
  }
  if (n < 4) {return 0.0;}   // 표본이 부족하면 판정 보류

  // 정규방정식을 중심화해서 푼다
  const double nd = n;
  const double cxx = sxx - sx * sx / nd;
  const double cxy = sxy - sx * sy / nd;
  const double cyy = syy - sy * sy / nd;
  const double cxz = sxz - sx * sz / nd;
  const double cyz = syz - sy * sz / nd;

  const double det = cxx * cyy - cxy * cxy;
  if (std::fabs(det) < 1e-12) {return 0.0;}

  const double a = (cyy * cxz - cxy * cyz) / det;
  const double b = (cxx * cyz - cxy * cxz) / det;

  return std::atan(std::sqrt(a * a + b * b)) * 180.0 / M_PI;
}


double ElevationLayer::maxStepHeight(unsigned int mx, unsigned int my) const
{
  const auto & c = cells_[cellIndex(mx, my)];
  if (!c.observed) {return 0.0;}

  double worst = 0.0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {continue;}
      const int nx = static_cast<int>(mx) + dx;
      const int ny = static_cast<int>(my) + dy;
      if (nx < 0 || ny < 0 ||
        nx >= static_cast<int>(size_x_) || ny >= static_cast<int>(size_y_))
      {
        continue;
      }
      const auto & o = cells_[cellIndex(nx, ny)];
      if (!o.observed) {continue;}
      worst = std::max(worst, std::fabs(static_cast<double>(c.mean() - o.mean())));
    }
  }
  return worst;
}


unsigned char ElevationLayer::classify(unsigned int mx, unsigned int my) const
{
  const auto & c = cells_[cellIndex(mx, my)];
  if (!c.observed) {return NO_INFORMATION;}

  const double h = c.max_z;   // 보수적으로 최대 높이를 쓴다

  // 1) 천장 제약 — 넘어가려면 로봇이 그 위를 날아야 한다
  //    장애물높이 + 비행고도 + 천장여유 > 천장  ->  통과 불가
  if (h + robot_flight_height_ + ceiling_clearance_ > ceiling_height_) {
    return cost_impassable_;
  }
  // 2) fly_over 상한 초과
  if (h > fly_over_max_) {
    return cost_impassable_;
  }
  // 3) 로버가 넘을 수 있는 높이인가
  if (h <= rover_traversable_max_) {
    // 높이는 낮아도 지형 특성 때문에 못 갈 수 있다
    const double slope = localSlopeDeg(mx, my);
    const double rough = c.stddev();
    const double step = maxStepHeight(mx, my);

    if (slope > max_slope_deg_ || rough > max_roughness_ || step > max_step_height_) {
      // 주행은 불가하지만 높이가 낮으므로 비행으로는 넘을 수 있다
      return cost_flyover_;
    }
    // 완전히 평탄하면 free, 아니면 주행 가능하되 비용 증가
    const bool pristine = (slope < max_slope_deg_ * 0.3) &&
      (rough < max_roughness_ * 0.3) && (step < max_step_height_ * 0.3);
    return pristine ? cost_free_ : cost_rover_;
  }
  // 4) 로버 한계 초과, fly_over 이하 -> 비행 필요
  return cost_flyover_;
}


// ---------------------------------------------------------------------------
// Layer 인터페이스
// ---------------------------------------------------------------------------
void ElevationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!has_dirty_) {return;}

  *min_x = std::min(*min_x, dirty_min_x_);
  *min_y = std::min(*min_y, dirty_min_y_);
  *max_x = std::max(*max_x, dirty_max_x_);
  *max_y = std::max(*max_y, dirty_max_y_);

  // 다음 주기를 위해 초기화 (갱신된 영역은 아래 updateCosts 에서 반영된다)
  has_dirty_ = false;
}


void ElevationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(data_mutex_);

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x_), max_i);
  max_j = std::min(static_cast<int>(size_y_), max_j);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const auto mx = static_cast<unsigned int>(i);
      const auto my = static_cast<unsigned int>(j);
      const unsigned char c = classify(mx, my);
      if (c == NO_INFORMATION) {continue;}

      // 자체 costmap 에도 기록해 둔다 (디버그/시각화용)
      costmap_[cellIndex(mx, my)] = c;

      // master 에는 더 큰 값만 덮어쓴다.
      // 다른 레이어(static/obstacle)가 이미 더 위험하다고 판단했으면 존중한다.
      const unsigned char old = master_grid.getCost(mx, my);
      if (old == NO_INFORMATION || c > old) {
        master_grid.setCost(mx, my, c);
      }
    }
  }

  if (publish_elevation_ && elevation_pub_) {
    publishElevationGrid();
  }
}


void ElevationLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (clear_on_reset_) {
    for (auto & c : cells_) {
      c.clear();
    }
    resetMaps();
  }
  has_dirty_ = false;
  current_ = true;
}


void ElevationLayer::publishElevationGrid()
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = global_frame_;
  msg.info.resolution = resolution_;
  msg.info.width = size_x_;
  msg.info.height = size_y_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;

  // 높이를 0~100 으로 스케일링한다 (0 = 0m, 100 = fly_over_max).
  // OccupancyGrid 는 -1(unknown)과 0~100만 표현할 수 있다.
  msg.data.resize(static_cast<size_t>(size_x_) * size_y_, -1);
  for (size_t k = 0; k < cells_.size(); ++k) {
    const auto & c = cells_[k];
    if (!c.observed) {continue;}
    const double ratio = std::clamp(
      static_cast<double>(c.max_z) / std::max(fly_over_max_, 1e-6), 0.0, 1.0);
    msg.data[k] = static_cast<int8_t>(ratio * 100.0);
  }
  elevation_pub_->publish(msg);
}

}  // namespace drobot_costmap_2_5d

PLUGINLIB_EXPORT_CLASS(
  drobot_costmap_2_5d::ElevationLayer, nav2_costmap_2d::Layer)
