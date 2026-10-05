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
#include <fstream>
#include <sstream>
#include <string>

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
  confident_flyover_max_ =
    getParam(node, p + "height_thresholds.confident_flyover_max", confident_flyover_max_);

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
  prior_map_ = getParam(node, p + "prior_map", std::string{});

  // 자체 격자(costmap_)에서 '아직 못 본 칸'의 값.
  // Costmap2D 기본 생성자는 default_value_ 를 초기화하지 않아서 (nav2 jazzy),
  // 그대로 두면 resizeMap/resetMaps 가 미정의 값으로 격자를 채운다.
  // 플래너가 이 격자에서 지형 등급을 읽으므로 (drobot_hybrid_planner 의
  // LayerTerrainSource) classify() 와 같은 뜻인 NO_INFORMATION 으로 못박는다.
  // master 에는 영향이 없다 — updateCosts 는 NO_INFORMATION 칸을 건너뛴다.
  default_value_ = NO_INFORMATION;

  matchSize();
  // matchSize() 가 cells_ 를 만든 뒤에 심어야 한다.
  if (!prior_map_.empty()) {
    loadPriorMap(prior_map_);
  }
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


// ---------------------------------------------------------------------------
// 정답 높이맵 (prior_map)
// ---------------------------------------------------------------------------
void ElevationLayer::loadPriorMap(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    RCLCPP_WARN(logger_, "prior_map 을 열 수 없다: '%s' — 센서만으로 동작한다", path.c_str());
    return;
  }

  // 주석(#) 을 건너뛰고 헤더 한 줄을 읽는다:
  //   <width> <height> <resolution> <origin_x> <origin_y>
  std::string line;
  unsigned int pw = 0, ph = 0;
  double pres = 0.0, pox = 0.0, poy = 0.0;
  bool header_ok = false;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream hs(line);
    if (hs >> pw >> ph >> pres >> pox >> poy) {header_ok = true;}
    break;
  }
  if (!header_ok || pw == 0 || ph == 0 || pres <= 0.0) {
    RCLCPP_WARN(logger_, "prior_map 헤더가 잘못됐다: '%s'", path.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  size_t seeded = 0, outside = 0, rows = 0;
  for (unsigned int j = 0; j < ph; ++j) {
    if (!std::getline(in, line)) {break;}
    std::istringstream rs(line);
    double h = 0.0;
    for (unsigned int i = 0; i < pw && (rs >> h); ++i) {
      // 격자 칸의 중심을 월드 좌표로. costmap 해상도가 달라도 worldToMap 이
      // 알맞은 칸으로 보내준다 (여러 칸이 한 칸에 모이면 add() 가 최대값을
      // 남기므로 보수적이다).
      const double wx = pox + (i + 0.5) * pres;
      const double wy = poy + (j + 0.5) * pres;
      unsigned int mx, my;
      if (!worldToMap(wx, wy, mx, my)) {++outside; continue;}
      const size_t ci = cellIndex(mx, my);
      cells_[ci].add(static_cast<float>(h));
      // 정답 높이는 카메라 측정보다 믿을 만하다. has_cloud 를 세워
      // classify() 와 병합 규칙이 '높이를 아는 칸'으로 다루게 한다.
      cells_[ci].has_cloud = true;
      ++seeded;
    }
    ++rows;
  }

  // 전체를 새로 판정해야 하므로 갱신 영역을 맵 전체로 잡는다.
  dirty_min_x_ = pox;
  dirty_min_y_ = poy;
  dirty_max_x_ = pox + pw * pres;
  dirty_max_y_ = poy + ph * pres;
  has_dirty_ = true;

  RCLCPP_INFO(
    logger_,
    "prior_map 적용: '%s' (%ux%u @ %.3f m, 원점 %.2f,%.2f) — %zu 칸 심음, "
    "%zu 칸은 costmap 밖",
    path.c_str(), pw, ph, pres, pox, poy, seeded, outside);
  if (rows != ph) {
    RCLCPP_WARN(
      logger_, "prior_map 행 수가 헤더와 다르다: 헤더 %u, 실제 %zu", ph, rows);
  }
}


void ElevationLayer::onFootprintChanged()
{
  // footprint 변화는 이 레이어 동작에 영향을 주지 않는다.
  // (인플레이션은 별도 레이어가 담당)
}


// ---------------------------------------------------------------------------
// rolling window 원점 이동
//
// 베이스 Costmap2D::updateOrigin 은 unsigned char costmap_ 만 새 원점으로
// 옮긴다. 우리의 높이 격자 cells_ 는 그와 평행한 별도 배열이라, 같은 규칙으로
// 함께 옮겨주지 않으면 (mx,my)↔월드 대응이 어긋나 높이가 엉뚱한 칸에 남는다.
// 셀 이동량(cell_ox, cell_oy)·겹침 영역 계산은 Costmap2D::updateOrigin 과
// 글자 그대로 같은 식을 써서 costmap_ 와 cells_ 가 1칸도 어긋나지 않게 한다.
// ---------------------------------------------------------------------------
void ElevationLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 새 원점을 격자 칸 단위 이동량으로 투영 (베이스와 동일한 truncation).
  const int cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  const int cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // 이동이 없으면 복사/리셋을 건너뛴다 (매 주기 불필요한 작업 방지).
  if (cell_ox == 0 && cell_oy == 0) {
    return;
  }

  const int sx = static_cast<int>(size_x_);
  const int sy = static_cast<int>(size_y_);

  // 새 창과 기존 창이 겹치는 영역 (기존 격자 좌표계 기준).
  const int ll_x = std::min(std::max(cell_ox, 0), sx);
  const int ll_y = std::min(std::max(cell_oy, 0), sy);
  const int ur_x = std::min(std::max(cell_ox + sx, 0), sx);
  const int ur_y = std::min(std::max(cell_oy + sy, 0), sy);
  const int ov_w = ur_x - ll_x;
  const int ov_h = ur_y - ll_y;

  // 겹치는 부분을 임시 버퍼에 보관.
  std::vector<ElevationCell> saved;
  if (ov_w > 0 && ov_h > 0) {
    saved.resize(static_cast<size_t>(ov_w) * static_cast<size_t>(ov_h));
    for (int row = 0; row < ov_h; ++row) {
      const size_t src = static_cast<size_t>(ll_y + row) * sx + ll_x;
      std::copy(
        cells_.begin() + src,
        cells_.begin() + src + ov_w,
        saved.begin() + static_cast<size_t>(row) * ov_w);
    }
  }

  // 전체를 미관측으로 리셋한 뒤, 겹치는 부분만 새 위치에 되돌려 놓는다.
  for (auto & c : cells_) {c.clear();}

  if (ov_w > 0 && ov_h > 0) {
    const int start_x = ll_x - cell_ox;
    const int start_y = ll_y - cell_oy;
    for (int row = 0; row < ov_h; ++row) {
      const size_t dst = static_cast<size_t>(start_y + row) * sx + start_x;
      std::copy(
        saved.begin() + static_cast<size_t>(row) * ov_w,
        saved.begin() + static_cast<size_t>(row) * ov_w + ov_w,
        cells_.begin() + dst);
    }
  }

  // 자체 char 격자 costmap_ 와 origin_x_/origin_y_ 는 베이스가 동일 규칙으로
  // 갱신한다. (베이스는 자신의 access 뮤텍스만 쓰므로 data_mutex_ 와 교착 없음.)
  nav2_costmap_2d::Costmap2D::updateOrigin(new_origin_x, new_origin_y);
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

    // 카메라만이 '실제로 측정한' 높이다. 통계는 여기서만 쌓인다.
    const size_t ci = cellIndex(mx, my);
    cells_[ci].add(z);
    cells_[ci].has_cloud = true;

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

  bool any = false;
  double lo_x = 0, lo_y = 0, hi_x = 0, hi_y = 0;

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

    // 높이는 '모른다'. 가짜 높이를 넣지 않는다.
    //
    // 예전에는 여기서 rover_traversable_max + 0.01 (= 0.16 m) 을 높이로 넣었다.
    // 그러면 classify 가 그 값을 flyover 구간(0.15 < h <= fly_over_max)으로 읽어
    // 3 m 벽까지 "16cm 턱 — 날아서 넘으면 됨(200)" 으로 분류했고, 그 200 이
    // obstacle_layer 의 254 를 덮어써 인플레이션이 금지 영역(253)을 만들지
    // 못했다. 가까이 갈수록(센서 범위 안) 벽이 녹아 없어져 모서리에 끼었다.
    //
    // 높이를 모를 때 안전한 추정은 '낮다' 가 아니라 '못 넘는다' 이다.
    // 사실만 남기고, 판정은 classify 가 한다.
    cells_[cellIndex(mx, my)].hit_by_scan = true;

    if (!any) {
      lo_x = hi_x = x;
      lo_y = hi_y = y;
      any = true;
    } else {
      lo_x = std::min(lo_x, x);
      hi_x = std::max(hi_x, x);
      lo_y = std::min(lo_y, y);
      hi_y = std::max(hi_y, y);
    }
  }

  // 갱신 영역을 실제 스캔 범위로 기록한다.
  // (예전에는 has_dirty_ 만 true 로 두어, 한 번도 갱신되지 않은 0 근처
  //  범위가 updateBounds 로 흘러갔다.)
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
      // 장애물 이웃은 '지형'이 아니다 — 평면 피팅에서 뺀다. 2026-10-05
      // 절벽을 가로질러 평면을 맞추면 바닥 칸의 경사가 수십 도로 나와,
      // 멀쩡한 바닥이 주행 불가로 판정된다 (장애물 둘레의 flyover 링).
      // 턱 너머 칸은 자기 높이로 따로 판정되므로 중복 처벌이기도 하다.
      if (c.mean() > rover_traversable_max_) {continue;}

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
      // 장애물 이웃과의 '단차'는 이 칸의 거칠기가 아니라 옆에 장애물이
      // 있다는 사실이다. 그 칸은 자기 높이로 flyover/impassable 이 되고,
      // 접근 여유는 inflation_layer 가 담당한다. 2026-10-05
      //
      // 이걸 빼지 않으면: 0.5 m 박스 옆 평지가 step 0.5 m > 0.05 에 걸려
      // flyover(200) 로 승격되고, 모든 장애물 둘레에 200 링이 생긴다.
      // prior_map 으로 전 영역에 높이를 심으면 링이 맵 전체를 감싸
      // 플래너가 해를 못 찾는다 (실측: "해 없음, 425 노드 확장").
      //
      // 진짜 험지(양쪽 다 낮은데 0.08 m 차이)는 여전히 걸린다 —
      // 두 칸 모두 rover_traversable_max_ 이하이기 때문이다.
      if (o.mean() > rover_traversable_max_) {continue;}
      worst = std::max(worst, std::fabs(static_cast<double>(c.mean() - o.mean())));
    }
  }
  return worst;
}


unsigned char ElevationLayer::classify(unsigned int mx, unsigned int my) const
{
  const auto & c = cells_[cellIndex(mx, my)];

  // 0) 높이 출처를 먼저 가린다.
  //    카메라(점군)만이 높이를 '측정'한다. 2D LiDAR 는 수평 평면 한 장이라
  //    "스캔 높이에 뭔가 있다"는 사실만 주고 높이는 원리적으로 모른다.
  if (!c.has_cloud) {
    // 카메라가 높이를 재지 못한 칸에서는 이 레이어가 아무 말도 하지 않는다.
    //
    // LiDAR 반사만 있는 칸을 여기서 254 로 찍고 싶은 유혹이 있지만, 그러면
    // 안 된다 — 이 레이어에는 '지우는' 수단이 없기 때문이다. ObstacleLayer 는
    // raytracing 으로 빔이 통과한 칸을 다시 free 로 되돌리지만, 여기에는
    // 그 기능이 없어서 한 번 선 표시가 영원히 남는다 (global 은 rolling 도
    // 아니라 창 밖으로 밀려나며 리셋되지도 않는다). 결국 유령 벽이 누적된다.
    //
    // 역할 분담:
    //   ObstacleLayer  — "여기 장애물이 있나" (마킹 + 클리어 모두 책임)
    //   ElevationLayer — "그게 얼마나 높나" (카메라가 실제로 잰 칸만)
    // 침묵하면 updateCosts 가 master 를 건드리지 않으므로 ObstacleLayer 의
    // 판정이 그대로 서고, 그쪽의 정상적인 클리어도 계속 동작한다.
    // 벽이 물렁해지는 문제는 updateCosts 의 병합 규칙(카메라 확인 없이는
    // 비용을 내리지 않는다)이 막는다.
    return NO_INFORMATION;
  }

  const double h = c.max_z;   // 보수적으로 최대 높이를 쓴다 (카메라 측정값)

  // 카메라가 쟀더라도, 가까운 큰 물체는 윗부분이 수직 FOV 를 벗어나 실제보다
  // 낮게 측정된다. LiDAR 가 같은 칸에서 반사를 받았는데 측정 높이가 확신
  // 구간을 넘으면 과소추정일 수 있으므로 flyover 로 내리지 않는다.
  const auto flyover_or_block = [&]() -> unsigned char {
      if (c.hit_by_scan && h > confident_flyover_max_) {return cost_impassable_;}
      return cost_flyover_;
    };

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
      return flyover_or_block();
    }
    // 완전히 평탄하면 free, 아니면 주행 가능하되 비용 증가
    const bool pristine = (slope < max_slope_deg_ * 0.3) &&
      (rough < max_roughness_ * 0.3) && (step < max_step_height_ * 0.3);
    return pristine ? cost_free_ : cost_rover_;
  }
  // 4) 로버 한계 초과, fly_over 이하 -> 비행 필요
  return flyover_or_block();
}


// ---------------------------------------------------------------------------
// Layer 인터페이스
// ---------------------------------------------------------------------------
void ElevationLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {return;}

  // rolling window: 창이 로봇을 따라 움직이면, 자체 격자(cells_, costmap_)를
  // 새 원점에 맞춰 이동시킨다. 이게 없으면 로봇이 전진할 때 높이가 이전
  // 좌표에 그대로 남아 빈 바닥이 장애물로 번진다 (로컬 costmap 사용 조건).
  // 고정 창(글로벌 costmap)에서는 isRolling()==false 라 아무 일도 하지 않는다.
  // ObstacleLayer 등 nav2 기본 레이어와 동일한 패턴.
  // updateOrigin 은 내부에서 data_mutex_ 를 직접 잠그므로, 아래 lock 보다
  // 먼저 (lock 밖에서) 호출해야 재진입 교착을 피한다.
  if (layered_costmap_->isRolling()) {
    updateOrigin(
      robot_x - getSizeInMetersX() / 2.0,
      robot_y - getSizeInMetersY() / 2.0);
  }

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
      const size_t ci = cellIndex(mx, my);
      const unsigned char c = classify(mx, my);
      if (c == NO_INFORMATION) {continue;}

      // 자체 격자에는 지형 등급을 그대로 남긴다 (플래너가 여기서 읽는다).
      costmap_[ci] = c;

      // master 병합 규칙
      //   비용을 '올리는' 것은 언제나 허용한다 (보수적이라 안전하다).
      //   비용을 '내리는' 것은 카메라가 실제로 높이를 잰 칸에서만 허용한다.
      //
      // 예전에는 조건 없이 덮어썼다. 그래서 LiDAR 만 본 칸의 flyover(200)
      // 판정이 obstacle_layer 의 254 를 끌어내렸고, 인플레이션이 금지
      // 영역(253)을 만들지 못해 컨트롤러가 벽으로 들어갔다.
      const unsigned char old_cost = master_grid.getCost(mx, my);
      if (old_cost == NO_INFORMATION || c >= old_cost) {
        master_grid.setCost(mx, my, c);
      } else if (cells_[ci].has_cloud && c >= cost_flyover_) {
        // 카메라가 "이 칸은 낮다"고 측정한 경우에만 다른 레이어 판정을 완화한다.
        // 2.5D 의 핵심 기능(넘을 수 있는 장애물을 254 → 200 으로 내려 비행
        // 경로를 열어주는 것)은 이 경로로 그대로 동작한다.
        //
        // 단 flyover(200) 아래로는 절대 내리지 않는다. 2026-10-05
        // 예전에는 가드가 없어서, 카메라가 벽 앞바닥이나 벽 밑동을 보고 그 칸을
        // 낮게(<=0.15 m) 측정하면 classify 가 free(0) 를 돌려주고, 그 값이
        // obstacle_layer 의 LETHAL(254) 을 덮어써 '벽에 구멍' 이 뚫렸다.
        // 플래너는 그 구멍으로 경로를 그었고(실측: 경로가 cost 254 칸을 통과),
        // 로봇은 그 경로를 따라가다 실제 벽에 부딪혔다. 로봇이 움직이면 시야가
        // 바뀌어 구멍이 메워지므로 스냅샷으로는 잘 안 잡히는 간헐적 현상이다.
        //
        // 원칙: 다른 센서가 '뭔가 있다' 고 한 칸에 대해 카메라가 증명할 수 있는
        // 것은 '낮다' 까지이지 '아무것도 없다' 가 아니다. 높이를 낮게 쟀다는
        // 이유로 장애물을 지우면 안 된다.
        master_grid.setCost(mx, my, c);
      }
      // 그 외(LiDAR 만 본 칸)는 기존 판정을 유지한다 — 통행 가능성을 발명하지 않는다.
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
