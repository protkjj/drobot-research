// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// 2.5D Elevation Costmap Layer
//
// RGB-D 포인트클라우드로 셀별 지형 높이를 누적하고, traversability를 판정해
// Nav2 costmap의 cost 값으로 변환한다.
//
// 4단계 분류 (elevation_params.yaml)
//     Free              h <= rover_traversable_max        cost 0
//     Rover-traversable 주행 가능하지만 비용 증가          cost 100
//     Fly-over          h <= fly_over_max, 비행 필요       cost 200
//     Impassable        천장까지 막힘 또는 판정 불가        cost 254 (LETHAL)
//
// traversability 판정 지표 3개
//     1. 경사도   — 국부 평면 피팅의 법선 각도
//     2. 거칠기   — 국부 영역 고도 표준편차
//     3. 단차     — 인접 셀 간 고도 차이
//
// ---------------------------------------------------------------------------
// 이 레이어와 플래너의 연결
//
// Nav2 Costmap2D 는 셀당 0~255 cost 만 저장하므로 '높이' 자체는 전달되지 않는다.
// HybridAStarPlanner 는 cost 값에서 등급을 역추론해 대표 높이를 쓴다
// (drobot_hybrid_planner/state_space.hpp 의 CostmapTerrainSource 참고).
//
// 정밀 높이가 필요해지면 이 레이어가 높이맵을 별도 토픽으로 퍼블리시하고
// 플래너가 구독하도록 바꾸면 된다. publish_elevation_grid 파라미터로
// 디버그용 퍼블리시를 켤 수 있게 해 두었다.
// ---------------------------------------------------------------------------

#ifndef DROBOT_COSTMAP_2_5D__ELEVATION_LAYER_HPP_
#define DROBOT_COSTMAP_2_5D__ELEVATION_LAYER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace drobot_costmap_2_5d
{

/// 셀 하나의 높이 통계.
/// 개별 포인트를 다 들고 있으면 메모리가 폭증하므로,
/// 표준편차 계산에 필요한 최소 통계량만 온라인으로 누적한다.
struct ElevationCell
{
  float min_z = 0.0f;
  float max_z = 0.0f;
  float sum_z = 0.0f;
  float sum_z2 = 0.0f;   ///< 제곱합 — 분산을 한 번에 구하기 위함
  uint32_t count = 0;
  bool observed = false;

  void add(float z)
  {
    if (!observed) {
      min_z = max_z = z;
      observed = true;
    } else {
      if (z < min_z) {min_z = z;}
      if (z > max_z) {max_z = z;}
    }
    sum_z += z;
    sum_z2 += z * z;
    ++count;
  }

  float mean() const {return count ? sum_z / static_cast<float>(count) : 0.0f;}

  /// 표본 표준편차. 관측이 2개 미만이면 0.
  float stddev() const
  {
    if (count < 2) {return 0.0f;}
    const float n = static_cast<float>(count);
    const float var = (sum_z2 - sum_z * sum_z / n) / (n - 1.0f);
    return var > 0.0f ? std::sqrt(var) : 0.0f;
  }

  void clear() {*this = ElevationCell{};}
};


class ElevationLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ElevationLayer() = default;
  ~ElevationLayer() override = default;

  // ---- nav2_costmap_2d::Layer 인터페이스 -----------------------------
  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  void onFootprintChanged() override;
  bool isClearable() override {return true;}

  void matchSize() override;

private:
  // ---- 콜백 ----------------------------------------------------------
  void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void laserScanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  /// 센서 프레임의 점을 costmap 전역 프레임으로 옮기는 변환을 얻는다.
  /// Layer 기반 클래스가 tf_ 버퍼를 제공하므로 그걸 쓴다.
  /// 실패하면 false — 조용히 틀린 맵을 만드는 것보다 버리는 게 낫다.
  bool lookupToGlobal(
    const std::string & source_frame, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & out) const;

  // ---- 분류 ----------------------------------------------------------
  /// 셀의 높이와 국부 지형 특성으로 cost 를 정한다.
  unsigned char classify(unsigned int mx, unsigned int my) const;

  /// 국부 경사도 (도). 3x3 이웃에 최소자승 평면을 피팅해 법선 각도를 구한다.
  double localSlopeDeg(unsigned int mx, unsigned int my) const;

  /// 인접 셀과의 최대 고도차 (m)
  double maxStepHeight(unsigned int mx, unsigned int my) const;

  /// 디버그용 높이맵 퍼블리시
  void publishElevationGrid();

  size_t cellIndex(unsigned int mx, unsigned int my) const
  {
    return static_cast<size_t>(my) * size_x_ + mx;
  }

  // ---- 상태 ----------------------------------------------------------
  std::vector<ElevationCell> cells_;
  unsigned int size_x_ = 0;
  unsigned int size_y_ = 0;

  // 갱신 영역 추적 (updateBounds 에 넘길 범위)
  double dirty_min_x_ = 0.0;
  double dirty_min_y_ = 0.0;
  double dirty_max_x_ = 0.0;
  double dirty_max_y_ = 0.0;
  bool has_dirty_ = false;

  // ---- 파라미터 ------------------------------------------------------
  // 높이 임계값
  double rover_traversable_max_ = 0.15;
  double fly_over_max_ = 2.0;
  double ceiling_height_ = 2.5;

  // traversability
  double max_slope_deg_ = 15.0;
  double max_roughness_ = 0.03;
  double max_step_height_ = 0.05;

  // cost 값
  unsigned char cost_free_ = 0;
  unsigned char cost_rover_ = 100;
  unsigned char cost_flyover_ = 200;
  unsigned char cost_impassable_ = 254;

  // 로봇 형상 (Impassable 판정용)
  double robot_flight_height_ = 0.8;
  double ceiling_clearance_ = 0.5;

  // 센서
  std::string depth_topic_ = "/camera/depth/points";
  std::string scan_topic_ = "/scan";
  bool use_pointcloud_ = true;
  bool use_laserscan_ = true;
  double min_obstacle_height_ = -0.5;   ///< 이보다 낮은 점은 무시 (노이즈)
  double max_obstacle_height_ = 3.0;    ///< 이보다 높은 점은 무시 (천장)
  double max_sensor_range_ = 4.0;       ///< D435i depth 유효 범위

  // 동작
  bool enabled_param_ = true;
  bool publish_elevation_ = false;
  bool clear_on_reset_ = true;

  // ---- ROS ------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr elevation_pub_;

  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("ElevationLayer")};
  rclcpp::Clock::SharedPtr clock_;
  std::mutex data_mutex_;
};

}  // namespace drobot_costmap_2_5d

#endif  // DROBOT_COSTMAP_2_5D__ELEVATION_LAYER_HPP_
