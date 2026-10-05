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

  /// 카메라(점군)가 이 칸의 높이를 실제로 '측정'했는가.
  /// 아래 통계(min_z/max_z/...)는 카메라 관측만으로 채워진다.
  bool has_cloud = false;

  /// 2D LiDAR 가 이 칸에서 반사를 받았는가 (높이는 모름).
  /// LiDAR 는 수평 평면 한 장만 훑으므로 "스캔 높이에 뭔가 있다"는 사실만 준다.
  /// 32cm 말뚝인지 3m 벽인지 구분할 수 없다 — 그래서 높이 통계에는 넣지 않고
  /// 이 플래그로만 남긴다.
  bool hit_by_scan = false;

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

  /// rolling window 지원: 창의 원점이 로봇을 따라 이동하면, 자체 높이 격자
  /// cells_ 를 베이스가 costmap_ 를 옮기는 것과 똑같은 규칙으로 재매핑한다.
  /// 이게 없으면 로컬(rolling) costmap 에서 로봇이 전진할 때 높이가 이전
  /// 좌표에 남아, 빈 바닥이 장애물(보라색 덩어리)로 번진다.
  void updateOrigin(double new_origin_x, double new_origin_y) override;

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

  /// 정답 높이맵을 파일에서 읽어 cells_ 에 미리 심는다 (prior_map 파라미터).
  ///
  /// 왜 필요한가: 센서는 표면만 본다. 2D LiDAR 는 장애물 앞면에 막혀 내부를
  /// 못 보고, 전방 카메라도 낮은 박스 윗면을 비스듬히 스쳐 거의 못 본다.
  /// 그런데 global_costmap 이 track_unknown_space: false 라 미관측이
  /// 자유공간이 되어, 플래너가 장애물 내부를 관통하는 경로를 냈다.
  ///   실측 (2026-10-05): 높이 관측률 4.8%, nogap 월드에서 박스 영역의
  ///   100% 가 미관측 -> 지상 경로가 늘 존재 -> 비행이 선택되지 않음.
  ///
  /// 높이를 심으면 classify() 가 평소대로 돌아 내부까지 등급이 매겨진다.
  /// 센서는 그 위에 덧씌우므로 동적 장애물 대응은 그대로다.
  /// 실패해도 경고만 남기고 계속한다 — 맵이 없는 월드가 정상이기 때문이다.
  void loadPriorMap(const std::string & path);

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

  /// LiDAR 도 함께 맞은 칸을 flyover(넘어갈 수 있음)로 내릴 때 요구하는 상한.
  /// 전방 카메라는 가까운 큰 물체의 윗부분이 수직 FOV 를 벗어나 실제보다 낮게
  /// 측정된다. LiDAR 가 "여기 단단한 게 있다"고 말하는데 카메라 측정치가
  /// 이 값보다 높으면 과소추정일 수 있으므로 flyover 로 내리지 않는다.
  /// (진짜 넘을 수 있는 낮은 장애물은 이 값 아래라 영향이 없다.)
  double confident_flyover_max_ = 0.6;

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

  /// 정답 높이맵 파일 경로. 비어 있으면 쓰지 않는다 (센서만으로 동작).
  std::string prior_map_;

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
