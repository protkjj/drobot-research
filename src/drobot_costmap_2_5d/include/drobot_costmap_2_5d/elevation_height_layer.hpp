// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef DROBOT_COSTMAP_2_5D__ELEVATION_HEIGHT_LAYER_HPP_
#define DROBOT_COSTMAP_2_5D__ELEVATION_HEIGHT_LAYER_HPP_

#include <memory>
#include <string>

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace drobot_costmap_2_5d
{

class ElevationLayer;

/// ElevationLayer 가 잰 실제 높이를 자기 격자에 담아 두는 레이어.
///
/// 왜 따로 두나
///   ElevationLayer 의 격자는 지형 '등급'(0 / 100 / 200 / 254) 이다. 플래너는 그
///   등급만 받아서 fly_over(200) 칸을 전부 대표 높이 0.60 m 로 읽었고, 0.6 m 를 넘는
///   장애물 위로 충돌 고도를 계획했다 (drobot_hybrid_planner/COST_HEIGHT_CONTRACT.md 4 절).
///   등급과 높이는 둘 다 필요하다 — 낮지만 거친 땅은 높이만으로 가를 수 없다.
///
///   플래너는 같은 프로세스(planner_server 의 global_costmap)에서 레이어를 이름으로
///   찾아 그 격자를 읽는다 (CostmapLayer = Costmap2D). 그 방식을 그대로 쓰도록 높이를
///   레이어 하나로 낸다. 토픽이 아니라 지연·동기화 문제가 없고, 플래너가
///   drobot_costmap_2_5d 에 컴파일 의존할 필요도 없다.
///
/// 격자 값 (A→B 계약 — 플래너 LayerTerrainSource 가 같은 규칙으로 읽는다)
///   0 ~ 250  높이 = 값 × height_unit (기본 0.01 m)
///   255      모름 (카메라가 재지 않은 칸)
///
/// master 는 건드리지 않는다. 플래너만 읽는 격자다.
class ElevationHeightLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ElevationHeightLayer() = default;
  ~ElevationHeightLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override {}
  bool isClearable() override {return false;}

private:
  ElevationLayer * findSource();

  std::string source_name_{"elevation_layer"};
  double unit_m_ = 0.01;
  ElevationLayer * source_ = nullptr;
  bool warned_ = false;
  rclcpp::Logger logger_{rclcpp::get_logger("ElevationHeightLayer")};
};

}  // namespace drobot_costmap_2_5d

#endif  // DROBOT_COSTMAP_2_5D__ELEVATION_HEIGHT_LAYER_HPP_
