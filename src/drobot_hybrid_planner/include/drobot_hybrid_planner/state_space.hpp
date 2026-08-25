// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// 하이브리드 상태공간 — 지상/공중 모드를 함께 다루는 격자 탐색 공간
//
// benchmark/planners/state_space.py 를 C++로 이식한 것이다.
// Python 벤치마크와 '같은 문제'를 풀어야 결과를 그대로 인용할 수 있으므로,
// 상태 정의·전이 규칙·비용 계산을 그대로 옮겼다.
//
// 상태 표현 (3D)
//     (mx, my, mode)   mode: GROUND=0 / AIR=1
//     air 모드의 고도는 상태에 없다. 지형높이 + flight_clearance 로 자동 결정.
//
// 전이 (엣지)
//     1. 지상 이동   ground -> ground   (8방향 인접)
//     2. 공중 이동   air -> air         (8방향 인접)
//     3. 이륙        ground -> air      (같은 셀)
//     4. 착륙        air -> ground      (같은 셀)
//
// ---------------------------------------------------------------------------
// 지형 높이를 어디서 얻는가 — 설계상 중요한 선택
//
// Nav2 의 Costmap2D 는 셀당 0~255 의 cost 만 저장한다. 높이 정보가 없다.
// 그런데 이 플래너는 '지형 높이'가 있어야 한다:
//     - 비행 고도 = 지형 높이 + flight_clearance
//     - ground effect 판정에 clearance(= z - h) 가 필요
//
// 선택지는 셋이었다:
//   (a) cost 값에서 등급을 읽고 등급별 대표 높이를 쓴다
//   (b) ElevationLayer 가 높이맵을 별도 토픽으로 퍼블리시하고 구독한다
//   (c) grid_map 패키지를 도입한다
//
// Phase 1 에서는 (a)를 쓴다. elevation_params.yaml 이 이미
// free/rover_traversable/fly_over/impassable 4단계와 cost 값을 정의하고 있어
// 추가 의존성 없이 바로 동작하기 때문이다.
// 다만 등급 대표값이라 실제 높이와 오차가 있으므로, TerrainSource 를
// 인터페이스로 빼서 나중에 (b)나 (c)로 갈아끼울 수 있게 했다.
// ---------------------------------------------------------------------------

#ifndef DROBOT_HYBRID_PLANNER__STATE_SPACE_HPP_
#define DROBOT_HYBRID_PLANNER__STATE_SPACE_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>

#include "drobot_hybrid_planner/energy_model.hpp"

namespace drobot_hybrid_planner
{

enum Mode : uint8_t { GROUND = 0, AIR = 1 };

/// 격자 상태. 해시 키로 쓰기 위해 정수 하나로 압축할 수 있게 했다.
///
/// level 은 비행 고도 인덱스다 (ProblemSpec::airLevels() 의 첨자).
/// GROUND 일 때는 항상 0.
///
/// 왜 고도가 상태에 들어가는가:
///     예전에는 비행 고도를 '지형높이 + flight_clearance' 로 매 셀 재계산했다
///     (지형 추종). 그러면 장애물 경계에서 한 셀 만에 고도가 장애물 높이만큼
///     점프한다. 격자 0.05m 에서 0.60m 장애물이면 상승각 85도가 되어
///     최대상승각(45도) 제약에 걸려 전이가 거부된다.
///     결과적으로 하이브리드 플래너가 어떤 장애물도 넘지 못했다
///     (test_state_space.cpp 참고).
///
///     그래서 '비행 구간 고도 고정' 방식을 쓴다:
///       - 이륙할 때 도달할 고도를 정하고
///       - 그 비행 구간 내내 같은 고도를 유지하며
///       - 착륙할 때 내려온다
///     실제 드론 운용(장애물 앞에서 미리 상승 후 수평 통과)과 일치한다.
struct State
{
  unsigned int mx = 0;
  unsigned int my = 0;
  uint8_t mode = GROUND;
  uint8_t level = 0;

  bool operator==(const State & o) const
  {
    return mx == o.mx && my == o.my && mode == o.mode && level == o.level;
  }
};

/// 지형 높이 조회 인터페이스.
/// 구현을 갈아끼울 수 있게 분리해 둔다 (위 주석 참고).
class TerrainSource
{
public:
  virtual ~TerrainSource() = default;
  /// 셀의 지형 높이 (m). 통과 불가면 매우 큰 값을 돌려준다.
  virtual double heightAt(unsigned int mx, unsigned int my) const = 0;
  /// 로버가 이 셀에 있을 수 있는가
  virtual bool roverTraversable(unsigned int mx, unsigned int my) const = 0;
};

/// Costmap2D 의 cost 값을 등급으로 읽어 대표 높이를 돌려주는 구현.
///
/// elevation_params.yaml 의 cost_values 매핑을 그대로 쓴다:
///     free(0) / rover_traversable(100) / fly_over(200) / impassable(254)
class CostmapTerrainSource : public TerrainSource
{
public:
  struct Config
  {
    // cost 임계값 (이 값 이하면 해당 등급)
    unsigned char free_max = 50;
    unsigned char rover_max = 150;
    unsigned char flyover_max = 253;
    // 등급별 대표 높이 (m)
    double h_free = 0.0;
    double h_rover = 0.15;      ///< rover_traversable_max
    double h_flyover = 0.60;    ///< fly-over 구간 대표값
    double h_impassable = 99.0;
  };

  CostmapTerrainSource(nav2_costmap_2d::Costmap2D * costmap, const Config & cfg)
  : costmap_(costmap), cfg_(cfg) {}

  double heightAt(unsigned int mx, unsigned int my) const override;
  bool roverTraversable(unsigned int mx, unsigned int my) const override;

private:
  nav2_costmap_2d::Costmap2D * costmap_;
  Config cfg_;
};


/// 이동 방식(modal) — 장애물을 만났을 때 '어떻게 넘어가느냐'로 나뉜다.
///
///   modal          우회   밟고넘기   비행
///   ------------------------------------
///   RoverDetour     O       X        X    장애물을 피해 돌아간다
///   RoverClimb      O       O        X    낮은 장애물은 타고 넘는다
///   Hybrid          O       X        O    드론으로 전환해 날아 넘는다
///
/// 셋이 같은 비용 함수를 쓰므로 결과를 직접 비교할 수 있다.
/// 밟고넘기는 climb_mode.max_height (기본 0.7m) 이하 장애물만 가능하다.
enum class Modal : uint8_t
{
  RoverDetour = 0,
  RoverClimb = 1,
  Hybrid = 2,
};

/// 한 번의 경로계획 문제를 정의하는 모든 것.
class ProblemSpec
{
public:
  struct Params
  {
    double flight_clearance = 0.8;    ///< 비행 고도 = 지형높이 + 이 값
    double ceiling_height = 2.5;      ///< 천장 높이
    bool allow_diagonal = true;
    bool check_diagonal_corners = true;
    double rover_max_height = 0.15;   ///< 로버 통과 가능 높이 상한
    Modal modal = Modal::Hybrid;      ///< 이동 방식 제한
  };

  ProblemSpec(
    nav2_costmap_2d::Costmap2D * costmap,
    std::shared_ptr<TerrainSource> terrain,
    const EnergyModel * model,
    const Params & params);

  // ---- 상태 유효성 ---------------------------------------------------
  bool inBounds(unsigned int mx, unsigned int my) const;
  double terrain(unsigned int mx, unsigned int my) const;
  bool groundOk(unsigned int mx, unsigned int my) const;

  // ---- modal 별 능력 --------------------------------------------------
  /// 드론으로 전환해 날 수 있는가
  bool allowFly() const {return params_.modal == Modal::Hybrid;}
  /// 장애물을 밟고 넘을 수 있는가
  bool allowClimb() const {return params_.modal == Modal::RoverClimb;}
  /// 로버가 올라설 수 있는 지형 높이의 상한 (m).
  /// 이 한 줄이 '우회'와 '밟고넘기'를 가르는 지점이다.
  double roverHLimit() const
  {
    return allowClimb() ? model_->roverClimbMaxH() : params_.rover_max_height;
  }

  /// 비행 고도 후보 목록 (오름차순).
  ///
  /// 맵에 실제로 존재하는 장애물 높이에서만 만든다 (h + flight_clearance).
  /// 그보다 높이 날 이유가 없고, 낮게 날면 장애물에 걸린다.
  /// CostmapTerrainSource 는 등급이 4개뿐이라 후보도 소수다.
  const std::vector<double> & airLevels() const {return air_levels_;}

  /// level 인덱스에 해당하는 실제 고도 (m)
  double levelZ(uint8_t level) const {return air_levels_[level];}

  /// 고정 고도 level 로 이 셀 위를 날 수 있는가.
  /// 지형 위 최소 여유를 지키고 천장 아래여야 한다.
  bool airOkAt(unsigned int mx, unsigned int my, uint8_t level) const;

  /// 대각선 이동에서 모서리를 뚫고 지나가지 않는가.
  ///
  /// 8방향 격자의 고전적 함정(corner-cutting): (0,0)->(1,1) 대각 이동이
  /// (1,0)과 (0,1)이 벽이어도 허용되면 벽 모서리를 대각으로 통과하는
  /// 물리적으로 불가능한 경로가 나온다.
  /// Python 벤치마크에서 실제로 이 버그 때문에 높이 1.10m 벽을 관통하는
  /// 경로가 "최적해"로 나왔다.
  bool diagOkGround(unsigned int mx, unsigned int my, int dx, int dy) const;
  bool diagOkAir(unsigned int mx, unsigned int my, int dx, int dy, uint8_t level) const;

  // ---- 전이 ----------------------------------------------------------
  struct Edge
  {
    State next;
    CostAccumulator acc;
  };
  /// 상태 s 에서 갈 수 있는 (다음상태, 비용) 목록을 out 에 채운다.
  void neighbors(const State & s, std::vector<Edge> & out) const;

  /// 지상 이동 한 스텝의 비용. 밟고넘기 modal 이면 상승분에 등반 비용을 더한다.
  CostAccumulator groundEdge(
    unsigned int mx, unsigned int my,
    unsigned int nx, unsigned int ny, double dist) const;

  // ---- 휴리스틱 ------------------------------------------------------
  /// 목표까지의 비용 하한 추정 (admissible 이어야 최적성 보장).
  /// 가장 싼 이동 수단인 지상 주행 단가로 하한을 잡는다.
  double heuristic(const State & s, const State & goal) const;

  // ---- 접근자 --------------------------------------------------------
  double zMax() const {return z_max_;}
  double hLimit() const {return h_limit_;}
  double resolution() const {return costmap_->getResolution();}
  nav2_costmap_2d::Costmap2D * costmap() const {return costmap_;}
  const EnergyModel * model() const {return model_;}
  const Params & params() const {return params_;}

  /// 상태를 정수 인덱스로 압축 (해시맵 키)
  /// 상태를 정수 하나로 압축한다 (closed 배열 첨자 / 해시 키).
  ///
  /// 배치: (셀 인덱스) x (1 + 고도 후보 수)
  ///   슬롯 0        = GROUND
  ///   슬롯 1..n     = AIR, level 0..n-1
  /// GROUND 는 level 을 쓰지 않으므로 슬롯 하나면 된다.
  size_t index(const State & s) const
  {
    const size_t cell =
      static_cast<size_t>(s.my) * costmap_->getSizeInCellsX() + s.mx;
    const size_t slot = (s.mode == GROUND) ? 0 : (1 + static_cast<size_t>(s.level));
    return cell * slotsPerCell() + slot;
  }
  size_t numStates() const
  {
    return static_cast<size_t>(costmap_->getSizeInCellsX()) *
           costmap_->getSizeInCellsY() * slotsPerCell();
  }
  size_t slotsPerCell() const {return 1 + air_levels_.size();}

private:
  /// 생성자에서 costmap 을 훑어 비행 고도 후보를 만든다.
  void buildAirLevels();

  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<TerrainSource> terrain_;
  const EnergyModel * model_;
  Params params_;

  double z_max_ = 1.75;      ///< 천장 제약에서 나온 최대 비행 고도
  double h_limit_ = 0.95;    ///< 통과 가능한 장애물 높이 상한
  double unit_cost_ground_;  ///< 지상 주행 1m당 비용 (휴리스틱용)

  /// 비행 고도 후보 (오름차순). 생성자에서 costmap 을 훑어 만든다.
  std::vector<double> air_levels_;
};

}  // namespace drobot_hybrid_planner

#endif  // DROBOT_HYBRID_PLANNER__STATE_SPACE_HPP_
