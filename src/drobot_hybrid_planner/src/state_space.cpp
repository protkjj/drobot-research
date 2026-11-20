// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "drobot_hybrid_planner/state_space.hpp"

#include <algorithm>
#include <set>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

// NO_INFORMATION 등 cost 상수는 별도 헤더에 있다.
// costmap_2d.hpp 만으로는 안 딸려온다.
#include <nav2_costmap_2d/cost_values.hpp>

namespace drobot_hybrid_planner
{

namespace
{
// 8방향 이웃: (dx, dy, 거리배수)
struct Neighbor { int dx; int dy; double k; };
constexpr double kSqrt2 = 1.41421356237309504880;

const Neighbor kNeighbors8[] = {
  {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
  {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2},
};
const Neighbor kNeighbors4[] = {
  {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
};
}  // namespace


// ---------------------------------------------------------------------------
// CostmapTerrainSource
// ---------------------------------------------------------------------------
double CostmapTerrainSource::heightAt(unsigned int mx, unsigned int my) const
{
  const unsigned char c = costmap_->getCost(mx, my);
  if (c <= cfg_.free_max) {return cfg_.h_free;}
  if (c <= cfg_.rover_max) {return cfg_.h_rover;}
  if (c <= cfg_.flyover_max) {return cfg_.h_flyover;}
  return cfg_.h_impassable;
}

bool CostmapTerrainSource::roverTraversable(unsigned int mx, unsigned int my) const
{
  const unsigned char c = costmap_->getCost(mx, my);
  // NO_INFORMATION(255)은 보수적으로 통과 불가로 본다.
  if (c == nav2_costmap_2d::NO_INFORMATION) {return false;}
  return c <= cfg_.rover_max;
}


// ---------------------------------------------------------------------------
// LayerTerrainSource
// ---------------------------------------------------------------------------
double LayerTerrainSource::heightAt(unsigned int mx, unsigned int my) const
{
  // LiDAR 등 다른 레이어가 LETHAL 로 막은 곳은 높이를 모르므로 통과 불가.
  // 비행도 막아야 한다 — 안 그러면 등급 격자가 못 본 장애물 위로 충돌 고도
  // 계획이 나온다 (COST_HEIGHT_CONTRACT.md 4절).
  if (master_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    return cfg_.h_impassable;
  }
  const unsigned char g = grade_->getCost(mx, my);
  if (g == nav2_costmap_2d::NO_INFORMATION) {return cfg_.h_free;}   // 미관측 — 낙관
  if (g <= cfg_.free_max) {return cfg_.h_free;}
  if (g <= cfg_.rover_max) {return cfg_.h_rover;}
  if (g <= cfg_.flyover_max) {return cfg_.h_flyover;}
  return cfg_.h_impassable;
}

bool LayerTerrainSource::roverTraversable(unsigned int mx, unsigned int my) const
{
  // 지형만 본다. 충돌(footprint)은 collides() 가 따로 판정한다.
  const unsigned char g = grade_->getCost(mx, my);
  if (g == nav2_costmap_2d::NO_INFORMATION) {return true;}          // 미관측 — 낙관
  return g <= cfg_.rover_max;
}

bool LayerTerrainSource::collides(unsigned int mx, unsigned int my) const
{
  const unsigned char c = master_->getCost(mx, my);
  if (c == nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
  // 출발점 주변의 253 은 빠져나갈 수 있게 둔다 (relaxInscribedAround)
  return c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && !relaxed(mx, my);
}

bool LayerTerrainSource::relaxed(unsigned int mx, unsigned int my) const
{
  if (relax_r_ == 0) {return false;}
  const int64_t dx = static_cast<int64_t>(mx) - static_cast<int64_t>(relax_mx_);
  const int64_t dy = static_cast<int64_t>(my) - static_cast<int64_t>(relax_my_);
  const int64_t r = static_cast<int64_t>(relax_r_);
  return dx * dx + dy * dy <= r * r;
}

bool LayerTerrainSource::observed(unsigned int mx, unsigned int my) const
{
  return grade_->getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION;
}


// ---------------------------------------------------------------------------
// ProblemSpec
// ---------------------------------------------------------------------------
ProblemSpec::ProblemSpec(
  nav2_costmap_2d::Costmap2D * costmap,
  std::shared_ptr<TerrainSource> terrain,
  const EnergyModel * model,
  const Params & params)
: costmap_(costmap), terrain_(std::move(terrain)), model_(model), params_(params)
{
  z_max_ = model_->maxFlightAltitude(params_.ceiling_height);

  // 3D: 고도가 h + flight_clearance 로 고정이므로 h + clearance <= z_max
  h_limit_ = z_max_ - params_.flight_clearance;

  buildAirLevels();

  // 비행 거리 구간 수. fbin 이 uint8_t 라 255 를 넘지 않게 한다.
  // 구간 폭이 너무 작으면 폭을 넓혀서라도 상한을 지킨다 (제약은 flown 으로 정확히 건다).
  if (limitsFlight()) {
    const double bin = std::max(params_.flight_distance_bin, 1e-3);
    n_fbins_ = static_cast<unsigned int>(
      std::min(255.0, std::ceil(params_.max_flight_distance / bin)));
    n_fbins_ = std::max(n_fbins_, 1u);
    params_.flight_distance_bin = params_.max_flight_distance / n_fbins_;
  }

  // 1m 이동의 최소 비용 (휴리스틱 하한용).
  // 예전에는 '비행은 지상보다 비싸다' 고 보고 지상 단가만 썼다. 그런데
  // derived 세트는 비행 0.78 < 지상 0.80 (1m 당 비용)이라 휴리스틱이 실제
  // 비용을 넘어 admissible 하지 않았다 — A* 가 최적해를 보장하지 못한다.
  // 두 단가 중 작은 쪽을 쓴다. 비행 단가는 ground effect 없는 값(가장 싼 경우),
  // 이착륙 고정비는 0 으로 본다 (하한이므로).
  const double ground = model_->cost(model_->groundMove(1.0));
  const double air = model_->cost(model_->airMoveHorizontal(1.0, 1e3));
  unit_cost_min_ = allowFly() ? std::min(ground, air) : ground;

  // 경로 여유 지도. 플래너는 계획마다 다시 만든다 (createPlan).
  // 여유 비용은 0 이상이라 위 휴리스틱 하한은 그대로 admissible 하다.
  updateClearance();
}


bool ProblemSpec::inBounds(unsigned int mx, unsigned int my) const
{
  return mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY();
}

double ProblemSpec::terrain(unsigned int mx, unsigned int my) const
{
  return terrain_->heightAt(mx, my);
}

bool ProblemSpec::groundOk(unsigned int mx, unsigned int my) const
{
  if (!inBounds(mx, my)) {return false;}
  // footprint 가 장애물과 겹치면 어느 modal 이든 못 선다.
  // (밟고넘기도 — 장애물 '위'는 갈 수 있어도 벽에 몸이 끼는 자리는 안 된다)
  if (terrain_->collides(mx, my)) {return false;}
  // 이 modal 에서 '설 수 없는 지형' 에 너무 붙은 칸 (경로 여유, 꺼져 있으면 통과).
  // collides() 는 master 의 LETHAL·INSCRIBED 만 보므로, inflation 이 안 붙는
  // 1.2 m 이하 장애물(상자)에는 몸체 충돌 검사가 없었다 — 이게 그 구멍을 막는다.
  if (!clearanceOk(mx, my)) {return false;}
  if (!allowClimb()) {
    return terrain_->roverTraversable(mx, my);
  }
  // 밟고넘기 modal 에서는 장애물 위(<= roverHLimit)도 '있을 수 있는 곳'이다.
  // 1e-9 여유는 0.70m 장애물이 부동소수 오차로 걸러지는 걸 막기 위함.
  const double h = terrain_->heightAt(mx, my);
  return h <= roverHLimit() + 1e-9;
}

bool ProblemSpec::nearestGroundCell(
  unsigned int mx, unsigned int my, unsigned int radius_cells,
  unsigned int & ox, unsigned int & oy) const
{
  const int64_t r = static_cast<int64_t>(radius_cells);
  int64_t best = -1;
  for (int64_t dy = -r; dy <= r; ++dy) {
    for (int64_t dx = -r; dx <= r; ++dx) {
      const int64_t d2 = dx * dx + dy * dy;
      if (d2 > r * r || (best >= 0 && d2 >= best)) {continue;}
      const int64_t x = static_cast<int64_t>(mx) + dx;
      const int64_t y = static_cast<int64_t>(my) + dy;
      if (x < 0 || y < 0) {continue;}
      if (!groundOk(static_cast<unsigned int>(x), static_cast<unsigned int>(y))) {continue;}
      best = d2;
      ox = static_cast<unsigned int>(x);
      oy = static_cast<unsigned int>(y);
    }
  }
  return best >= 0;
}

bool ProblemSpec::landingOk(unsigned int mx, unsigned int my) const
{
  if (!groundOk(mx, my)) {return false;}
  // 주행은 미관측을 평지로 보고 들어가 본다 (가 보면 보인다).
  // 착륙은 다르다 — 카메라가 못 본 장애물 윗면도 '평지'로 보이므로
  // 거기 내려앉으면 장애물 속에 박힌다. 2026-10-02 시뮬에서 실제로 그랬다.
  return !params_.land_only_on_observed || terrain_->observed(mx, my);
}


// ---------------------------------------------------------------------------
// 경로 여유 — A* 가 로봇을 점으로 보고 장애물에 바짝 붙는 것을 막는다.
//
// 2026-10-02 시뮬: 경로가 상자 모서리를 바짝 돌아 로봇이 모서리에 박혔다
// (base_map_h0.5, 3회 중 2회 aborted, 1회는 뒤집힘). Nav2 순정 플래너는 inflation
// 비용을 이동 비용에 더해 가운데를 고르지만, 이 플래너는 inflation 을 충돌(253·254)
// 에만 쓰고, 상자처럼 1.2 m 이하 장애물에는 inflation 자체가 안 붙는다.
// 그래서 'modal 기준으로 설 수 없는 칸' 까지의 거리를 직접 재서
//   (1) min_ground_clearance 보다 가까우면 주행·착륙 불가
//   (2) 가까울수록 clearance_weight·exp(-decay·(d - min)) 만큼 1 m 당 비용 추가
// 를 건다. 비용은 E·T 와 별도 항(CostAccumulator::clearance_penalty)이다.
// ---------------------------------------------------------------------------
bool ProblemSpec::groundObstacle(unsigned int mx, unsigned int my) const
{
  // groundOk 의 지형 판정과 같은 기준이다 — roverHLimit 이 우회와 밟고넘기를 가른다.
  // 밟고넘기(0.7 m)에서는 0.5 m 상자가 장애물이 아니라 올라탈 곳이다.
  // LayerTerrainSource 는 master LETHAL 칸을 h_impassable 로 돌려주므로 벽도 걸린다.
  // 미관측 칸은 평지로 읽혀 장애물이 아니다 — '주행은 낙관' 과 같다.
  return terrain_->heightAt(mx, my) > roverHLimit() + 1e-9;
}

void ProblemSpec::updateClearance()
{
  if (!clearanceEnabled()) {
    clearance_.clear();
    return;
  }
  const unsigned int nx = costmap_->getSizeInCellsX();
  const unsigned int ny = costmap_->getSizeInCellsY();
  const double res = costmap_->getResolution();
  // 판정에 필요한 거리까지만 잰다. 그보다 먼 칸은 '충분히 멀다' 값으로 남는다.
  const float far = static_cast<float>(
    std::max(params_.clearance_range, params_.min_ground_clearance) + res);
  clearance_.assign(static_cast<size_t>(nx) * ny, far);

  // 장애물 칸 전부에서 동시에 퍼져 나가는 다익스트라 (8방향, 대각은 √2 칸).
  // 정확한 유클리드 거리보다 최대 8 % 쯤 길게 나오지만 (chamfer 근사) 여유 판정에는 충분하다.
  using Item = std::pair<float, size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  for (unsigned int my = 0; my < ny; ++my) {
    for (unsigned int mx = 0; mx < nx; ++mx) {
      if (!groundObstacle(mx, my)) {continue;}
      const size_t i = static_cast<size_t>(my) * nx + mx;
      clearance_[i] = 0.0f;
      open.push({0.0f, i});
    }
  }
  while (!open.empty()) {
    const Item top = open.top();
    open.pop();
    const float d = top.first;
    const size_t i = top.second;
    if (d > clearance_[i]) {continue;}   // 더 짧은 거리로 이미 갱신된 칸
    const int mx = static_cast<int>(i % nx);
    const int my = static_cast<int>(i / nx);
    for (const auto & nb : kNeighbors8) {
      const int x = mx + nb.dx;
      const int y = my + nb.dy;
      const bool inside = x >= 0 && y >= 0 &&
        x < static_cast<int>(nx) && y < static_cast<int>(ny);
      if (!inside) {continue;}
      const float nd = d + static_cast<float>(nb.k * res);
      if (nd >= far) {continue;}
      const size_t j = static_cast<size_t>(y) * nx + static_cast<size_t>(x);
      if (nd < clearance_[j]) {
        clearance_[j] = nd;
        open.push({nd, j});
      }
    }
  }
}

double ProblemSpec::clearance(unsigned int mx, unsigned int my) const
{
  if (clearance_.empty() || !inBounds(mx, my)) {
    return std::numeric_limits<double>::infinity();
  }
  return clearance_[static_cast<size_t>(my) * costmap_->getSizeInCellsX() + mx];
}

bool ProblemSpec::clearanceOk(unsigned int mx, unsigned int my) const
{
  if (params_.min_ground_clearance <= 0.0 || clearance_.empty()) {return true;}
  // 출발점 주변은 풀어 준다 — 이미 장애물 옆에 서 있어도 빠져나갈 수 있게
  if (relax_r_ > 0) {
    const int64_t dx = static_cast<int64_t>(mx) - static_cast<int64_t>(relax_mx_);
    const int64_t dy = static_cast<int64_t>(my) - static_cast<int64_t>(relax_my_);
    const int64_t r = static_cast<int64_t>(relax_r_);
    if (dx * dx + dy * dy <= r * r) {return true;}
  }
  // float 로 저장한 거리라 아주 작은 여유를 둔다 (6 칸 × 0.05 m = 0.30 이 걸러지지 않게)
  return clearance(mx, my) >= params_.min_ground_clearance - 1e-6;
}

double ProblemSpec::clearancePenaltyPerMeter(unsigned int mx, unsigned int my) const
{
  if (params_.clearance_weight <= 0.0 || clearance_.empty()) {return 0.0;}
  const double d = clearance(mx, my);
  if (d >= params_.clearance_range) {return 0.0;}
  // Nav2 InflationLayer 와 같은 모양 — 최소 여유에서 최대, 멀어질수록 지수적으로 준다
  const double x = std::max(0.0, d - params_.min_ground_clearance);
  return params_.clearance_weight * std::exp(-params_.clearance_decay * x);
}

double ProblemSpec::segmentClearancePenalty(double ax, double ay, double bx, double by) const
{
  if (params_.clearance_weight <= 0.0 || clearance_.empty()) {return 0.0;}
  // 격자 간격으로 짚으며 groundEdge 와 같은 사다리꼴 규칙으로 더한다
  const double d = std::hypot(bx - ax, by - ay);
  const double res = costmap_->getResolution();
  const int n = std::max(1, static_cast<int>(std::lround(d / res)));
  const double step = d / n;
  unsigned int mx = 0, my = 0;
  const auto at = [&](double x, double y) {
      return costmap_->worldToMap(x, y, mx, my) ? clearancePenaltyPerMeter(mx, my) : 0.0;
    };
  double total = 0.0;
  double prev = at(ax, ay);
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double cur = at(ax + (bx - ax) * t, ay + (by - ay) * t);
    total += 0.5 * (prev + cur) * step;
    prev = cur;
  }
  return total;
}


CostAccumulator ProblemSpec::groundEdge(
  unsigned int mx, unsigned int my,
  unsigned int nx, unsigned int ny, double dist) const
{
  // 올라가는 스텝에만 등반 비용이 붙는다 (roverClimbMove 주석 참고).
  CostAccumulator acc = allowClimb() ?
    model_->roverClimbMove(dist, terrain(nx, ny) - terrain(mx, my)) :
    model_->groundMove(dist);
  // 경로 여유 — 두 칸 값의 평균 × 거리 (사다리꼴). 꺼져 있으면 0 이라 예전과 같다.
  acc.clearance_penalty =
    0.5 * (clearancePenaltyPerMeter(mx, my) + clearancePenaltyPerMeter(nx, ny)) * dist;
  return acc;
}

void ProblemSpec::buildAirLevels()
{
  // 맵에 실제로 존재하는 지형 높이만 후보로 삼는다.
  // 그 위 flight_clearance 만큼이 그 장애물을 넘기 위한 최소 고도다.
  // 그보다 높이 날 이유가 없고, 낮게 날면 걸린다.
  //
  // CostmapTerrainSource 는 cost 를 4등급으로 읽으므로 후보가 많아야 3~4개다.
  // 상태공간이 그만큼만 늘어난다.
  std::set<double> heights;
  const unsigned int nx = costmap_->getSizeInCellsX();
  const unsigned int ny = costmap_->getSizeInCellsY();
  for (unsigned int my = 0; my < ny; ++my) {
    for (unsigned int mx = 0; mx < nx; ++mx) {
      const double h = terrain_->heightAt(mx, my);
      if (h > h_limit_) {continue;}      // 어차피 못 넘는 높이
      heights.insert(h);
    }
  }

  std::set<double> levels;
  for (const double h : heights) {
    const double z = h + params_.flight_clearance;
    if (z <= z_max_ + 1e-9) {levels.insert(z);}
  }
  // 최소 하나는 있어야 이륙이 가능하다 (평지 위 비행)
  if (levels.empty()) {
    levels.insert(std::min(params_.flight_clearance, z_max_));
  }

  air_levels_.assign(levels.begin(), levels.end());

  // level 을 uint8_t 로 담으므로 상한을 넘지 않는지 확인한다.
  // 등급 기반 지형에서는 실제로 몇 개뿐이라 걸릴 일이 없다.
  if (air_levels_.size() > 255) {
    air_levels_.resize(255);
  }
}

bool ProblemSpec::airOkAt(unsigned int mx, unsigned int my, uint8_t level) const
{
  if (!inBounds(mx, my)) {return false;}
  if (level >= air_levels_.size()) {return false;}
  const double z = air_levels_[level];
  const double h = terrain(mx, my);
  return z >= h + model_->minFlightClearance() - 1e-9 && z <= z_max_ + 1e-9;
}

bool ProblemSpec::diagOkGround(unsigned int mx, unsigned int my, int dx, int dy) const
{
  if (dx == 0 || dy == 0) {return true;}
  return groundOk(mx + dx, my) && groundOk(mx, my + dy);
}

bool ProblemSpec::diagOkAir(
  unsigned int mx, unsigned int my, int dx, int dy, uint8_t level) const
{
  if (dx == 0 || dy == 0) {return true;}
  return airOkAt(mx + dx, my, level) && airOkAt(mx, my + dy, level);
}


void ProblemSpec::neighbors(const State & s, std::vector<Edge> & out) const
{
  out.clear();
  const double res = costmap_->getResolution();
  const Neighbor * nb = params_.allow_diagonal ? kNeighbors8 : kNeighbors4;
  const size_t n_nb = params_.allow_diagonal ? 8 : 4;

  if (s.mode == GROUND) {
    // 1) 지상 이동
    for (size_t i = 0; i < n_nb; ++i) {
      const int nx = static_cast<int>(s.mx) + nb[i].dx;
      const int ny = static_cast<int>(s.my) + nb[i].dy;
      if (nx < 0 || ny < 0) {continue;}
      const auto ux = static_cast<unsigned int>(nx);
      const auto uy = static_cast<unsigned int>(ny);
      if (!groundOk(ux, uy)) {continue;}
      if (params_.check_diagonal_corners &&
        !diagOkGround(s.mx, s.my, nb[i].dx, nb[i].dy))
      {
        continue;
      }
      out.push_back(
        {{ux, uy, GROUND, 0}, groundEdge(s.mx, s.my, ux, uy, nb[i].k * res)});
    }
    // 2) 이륙 — 어느 고도로 뜰지 여기서 정한다 (hybrid modal 에서만).
    //    3D 에서 고도를 정하는 유일한 순간이다. 이후 비행 구간 내내 고정된다.
    if (allowFly()) {
      for (size_t lv = 0; lv < air_levels_.size(); ++lv) {
        const auto level = static_cast<uint8_t>(lv);
        if (!airOkAt(s.mx, s.my, level)) {continue;}
        out.push_back({{s.mx, s.my, AIR, level, 0, 0.0f}, model_->takeoff(air_levels_[lv])});
      }
    }

  } else {
    // 3) 공중 이동 — 고도가 고정이므로 수직 비용도, 상승각 검사도 없다.
    //
    //    고도를 바꾸려면 착륙 후 다시 이륙해야 한다.
    //    그 비용은 takeoff/landing 으로 정상 계상되므로 공짜가 아니다.
    const double z = air_levels_[s.level];
    for (size_t i = 0; i < n_nb; ++i) {
      const int nx = static_cast<int>(s.mx) + nb[i].dx;
      const int ny = static_cast<int>(s.my) + nb[i].dy;
      if (nx < 0 || ny < 0) {continue;}
      const auto ux = static_cast<unsigned int>(nx);
      const auto uy = static_cast<unsigned int>(ny);
      if (!airOkAt(ux, uy, s.level)) {continue;}
      if (params_.check_diagonal_corners &&
        !diagOkAir(s.mx, s.my, nb[i].dx, nb[i].dy, s.level))
      {
        continue;
      }

      // 비행 한 구간 거리 제약 (max_flight_segment). 정확한 누적 거리로 건다.
      const double step = nb[i].k * res;
      const double flown = static_cast<double>(s.flown) + step;
      uint8_t fbin = 0;
      if (limitsFlight()) {
        // 1 mm 여유: mode_manager 는 착륙점까지의 거리를 좌표로 다시 재서
        // dist > 한계 로 검사한다. 칸 중심 좌표의 부동소수 오차로 정확히 5.00 m
        // 비행이 '초과' 로 찍힌 것을 확인했다.
        if (flown > params_.max_flight_distance - 1e-3) {continue;}
        fbin = static_cast<uint8_t>(std::min<double>(
            n_fbins_ - 1, std::floor(flown / params_.flight_distance_bin)));
      }

      // clearance 는 보수적으로 더 낮은 쪽을 쓴다
      const double clr = std::min(z - terrain(s.mx, s.my), z - terrain(ux, uy));

      out.push_back(
        {{ux, uy, AIR, s.level, fbin, static_cast<float>(flown)},
          model_->airMoveHorizontal(step, clr)});
    }
    // 4) 착륙 — 로버가 설 수 있고, 센서가 본 셀에만 (landingOk 주석 참고)
    if (landingOk(s.mx, s.my)) {
      out.push_back({{s.mx, s.my, GROUND, 0}, model_->landing(z)});
    }
  }
}


double ProblemSpec::heuristic(const State & s, const State & goal) const
{
  // 8방향 이동이므로 옥타일 거리 (유클리드보다 타이트하면서 admissible)
  const double dx = std::fabs(static_cast<double>(s.mx) - static_cast<double>(goal.mx));
  const double dy = std::fabs(static_cast<double>(s.my) - static_cast<double>(goal.my));
  const double d_cells = params_.allow_diagonal ?
    (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy) :
    (dx + dy);
  return unit_cost_min_ * d_cells * costmap_->getResolution();
}

}  // namespace drobot_hybrid_planner
