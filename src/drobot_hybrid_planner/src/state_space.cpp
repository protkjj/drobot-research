// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "drobot_hybrid_planner/state_space.hpp"

#include <algorithm>
#include <set>
#include <cmath>

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

  // 지상 주행 1m당 비용 (휴리스틱 하한용).
  // 비행은 지상보다 비싸므로 지상 단가로 잡으면 admissible 하다.
  unit_cost_ground_ = model_->cost(model_->groundMove(1.0));
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
  if (!allowClimb()) {
    return terrain_->roverTraversable(mx, my);
  }
  // 밟고넘기 modal 에서는 장애물 위(<= roverHLimit)도 '있을 수 있는 곳'이다.
  // 1e-9 여유는 0.70m 장애물이 부동소수 오차로 걸러지는 걸 막기 위함.
  const double h = terrain_->heightAt(mx, my);
  return h <= roverHLimit() + 1e-9;
}


CostAccumulator ProblemSpec::groundEdge(
  unsigned int mx, unsigned int my,
  unsigned int nx, unsigned int ny, double dist) const
{
  if (!allowClimb()) {
    return model_->groundMove(dist);
  }
  // 올라가는 스텝에만 등반 비용이 붙는다 (roverClimbMove 주석 참고).
  return model_->roverClimbMove(dist, terrain(nx, ny) - terrain(mx, my));
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
        out.push_back({{s.mx, s.my, AIR, level}, model_->takeoff(air_levels_[lv])});
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

      // clearance 는 보수적으로 더 낮은 쪽을 쓴다
      const double clr = std::min(z - terrain(s.mx, s.my), z - terrain(ux, uy));

      out.push_back(
        {{ux, uy, AIR, s.level}, model_->airMoveHorizontal(nb[i].k * res, clr)});
    }
    // 4) 착륙 — 로버가 설 수 있는 셀에만
    if (groundOk(s.mx, s.my)) {
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
  return unit_cost_ground_ * d_cells * costmap_->getResolution();
}

}  // namespace drobot_hybrid_planner
