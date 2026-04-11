# 단일 모드 전환 지점
# 플래너(Hybrid RRT*)가 생성, BT(CheckModeSwitch)가 소비

# 헤더 — timestamp + frame_id (보통 "map")
std_msgs/Header header

# 전환 위치 — x, y는 맵 좌표, z는 지면 높이
geometry_msgs/Point position

# 전환 방향
uint8 GROUND_TO_AIR=0
uint8 AIR_TO_GROUND=1
uint8 switch_type

# 비행 목표 고도 (m, 지면 기준)
# 플래너가 장애물 높이 + 0.8~1.0m 여유로 계산
float64 flight_altitude

# 이 전환의 예상 에너지 비용 (Wh)
# BT에서 배터리 잔량과 비교하여 비행 가능 여부 판단
float64 estimated_energy_cost

# 이륙-착륙 쌍 매칭용 ID
# 같은 pair_id를 가진 GROUND_TO_AIR + AIR_TO_GROUND가 한 비행 구간
uint32 pair_id
