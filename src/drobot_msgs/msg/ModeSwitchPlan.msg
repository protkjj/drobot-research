# 경로 전체의 모드 전환 계획
# 플래너가 nav_msgs/Path와 함께 퍼블리시
# 토픽: /mode_switch_points

std_msgs/Header header

# 전환 지점 배열 — 경로 순서대로 정렬
ModeSwitchPoint[] switch_points

# 전체 비행 구간의 예상 에너지 합 (Wh)
# BT에서 경로 전체 비행 감당 가능한 배터리가 있는지 한 번에 체크
float64 total_flight_energy
