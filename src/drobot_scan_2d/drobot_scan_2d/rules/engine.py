"""
Rule Engine for Drobot Navigation VER1.

Evaluates driving rules and constraints during navigation.
"""
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import logging
import yaml
import os

# 로거 설정
logger = logging.getLogger(__name__)


@dataclass
class RuleAction:
    """Action to take based on rule evaluation."""
    action_type: str  # "continue", "slow_down", "stop", "reroute", "abort"
    speed_factor: float = 1.0  # Speed multiplier (1.0 = normal)
    wait_time: float = 0.0  # Time to wait (seconds)
    message: str = ""  # Log message


class RuleEngine:
    """Evaluates navigation rules and constraints."""

    def __init__(self, rules_file: Optional[str] = None):
        """
        Initialize rule engine.

        Args:
            rules_file: Path to rules.yaml file
        """
        self.rules: Dict = {}
        self.forbidden_zones: List = []
        self.speed_limit_zones: List = []
        self.stop_rules: List = []
        self.path_preferences: Dict = {}
        self.general_rules: Dict = {}

        if rules_file and os.path.exists(rules_file):
            self.load_rules(rules_file)

    def load_rules(self, rules_file: str) -> bool:
        """
        Load rules from YAML file.

        Args:
            rules_file: Path to rules.yaml

        Returns:
            True if loaded successfully
        """
        try:
            with open(rules_file, 'r') as f:
                self.rules = yaml.safe_load(f)

            self.forbidden_zones = self.rules.get('forbidden_zones', []) or []
            self.speed_limit_zones = self.rules.get('speed_limit_zones', []) or []
            self.stop_rules = self.rules.get('stop_rules', []) or []
            self.path_preferences = self.rules.get('path_preferences', {}) or {}
            self.general_rules = self.rules.get('general', {}) or {}

            return True
        except Exception as e:
            logger.error(f"Failed to load rules: {e}")
            return False

    def check_forbidden_zone(
        self,
        x: float,
        y: float
    ) -> Tuple[bool, Optional[str]]:
        """
        Check if position is in a forbidden zone.

        Args:
            x, y: Position to check

        Returns:
            Tuple of (is_forbidden, zone_name)
        """
        for zone in self.forbidden_zones:
            if self._point_in_polygon(x, y, zone.get('points', [])):
                return True, zone.get('name', 'Unknown')
        return False, None

    def check_speed_limit(
        self,
        x: float,
        y: float
    ) -> float:
        """
        Get speed limit at position.

        Args:
            x, y: Current position

        Returns:
            Maximum allowed speed (m/s)
        """
        max_speed = self.general_rules.get('max_speed', 0.22)

        for zone in self.speed_limit_zones:
            if self._point_in_polygon(x, y, zone.get('points', [])):
                zone_limit = zone.get('max_speed', max_speed)
                max_speed = min(max_speed, zone_limit)

        return max_speed

    def evaluate_stop_rules(
        self,
        obstacle_distance: float,
        obstacle_detected: bool
    ) -> RuleAction:
        """
        Evaluate stop rules based on current situation.

        Args:
            obstacle_distance: Distance to nearest obstacle
            obstacle_detected: Whether obstacle is in path

        Returns:
            RuleAction to take
        """
        for rule in self.stop_rules:
            trigger = rule.get('trigger', '')
            distance_threshold = rule.get('distance', 0.5)

            if trigger == 'collision_imminent' and obstacle_distance < distance_threshold:
                return RuleAction(
                    action_type='stop',
                    wait_time=rule.get('wait_time', 0.0),
                    message=f"Emergency stop: obstacle at {obstacle_distance:.2f}m"
                )

            if trigger == 'obstacle_in_path' and obstacle_detected:
                if obstacle_distance < distance_threshold:
                    action = rule.get('action', 'wait_and_retry')
                    return RuleAction(
                        action_type=action,
                        wait_time=rule.get('wait_time', 3.0),
                        message=f"Obstacle detected at {obstacle_distance:.2f}m, waiting..."
                    )

        return RuleAction(action_type='continue')

    def get_path_preferences(self) -> Dict:
        """Get path planning preferences."""
        return self.path_preferences

    def get_max_speed(self) -> float:
        """Get general maximum speed."""
        return self.general_rules.get('max_speed', 0.22)

    def get_wall_clearance(self) -> float:
        """Get preferred wall clearance distance."""
        return self.path_preferences.get('wall_clearance', 0.3)

    def _point_in_polygon(
        self,
        x: float,
        y: float,
        polygon: List[List[float]]
    ) -> bool:
        """
        Check if point is inside polygon (ray casting algorithm).

        Args:
            x, y: Point to check
            polygon: List of [x, y] vertices

        Returns:
            True if point is inside polygon
        """
        if not polygon or len(polygon) < 3:
            return False

        n = len(polygon)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i

        return inside
