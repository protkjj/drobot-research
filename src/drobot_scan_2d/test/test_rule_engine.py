#!/usr/bin/env python3
"""Unit tests for RuleEngine."""
import os
import tempfile
import pytest
import yaml

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from drobot_scan_2d.rules.engine import RuleEngine, RuleAction


# ==================== Fixtures ====================

@pytest.fixture
def sample_rules():
    """Sample rules dict for testing."""
    return {
        'forbidden_zones': [
            {
                'name': 'danger_zone',
                'type': 'polygon',
                'points': [[0, 0], [2, 0], [2, 2], [0, 2]],
            }
        ],
        'speed_limit_zones': [
            {
                'name': 'slow_zone',
                'type': 'polygon',
                'points': [[3, 3], [5, 3], [5, 5], [3, 5]],
                'max_speed': 0.1,
            }
        ],
        'stop_rules': [
            {
                'name': 'emergency_stop',
                'trigger': 'collision_imminent',
                'distance': 0.3,
                'wait_time': 0.0,
                'action': 'immediate_stop',
            },
            {
                'name': 'obstacle_detected',
                'trigger': 'obstacle_in_path',
                'distance': 0.5,
                'wait_time': 3.0,
                'action': 'wait_and_retry',
            },
        ],
        'path_preferences': {
            'prefer_wide_paths': True,
            'wall_clearance': 0.4,
        },
        'general': {
            'max_speed': 0.22,
        },
    }


@pytest.fixture
def rules_file(sample_rules):
    """Create a temporary rules YAML file."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(sample_rules, f)
        path = f.name
    yield path
    os.unlink(path)


@pytest.fixture
def engine(rules_file):
    """RuleEngine loaded with sample rules."""
    return RuleEngine(rules_file)


# ==================== Tests ====================

class TestRuleEngineInit:
    def test_init_without_file(self):
        engine = RuleEngine()
        assert engine.forbidden_zones == []
        assert engine.stop_rules == []

    def test_init_with_nonexistent_file(self):
        engine = RuleEngine('/nonexistent/path.yaml')
        assert engine.forbidden_zones == []

    def test_init_with_valid_file(self, engine):
        assert len(engine.forbidden_zones) == 1
        assert len(engine.stop_rules) == 2
        assert len(engine.speed_limit_zones) == 1


class TestForbiddenZone:
    def test_point_inside(self, engine):
        is_forbidden, name = engine.check_forbidden_zone(1.0, 1.0)
        assert is_forbidden is True
        assert name == 'danger_zone'

    def test_point_outside(self, engine):
        is_forbidden, name = engine.check_forbidden_zone(5.0, 5.0)
        assert is_forbidden is False
        assert name is None

    def test_point_on_edge(self, engine):
        # Edge behavior depends on ray casting implementation
        is_forbidden, _ = engine.check_forbidden_zone(0.0, 1.0)
        # Just check it doesn't crash
        assert isinstance(is_forbidden, bool)


class TestSpeedLimit:
    def test_inside_slow_zone(self, engine):
        speed = engine.check_speed_limit(4.0, 4.0)
        assert speed == 0.1

    def test_outside_slow_zone(self, engine):
        speed = engine.check_speed_limit(0.0, 0.0)
        assert speed == 0.22

    def test_general_max_speed(self, engine):
        assert engine.get_max_speed() == 0.22


class TestStopRules:
    def test_collision_imminent(self, engine):
        action = engine.evaluate_stop_rules(
            obstacle_distance=0.2, obstacle_detected=True
        )
        assert action.action_type == 'stop'
        assert 'Emergency' in action.message

    def test_obstacle_in_path(self, engine):
        action = engine.evaluate_stop_rules(
            obstacle_distance=0.4, obstacle_detected=True
        )
        assert action.action_type == 'wait_and_retry'
        assert action.wait_time == 3.0

    def test_no_obstacle(self, engine):
        action = engine.evaluate_stop_rules(
            obstacle_distance=5.0, obstacle_detected=False
        )
        assert action.action_type == 'continue'


class TestPointInPolygon:
    def test_square(self, engine):
        square = [[0, 0], [4, 0], [4, 4], [0, 4]]
        assert engine._point_in_polygon(2, 2, square) is True
        assert engine._point_in_polygon(5, 5, square) is False

    def test_triangle(self, engine):
        triangle = [[0, 0], [4, 0], [2, 4]]
        assert engine._point_in_polygon(2, 1, triangle) is True
        assert engine._point_in_polygon(0, 4, triangle) is False

    def test_empty_polygon(self, engine):
        assert engine._point_in_polygon(0, 0, []) is False

    def test_too_few_points(self, engine):
        assert engine._point_in_polygon(0, 0, [[0, 0], [1, 1]]) is False


class TestPathPreferences:
    def test_wall_clearance(self, engine):
        assert engine.get_wall_clearance() == 0.4

    def test_default_wall_clearance(self):
        engine = RuleEngine()
        assert engine.get_wall_clearance() == 0.3
