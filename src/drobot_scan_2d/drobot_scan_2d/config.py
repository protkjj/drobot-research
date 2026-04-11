"""
Configuration for Drobot Navigation VER1.

Clean, organized parameters for goal-based autonomous navigation.
"""


class Config:
    """Navigation configuration constants."""

    # ==================== Safety ====================
    COLLISION_STOP_DIST = 0.25      # m - emergency stop distance
    COLLISION_WARN_DIST = 0.45      # m - slow down distance
    SAFETY_MARGIN = 0.20            # m - minimum distance from obstacles

    # ==================== Navigation ====================
    GOAL_TOLERANCE_XY = 0.25        # m - goal reached threshold
    GOAL_TOLERANCE_YAW = 0.25       # rad - orientation tolerance
    NAVIGATION_TIMEOUT = 180.0      # sec - max time per goal (60→180)
    PROGRESS_CHECK_TIME = 15.0      # sec - check movement progress
    MIN_PROGRESS_DISTANCE = 0.3     # m - minimum movement required

    # ==================== Rule Engine ====================
    RULES_FILE = "rules.yaml"       # rules definition file

    # Speed reduction factors
    SPEED_REDUCTION_NARROW = 0.5    # 50% speed in narrow passages
    SPEED_REDUCTION_NEAR_OBSTACLE = 0.7  # 70% speed near obstacles

    # Stop conditions
    OBSTACLE_STOP_TIME = 3.0        # sec - wait time when obstacle detected

    # ==================== Timers ====================
    NAVIGATION_LOOP_RATE = 10.0     # Hz - main loop frequency
    RULE_CHECK_RATE = 5.0           # Hz - rule evaluation frequency
    COLLISION_CHECK_RATE = 20.0     # Hz - collision detection frequency
