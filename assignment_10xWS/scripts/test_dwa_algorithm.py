"""
for test 
"""

import sys
import os
import numpy as np
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from custom_dwa_planner.dwa_algorithm import DWAPlanner, DWAConfig


def test_dwa_basic():
    """ basic DWA functionality."""
    print("=" * 60)
    print("Testing DWA Algorithm")
    print("=" * 60)
    
    # Create DWA planner
    config = DWAConfig()
    planner = DWAPlanner(config)
    
    # Test case 1: Simple navigation to goal
    print("\nTest 1: Simple navigation to goal")
    print("-" * 60)
    
    # Robot state: [x, y, yaw, v, omega]
    robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([5.0, 5.0])
    obstacles = np.array([])  # No obstacles
    
    print(f"Robot state: {robot_state}")
    print(f"Goal: {goal}")
    print(f"Obstacles: None")
    
    best_u, best_trajectory = planner.plan(robot_state, goal, obstacles)
    
    print(f"\nOptimal velocity command:")
    print(f"  Linear velocity (v): {best_u[0]:.3f} m/s")
    print(f"  Angular velocity (ω): {best_u[1]:.3f} rad/s")
    print(f"  Trajectory length: {len(best_trajectory)} points")
    print(f"  Final position: ({best_trajectory[-1, 0]:.2f}, {best_trajectory[-1, 1]:.2f})")
    
    # Test case 2: Navigation with obstacles
    print("\n" + "=" * 60)
    print("Test 2: Navigation with obstacles")
    print("-" * 60)
    
    robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([5.0, 0.0])
    obstacles = np.array([
        [2.5, 0.5],
        [2.5, 0.3],
        [2.5, 0.1],
        [2.5, -0.1],
        [2.5, -0.3],
        [2.5, -0.5],
    ])
    
    print(f"Robot state: {robot_state}")
    print(f"Goal: {goal}")
    print(f"Obstacles: {len(obstacles)} points")
    
    best_u, best_trajectory = planner.plan(robot_state, goal, obstacles)
    
    print(f"\nOptimal velocity command:")
    print(f"  Linear velocity (v): {best_u[0]:.3f} m/s")
    print(f"  Angular velocity (ω): {best_u[1]:.3f} rad/s")
    print(f"  Trajectory length: {len(best_trajectory)} points")
    print(f"  Final position: ({best_trajectory[-1, 0]:.2f}, {best_trajectory[-1, 1]:.2f})")
    
    # Test case 3: Robot already at goal
    print("\n" + "=" * 60)
    print("Test 3: Robot near goal")
    print("-" * 60)
    
    robot_state = np.array([5.0, 5.0, 0.0, 0.1, 0.0])
    goal = np.array([5.0, 5.0])
    obstacles = np.array([])
    
    print(f"Robot state: {robot_state}")
    print(f"Goal: {goal}")
    
    distance_to_goal = math.sqrt(
        (goal[0] - robot_state[0])**2 + 
        (goal[1] - robot_state[1])**2
    )
    
    print(f"Distance to goal: {distance_to_goal:.3f} m")
    
    if distance_to_goal < 0.3:
        print("Robot is at goal! Should stop.")
    
    # Test case 4: Dynamic window calculation
    print("\n" + "=" * 60)
    print("Test 4: Dynamic window calculation")
    print("-" * 60)
    
    robot_state = np.array([0.0, 0.0, 0.0, 0.15, 0.2])
    dw = planner._calc_dynamic_window(robot_state)
    
    print(f"Current velocity: v={robot_state[3]:.2f} m/s, ω={robot_state[4]:.2f} rad/s")
    print(f"Dynamic window:")
    print(f"  Linear velocity range: [{dw[0]:.3f}, {dw[1]:.3f}] m/s")
    print(f"  Angular velocity range: [{dw[2]:.3f}, {dw[3]:.3f}] rad/s")
    
    # Test case 5: Trajectory prediction
    print("\n" + "=" * 60)
    print("Test 5: Trajectory prediction")
    print("-" * 60)
    
    robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    v = 0.2
    omega = 0.3
    
    trajectory = planner._predict_trajectory(robot_state, v, omega)
    
    print(f"Predicting trajectory with v={v} m/s, ω={omega} rad/s")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Start position: ({trajectory[0, 0]:.2f}, {trajectory[0, 1]:.2f})")
    print(f"End position: ({trajectory[-1, 0]:.2f}, {trajectory[-1, 1]:.2f})")
    print(f"End orientation: {trajectory[-1, 2]:.2f} rad ({math.degrees(trajectory[-1, 2]):.1f}°)")
    
    print("\n" + "=" * 60)
    print("All tests completed successfully!")
    print("=" * 60)


if __name__ == '__main__':
    test_dwa_basic()
