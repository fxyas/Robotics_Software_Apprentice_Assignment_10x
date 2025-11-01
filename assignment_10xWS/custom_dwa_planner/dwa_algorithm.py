"""Dynamic Window Approach (DWA) Algorithm Implementation.
This module implements the core DWA algorithm for local path planning."""

import numpy as np
from typing import Tuple, List
import math

class DWAConfig:
    """Configuration parameters for DWA algorithm."""
    
    def __init__(self):
        # Robot specifications
        self.max_speed = 0.22  
        self.min_speed = 0.0  # Forward only
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # rad
        self.max_accel = 0.2  
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  
        
        # Velocity resolution
        self.v_resolution = 0.02  
        self.yaw_rate_resolution = 0.2 * math.pi / 180.0  
        
        # Simulation parameters
        self.dt = 0.1  #  Time tick for motion prediction
        self.predict_time = 2.0  
        
        # Cost function weights
        self.to_goal_cost_gain = 3.0  # Prioritize goal reaching
        self.speed_cost_gain = 1.0  # Encourage higher speeds
        self.obstacle_cost_gain = 1.5  # Balanced obstacle avoidance (increased from 0.5)
        
        # Robot radius for collision checking
        self.robot_radius = 0.20  # TurtleBot3 burger is ~0.178m
        
        # Obstacle distance threshold
        self.obstacle_threshold = 1.2  # m Penalize closer obstacles


class DWAPlanner:
    """Dynamic Window Approach local planner."""
    
    def __init__(self, config: DWAConfig = None):

        self.config = config if config else DWAConfig()
        
    def plan(self, x: np.ndarray, goal: np.ndarray, obstacles: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        
        # Calculate dynamic window
        dw = self._calc_dynamic_window(x)
        
        # Evaluate all velocity samples in dynamic window
        best_u, best_trajectory = self._evaluate_trajectories(x, dw, goal, obstacles)
        
        return best_u, best_trajectory
    
    def _calc_dynamic_window(self, x: np.ndarray) -> np.ndarray:
        """
        Calculate dynamic window based on current velocity and acceleration limits.
        
        Args: x: Current state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            
        Returns: Dynamic window [v_min, v_max, yaw_rate_min, yaw_rate_max]
        """
        # Dynamic window from robot specification
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        # Dynamic window from motion model (acceleration limits)
        Vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_delta_yaw_rate * self.config.dt,
              x[4] + self.config.max_delta_yaw_rate * self.config.dt]
        
        # Intersection of Vs and Vd
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return np.array(dw)
    
    def _evaluate_trajectories(self, x: np.ndarray, dw: np.ndarray, 
                               goal: np.ndarray, obstacles: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:

        best_u = np.array([0.1, 0.0])  # Default to small forward velocity
        min_cost = float('inf')
        best_trajectory = np.array([x])
        
        # Sample velocities within dynamic window
        for v in np.arange(max(dw[0], 0.05), dw[1], self.config.v_resolution):  # Ensure minimum velocity
            for omega in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                # Predict trajectory
                trajectory = self._predict_trajectory(x, v, omega)
                
                # Calculate cost
                to_goal_cost = self._calc_to_goal_cost(trajectory, goal)
                speed_cost = self._calc_speed_cost(v)
                obstacle_cost = self._calc_obstacle_cost(trajectory, obstacles)
                
                # Check if trajectory is valid (no collision)
                if obstacle_cost == float('inf'):
                    continue
                
                # Total cost
                total_cost = (self.config.to_goal_cost_gain * to_goal_cost +
                            self.config.speed_cost_gain * speed_cost +
                            self.config.obstacle_cost_gain * obstacle_cost)
                
                # Update best trajectory
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = np.array([v, omega])
                    best_trajectory = trajectory
        
        return best_u, best_trajectory
    
    def _predict_trajectory(self, x: np.ndarray, v: float, omega: float) -> np.ndarray:
        """
        Predict trajectory for given velocity command.
        
        Args: x: Current state
            v: Linear velocity
            omega: Angular velocity
            
        Returns:
            Predicted trajectory as array of states
        """
        trajectory = [x.copy()]
        time = 0.0
        current_x = x.copy()
        
        while time <= self.config.predict_time:
            current_x = self._motion_model(current_x, v, omega)
            trajectory.append(current_x.copy())
            time += self.config.dt
        
        return np.array(trajectory)
    
    def _motion_model(self, x: np.ndarray, v: float, omega: float) -> np.ndarray:
        """ Motion model for differential drive robot.
        Args:
            x: Current state [x, y, yaw, v, omega]
            v: Linear velocity command
            omega: Angular velocity command
        Returns:
            Next state
        """
        x_next = x.copy()
        x_next[2] += omega * self.config.dt  # yaw
        x_next[0] += v * math.cos(x_next[2]) * self.config.dt  # x
        x_next[1] += v * math.sin(x_next[2]) * self.config.dt  # y
        x_next[3] = v  # v
        x_next[4] = omega  # omega
        
        return x_next
    
    def _calc_to_goal_cost(self, trajectory: np.ndarray, goal: np.ndarray) -> float:
        """ Calculate cost based on distance to goal.
        Args:
            trajectory: Predicted trajectory
            goal: Goal position
            
        Returns:
            Cost value (lower is better)
        """
        # Use the end point of trajectory
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        
        return cost
    
    def _calc_speed_cost(self, v: float) -> float:
        """
        Calculate cost based on speed (encourage faster movement)
        
        Args:
            v: Linear velocity
            
        Returns:
            Cost value (lower is better)
        """
        return self.config.max_speed - v
    
    def _calc_obstacle_cost(self, trajectory: np.ndarray, obstacles: np.ndarray) -> float:
        """
        Calculate cost based on proximity to obstacles.
        
        Args:
            trajectory: Predicted trajectory
            obstacles: Obstacle points
            
        Returns:
            Cost value (lower is better), inf if collision
        """
        if len(obstacles) == 0:
            return 0.0
        
        min_distance = float('inf')
        
        # Check each point in trajectory (sample every other point for speed)
        for state in trajectory[::2]:
            for obstacle in obstacles:
                dx = state[0] - obstacle[0]
                dy = state[1] - obstacle[1]
                distance = math.sqrt(dx**2 + dy**2)
                
                # Reject if collision with proper safety margin
                if distance <= self.config.robot_radius:
                    return float('inf')
                
                min_distance = min(min_distance, distance)
        
        # Return inverse distance as cost (closer = higher cost)
        if min_distance < self.config.obstacle_threshold:
            return 1.0 / min_distance  # Proper penalty to avoid obstacles
        else:
            return 0.0
