#!/usr/bin/env python3

def additional_lateral_acceleration(longitudinal_speed, longitudinal_acceleration, curvature, time = 0.01):
  """
  Calculate the additional lateral acceleration due to longitudinal acceleration.
  
  Parameters:
  longitudinal_speed (float): The initial longitudinal speed of the vehicle (m/s).
  longitudinal_acceleration (float): The longitudinal acceleration of the vehicle (m/s^2).
  curvature (float): The curvature of the path (1/m).
  time (float): The time since acceleration started (s).
  
  Returns:
  float: The additional lateral acceleration (m/s^2).
  """
  
  initial_lateral_acceleration = longitudinal_speed**2 * curvature
  current_longitudinal_speed = longitudinal_speed + longitudinal_acceleration * time
  current_lateral_acceleration = current_longitudinal_speed**2 * curvature
  additional_lateral_acceleration = current_lateral_acceleration - initial_lateral_acceleration
  
  return additional_lateral_acceleration

def additional_lateral_acceleration1(longitudinal_acceleration, curvature):
  return longitudinal_acceleration * curvature

print(additional_lateral_acceleration(35.0, 3, 0.015, 0.0145))
print(additional_lateral_acceleration1(3, 0.015))