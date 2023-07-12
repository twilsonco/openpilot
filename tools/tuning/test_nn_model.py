#!/usr/bin/env python3
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ast
import re


def sign(x):
  if x > 0.0:
    return 1.0
  elif x < 0.0:
    return -1.0
  else:
    return 0.0

class FluxModel:
  # dict used to rename activation functions whose names aren't valid python identifiers
  activation_function_names = {'σ': 'sigmoid'}
  def __init__(self, params_file, zero_bias=False):
    with open(params_file, "r") as f:
      params = json.load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]
    self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
    self.input_std = np.array(params["input_std"], dtype=np.float32).T
    test_dict = params["test_dict_zero_bias"] if zero_bias else params["test_dict"]
    self.layers = []

    for layer_params in params["layers"]:
      W = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_W'))], dtype=np.float32).T
      b = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_b'))], dtype=np.float32).T
      if zero_bias:
        b = np.zeros_like(b)
      activation = layer_params["activation"]
      for k, v in self.activation_function_names.items():
        activation = activation.replace(k, v)
      self.layers.append((W, b, activation))
      
    self.test(test_dict)
    if not self.test_passed:
      raise ValueError(f"NN FF model failed test: {params_file}")

    # Compute the mean slope of the nnff from 0 to 3m/s^2 so
    # that kf can be estimated in torqued from lateral accel.
    # Do it for a range of speeds and then we can interpolate between them.
    mean_slope_lat_accel_limits = (1.5, 2.5)
    mean_slope_speeds = [float(i) for i in np.arange(15.0, 25.0, 2.0)]
    mean_slope_lat_accels = np.arange(mean_slope_lat_accel_limits[0], mean_slope_lat_accel_limits[1], 0.1)
    mean_slopes = [float(np.mean([self.evaluate([s, la])/la for la in mean_slope_lat_accels])) for s in mean_slope_speeds]
    # This can be compared to the torqued slope to compute
    # the kf value.
    self.mean_mean_slope = float(np.mean(mean_slopes))
    self.lat_accel_factor = 1/self.mean_mean_slope
    
  # Begin activation functions.
  # These are called by name using the keys in the model json file
  def sigmoid(self, x):
    return 1 / (1 + np.exp(-x))

  def identity(self, x):
    return x
  # End activation functions

  def forward(self, x):
    for W, b, activation in self.layers:
      if hasattr(self, activation):
        x = getattr(self, activation)(x.dot(W) + b)
      else:
        raise ValueError(f"Unknown activation: {activation}")
    return x

  def evaluate(self, input_array):
    if len(input_array) != self.input_size:
      # If the input is length two, then it's a simplified evaluation with only speed and lateral acceleration.
      # In that case, need to add on zeros to fill out the input array to match the correct length.
      if len(input_array) == 2:
        input_array = input_array + [0] * (self.input_size - 2)
      else:
        raise ValueError(f"Input array length {len(input_array)} does not match the expected length {self.input_size}")
        
    input_array = np.array(input_array, dtype=np.float32)#.reshape(1, -1)

    # Rescale the input array using the input_mean and input_std
    input_array = (input_array - self.input_mean) / self.input_std

    output_array = self.forward(input_array)

    return float(output_array[0, 0])

  # torqued uses a linear fit of steer command vs lateral accel.
  # NNFF is non-linear for many cars, so we need to provide a way
  # to "linearize" the recorded steer command so the linear fit
  # still works.
  def get_steer_cmd_scale(self, speed, lat_accel):
    abs_lat_accel = max(0.05, abs(lat_accel))
    comp_lat_accel = self.lat_accel_factor
    comp_slope = self.evaluate([speed, comp_lat_accel]) / comp_lat_accel
    current_value = self.evaluate([speed, abs_lat_accel])
    linear_value = abs_lat_accel * comp_slope
    return abs(abs_lat_accel - 1.2 * abs(linear_value - current_value)) / abs_lat_accel
    
  def test(self, test_data: dict):
    num_passed = 0
    num_failed = 0
    allowed_chars = r'^[-\d.,\[\] ]+$'
    self.test_passed = False

    for input_str, expected_output in test_data.items():
      if not re.match(allowed_chars, input_str):
        raise ValueError(f"Invalid characters in NN FF model testing input string: {input_str}")

      input_list = ast.literal_eval(input_str)
      model_output = self.evaluate(input_list)

      if abs(model_output - expected_output) <= 5e-5:
        num_passed += 1
      else:
        num_failed += 1
        raise ValueError(f"NN FF model failed test at value {input_list}: expected {expected_output}, got {model_output}")

    summary_str = (
      f"Test results: PASSED ({num_passed} inputs tested) "
    )
    
    self.test_passed = num_failed == 0
    self.test_str = summary_str

  def summary(self, do_print=True):
    summary_lines = [
      "FluxModel Summary:",
      f"Input size: {self.input_size}",
      f"Output size: {self.output_size}",
      f"Number of layers: {len(self.layers)}",
      self.test_str,
      "Layer details:"
    ]

    for i, (W, b, activation) in enumerate(self.layers):
      summary_lines.append(
          f"  Layer {i + 1}: W: {W.shape}, b: {b.shape}, f: {activation}"
      )
    
    summary_str = "\n".join(summary_lines)

    if do_print:
      print(summary_str)

    return summary_str
  
  

def main():
  in_data = pd.read_csv("/Users/haiiro/NoSync/voltlat_large_balanced.csv")
  s = 10
  in_data = in_data[np.abs(in_data["v_ego"] - s) < 1]
  model = FluxModel("/Volumes/video/scratch-video/latmodels/CHEVROLET VOLT PREMIER 2017/CHEVROLET VOLT PREMIER 2017.json", zero_bias=False)
  model.summary(do_print=False)
  # print(f"{model.evaluate([25, 0.5, 0.1, 0.05] + [0] * (model.input_size-4)) = }")
  # print(in_data.head())
  
  
  print(f"{model.lat_accel_factor = }")
  
  # test model.get_slope_offset(speed, lat accel)
  # for a range of speeds and positive lat accels, in
  # a table with heading rows and columns.
  print("speed \ lat accel: " + ', '.join([f"{la:0.3f}" for la in np.arange(-1.0, 1.1, 0.2)]))
  for s in range(3, 25, 3):
    print(f"{s}: " + ', '.join([f"{model.get_steer_cmd_scale(s, la):.3f}" for la in np.arange(-1.0, 1.1, 0.2)]))
  
  # print(vars(model))
  
  # plt.scatter(in_data["lateral_accel"], in_data["steer_cmd"], s=10, edgecolors='white', linewidths=1)
  # x = np.arange(-4.2, 4.2, 0.1)
  # # print(x)
  # # print(y)
  # y = np.array([model.evaluate([s, xi, 0, 0] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model', color='black', linewidth=2)
  # y = np.array([model.evaluate([s, xi, 0, 0.07] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ +glat', color='black', linewidth=2, alpha=0.5)
  # y = np.array([model.evaluate([s, xi, 0, -0.07] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ -glat', color='black', linewidth=2, linestyle='--', alpha=0.5)
  # y = np.array([model.evaluate([s, xi, 2, 0] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ +jerk', color='red', linewidth=2)
  # y = np.array([model.evaluate([s, xi, 2, 0.07] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ +jerk +glat', color='red', linewidth=2, alpha=0.5)
  # y = np.array([model.evaluate([s, xi, 2, -0.07] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ +jerk -glat', color='red', linewidth=2, linestyle='--', alpha=0.5)
  # y = np.array([model.evaluate([s, xi, -2, 0] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ -jerk', color='blue', linewidth=2)
  # y = np.array([model.evaluate([s, xi, -2, 0.07] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ -jerk +glat', color='blue', linewidth=2, alpha=0.5)
  # y = np.array([model.evaluate([s, xi, -2, -0.07] + [0] * (model.input_size-4)) for xi in x])
  # plt.plot(x, y, label='model w/ -jerk -glat', color='blue', linewidth=2, linestyle='--', alpha=0.5)
  # plt.xlabel('lat accel [m/s^2]')
  # plt.ylabel('steer cmd 3Nm')
  # plt.grid(True)
  # plt.legend()

  # # show the plot
  # plt.show()
  return

if __name__ == "__main__":
  main()