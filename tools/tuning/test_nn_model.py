#!/usr/bin/env python3
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ast
import re

class FluxModel:
  # dict used to rename activation functions whose names aren't valid python identifiers
  activation_function_names = {'σ': 'sigmoid'}
  def __init__(self, params_file, zero_bias=True):
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

  # Begin activation functions.
  # These are called by name using the keys in the model json file
  def sigmoid(self, x):
    return 1 / (1 + np.exp(-x))
    
  def tanh(self, x):
    return np.tanh(x)

  def sigmoid_fast(self, x):
    return 0.5 * (x / (1 + np.abs(x)) + 1)
    # return x / (1 + np.abs(x))

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
    input_array = np.array(input_array, dtype=np.float32)#.reshape(1, -1)

    if input_array.shape[0] != self.input_size:
      raise ValueError(f"Input array last dimension {input_array.shape[-1]} does not match the expected length {self.input_size}")
    # Rescale the input array using the input_mean and input_std
    input_array = (input_array - self.input_mean) / self.input_std

    output_array = self.forward(input_array)

    return float(output_array[0, 0])
  
  def test(self, test_data: dict) -> str:
    num_passed = 0
    num_failed = 0
    allowed_chars = r'^[-\d.,\[\] ]+$'

    for input_str, expected_output in test_data.items():
      if not re.match(allowed_chars, input_str):
        raise ValueError(f"Invalid characters in NN FF model testing input string: {input_str}")

      input_list = ast.literal_eval(input_str)
      model_output = self.evaluate(input_list)

      if abs(model_output - expected_output) <= 1e-6:
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
  in_data = in_data[np.abs(in_data["v_ego"] - 20) < 1]
  model = FluxModel("/Users/haiiro/NoSync/voltlat.json", zero_bias=False)
  model.summary(do_print=True)
  print(f"{model.evaluate([25, 0.5, 0.1, 0.05]) = }")
  # print(in_data.head())
  
  # print(vars(model))
  
  plt.scatter(in_data["lateral_accel"], in_data["steer_cmd"], s=10, edgecolors='white', linewidths=1)
  x = np.arange(-4, 4, 0.1)
  # print(x)
  # print(y)
  y = np.array([model.evaluate([20, xi, 0, 0]) for xi in x])
  plt.plot(x, y, label='model', color='black', linewidth=2)
  y = np.array([model.evaluate([20, xi, 0, 0.07]) for xi in x])
  plt.plot(x, y, label='model w/ +glat', color='black', linewidth=2, alpha=0.5)
  y = np.array([model.evaluate([20, xi, 0, -0.07]) for xi in x])
  plt.plot(x, y, label='model w/ -glat', color='black', linewidth=2, linestyle='--', alpha=0.5)
  y = np.array([model.evaluate([20, xi, 2, 0]) for xi in x])
  plt.plot(x, y, label='model w/ +jerk', color='red', linewidth=2)
  y = np.array([model.evaluate([20, xi, 2, 0.07]) for xi in x])
  plt.plot(x, y, label='model w/ +jerk +glat', color='red', linewidth=2, alpha=0.5)
  y = np.array([model.evaluate([20, xi, 2, -0.07]) for xi in x])
  plt.plot(x, y, label='model w/ +jerk -glat', color='red', linewidth=2, linestyle='--', alpha=0.5)
  y = np.array([model.evaluate([20, xi, -2, 0]) for xi in x])
  plt.plot(x, y, label='model w/ -jerk', color='blue', linewidth=2)
  y = np.array([model.evaluate([20, xi, -2, 0.07]) for xi in x])
  plt.plot(x, y, label='model w/ -jerk +glat', color='blue', linewidth=2, alpha=0.5)
  y = np.array([model.evaluate([20, xi, -2, -0.07]) for xi in x])
  plt.plot(x, y, label='model w/ -jerk -glat', color='blue', linewidth=2, linestyle='--', alpha=0.5)
  plt.xlabel('lat accel [m/s^2]')
  plt.ylabel('steer cmd 3Nm')
  plt.grid(True)
  plt.legend()

  # show the plot
  plt.show()
  return

if __name__ == "__main__":
  main()