#!/Users/haiiro/.pyenv/shims/python3
##!/usr/bin/env python3
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ast
import re


FRICTION_FACTOR = 1.5  # ~85% of data coverage

def sign(x):
  if x > 0.0:
    return 1.0
  elif x < 0.0:
    return -1.0
  else:
    return 0.0
  
def slope2rot(slope):
  sin = np.sqrt(slope**2 / (slope**2 + 1))
  cos = np.sqrt(1 / (slope**2 + 1))
  return np.array([[cos, -sin], [sin, cos]])

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
    
    self.validate_layers()
    self.test(test_dict)
    if not self.test_passed:
      raise ValueError(f"NN FF model failed test: {params_file}")
    
  # Begin activation functions.
  # These are called by name using the keys in the model json file
  def sigmoid(self, x):
    return 1 / (1 + np.exp(-x))

  def identity(self, x):
    return x
  # End activation functions

  def forward(self, x):
    for W, b, activation in self.layers:
      x = getattr(self, activation)(x.dot(W) + b)
    return x

  def evaluate(self, input_array):
    in_len = len(input_array)
    if in_len != self.input_size:
      # If the input is length 2-4, then it's a simplified evaluation.
      # In that case, need to add on zeros to fill out the input array to match the correct length.
      if 2 <= in_len <= 4:
        input_array = input_array + [0] * (self.input_size - in_len)
      else:
        raise ValueError(f"Input array length {len(input_array)} does not match the expected length {self.input_size}")
        
    input_array = np.array(input_array, dtype=np.float32)#.reshape(1, -1)

    # Rescale the input array using the input_mean and input_std
    input_array = (input_array - self.input_mean) / self.input_std

    output_array = self.forward(input_array)

    return float(output_array[0, 0])
  
  def validate_layers(self):
    for W, b, activation in self.layers:
      if hasattr(self, activation):
        continue
      else:
        raise ValueError(f"Unknown activation: {activation}")
    
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
  model = FluxModel("/Users/haiiro/NoSync/opsunny/openpilot/selfdrive/car/torque_data/lat_models/CHEVROLET VOLT PREMIER 2017.json", zero_bias=False)
  model.summary(do_print=False)
  # print(f"{model.evaluate([25, 0.5, 0.1, 0.05] + [0] * (model.input_size-4)) = }")
  # print(in_data.head())
  
  

  # # show the plot
  # plt.show()
  return

if __name__ == "__main__":
  main()