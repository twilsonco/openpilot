#!/usr/bin/env python3
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class FluxModel:
    def __init__(self, params_file, zero_bias=True):
        with open(params_file, "r") as f:
            params = json.load(f)

        self.input_size = params["input_size"]
        self.output_size = params["output_size"]
        self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
        self.input_std = np.array(params["input_std"], dtype=np.float32).T
        self.layers = []

        for layer_params in params["layers"]:
            W = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_W'))], dtype=np.float32)
            b = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_b'))], dtype=np.float32)
            if zero_bias:
              b = np.zeros_like(b)
            activation = layer_params["activation"]
            self.layers.append((W, b, activation))

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def identity(self, x):
        return x

    def forward(self, x):
        for W, b, activation in self.layers:
            if activation == 'σ':
                x = self.sigmoid(x.dot(W) + b)
            elif activation == 'identity':
                x = self.identity(x.dot(W) + b)
            else:
                raise ValueError(f"Unknown activation: {activation}")
        return x

    def evaluate(self, input_array):
        input_array = np.array(input_array, dtype=np.float32)#.reshape(1, -1)

        if input_array.shape[-1] != self.input_size:
            raise ValueError(f"Input array last dimension {input_array.shape[-1]} does not match the expected length {self.input_size}")
        # Rescale the input array using the input_mean and input_std
        input_array = (input_array - self.input_mean) / self.input_std

        output_array = self.forward(input_array)

        return output_array[0, 0]
  

def main():
  in_data = pd.read_csv("/Users/haiiro/NoSync/balanced_data.csv")
  in_data = in_data[np.abs(in_data["v_ego"] - 20) < 5]
  model = FluxModel("/Users/haiiro/NoSync/voltlat.json")
  print(f"{model.evaluate([10, 1.5, 0.5, 0.2]) = }")
  # print(in_data.head())
  
  # print(vars(model))
  
  plt.scatter(in_data["lateral_accel"], in_data["steer_cmd"], s=10, edgecolors='white', linewidths=1)
  x = np.arange(-3, 3, 0.1)
  y = np.array([model.evaluate([20, xi, 0, 0]) for xi in x])
  # print(x)
  # print(y)
  plt.plot(x, y, label='model', color='black', linewidth=2)
  plt.xlabel('lat accel [m/s^2]')
  plt.ylabel('steer cmd 3Nm')
  plt.legend()

  # show the plot
  plt.show()
  return

if __name__ == "__main__":
  main()