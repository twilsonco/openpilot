#!/Users/haiiro/.pyenv/shims/python3
##!/usr/bin/env python3
import numpy as np
import feather
import matplotlib.pyplot as plt

from common.numpy_fast import interp

def identity(Z):
    # actually a leaky identity
    return np.maximum(0.01*Z, Z)

def sigmoid(Z):
    # symmetric sigmoid
    return 1/(1+np.exp(-Z))

def relu(Z):
    return np.maximum(0,Z)

ACT_FUNC = {
    'identity': identity,
    'sigmoid': sigmoid,
    'relu': relu
}

def identity_backward(dA, Z):
    # actually a leaky identity
    dZ = np.array(dA, copy = True)
    dZ[Z <= 0] *= 0.01
    return dZ

def sigmoid_backward(dA, Z):
    sig = sigmoid(Z)
    return dA * sig * (1 - sig)

def relu_backward(dA, Z):
    dZ = np.array(dA, copy = True)
    dZ[Z <= 0] = 0
    return dZ

ACT_FUNC_GRAD = {
    'identity': identity_backward,
    'sigmoid': sigmoid_backward,
    'relu': relu_backward
}

TORQUE_NN_INPUT_VARIABLES = ["v_ego", "lateral_acceleration", "lateral_jerk", "road_roll"]

TORQUE_NN_LAYERS = [
  [len(TORQUE_NN_INPUT_VARIABLES), 25, "relu"],
  [25, 1, "sigmoid"]
]

TORQUE_NN_START_W = []#[[[0.0191, 0.156, 0.111, 0.155], [-0.0135, -0.183, 0.151, 0.0767], [-0.0438, 0.0165, -0.0431, -0.0377], [0.0134, -0.323, -0.204, -0.0513], [-0.102, -0.0199, -0.095, -0.134], [0.133, -0.35, -0.0226, -0.116], [-0.0544, 0.00969, -0.117, 0.0383], [-0.0268, -0.265, -0.109, 0.235], [-0.00554, -0.113, 0.0816, -0.121], [0.0379, -0.463, -0.195, 0.034], [0.0775, 0.02, -0.014, -0.0333], [-0.183, -0.29, -0.0847, 0.0961], [0.0396, -0.391, 0.0544, -0.0763], [-0.0581, 0.491, 0.128, -0.08], [-0.0942, -0.251, 0.0252, 0.121], [-0.046, 0.0141, -0.107, -0.126], [0.123, 0.571, 0.046, -0.0201], [0.0776, -0.288, 0.00125, 0.23], [-0.021, 0.428, -0.203, -0.0202], [0.0155, 0.0588, 0.0156, -0.214], [-0.0425, 0.358, 0.228, -0.177], [-0.0892, -0.223, 0.0973, 0.0257], [-0.00718, -0.248, -0.0352, 0.0656], [-0.0699, 0.00276, -0.0377, -0.15], [0.0127, 0.178, 0.021, -0.0773]], [[0.259, -0.195, -0.0245, -0.271, -0.031, -0.417, -0.0134, -0.375, -0.0604, -0.681, 0.155, -0.35, -0.434, 0.744, -0.331, 0.0311, 0.642, -0.355, 0.391, 0.107, 0.5, -0.237, -0.419, 0.0491, 0.262]]]
TORQUE_NN_START_B = []#[[-0.0934, -0.00341, -0.0508, 0.00394, -0.0199, 0.203, 0.188, 0.186, 0.0254, 0.506, -0.19, 0.15, 0.24, 0.606, 0.163, 0.0306, 0.297, 0.0559, 0.201, 0.0852, 0.251, -0.00324, 0.356, -0.14, 0.188], [0.06]]
TORQUE_NN_ACTIVATION_FUNCTIONS = []#["relu", "sigmoid"]

class NumPyNeuralNetwork:
  def __init__(self, 
               training = False, 
               model_layers = TORQUE_NN_LAYERS,
               W = TORQUE_NN_START_W,
               b = TORQUE_NN_START_B, 
               activation_functions = TORQUE_NN_ACTIVATION_FUNCTIONS, 
               learning_rate = 0.1, 
               input_mean = [], 
               input_std = [],
               random_seed = 42):
    assert len(model_layers) > 1 and len(W) == len(b) == len(activation_functions) and len(input_mean) == len(input_std)
    np.random.seed(random_seed)
    self.input_size = model_layers[0][0]
    self.output_size = model_layers[-1][1]
    self.W = [np.array(w, dtype=np.float64) for w in W]
    self.b = [np.array(bb, dtype=np.float64).reshape((1, -1)).T for bb in b]
    self.activation_function_names = activation_functions
    self.input_mean = np.array(input_mean, dtype=np.float64)
    self.input_std = np.array(input_std, dtype=np.float64)
    self.output_scale = 1.0
    if len(input_mean) != self.input_size:
      self.input_mean = np.zeros(self.input_size, dtype=np.float64)
      self.input_std = np.zeros(self.input_size, dtype=np.float64)
    for i,layer in enumerate(model_layers):
      if i >= len(self.W) or self.W[i].shape != (layer[1], layer[0]) \
          or self.b[i].shape[0] != layer[1] \
          or self.activation_function_names[i] != layer[2]:
        print(f"Model does not match the expected structure. Creating new model.")
        self.W = []
        self.b = []
        self.activation_function_names = []
        break
    if not self.W:
      for layer in model_layers:
        self.W.append(np.random.randn(layer[1], layer[0]).astype(np.float64) * 0.1)
        self.b.append(np.random.randn(layer[1], 1).astype(np.float64) * 0.1)
        self.activation_function_names.append(layer[2])
    self.activation_functions = [ACT_FUNC[activation] for activation in self.activation_function_names]
    self.activation_functions_grad = [ACT_FUNC_GRAD[activation] for activation in self.activation_function_names] if training else []
    self.training_cache = {}
    self.learning_rate = learning_rate
  
  def evaluate(self, input_array, do_normalize = True, do_rescale_output = True):
    # convert to numpy array if not already
    if not isinstance(input_array, np.ndarray):
      input_array = np.array(input_array, dtype=np.float64)
    # check the input array length
    in_len = input_array.shape[0]
    if in_len != self.input_size:
      # Need to add on zeros to fill out the input array to match the correct length.
      if 2 <= in_len < 4:
        input_array = np.concatenate((input_array, np.zeros((self.input_size - in_len, 1))), axis=0)
      else:
        raise ValueError(f"Input array length {len(input_array)} does not match the expected length {self.input_size}")
    # Rescale the input array using the input_mean and input_std
    if do_normalize:
      input_array = (input_array - self.input_mean) / self.input_std
    output_array = self.forward(input_array, do_rescale_output)
    # return only scalar value if input_array.shape[1] == 1 and self.output_size == 1
    if input_array.shape[1] == 1 and self.output_size == 1:
      return output_array[0, 0]
    elif self.output_size == 1:
      return output_array[0,:]
    else:
      return output_array
    
  def forward(self, x, do_rescale_output = False):
    for W, b, activation in zip(self.W, self.b, self.activation_functions):
      x = activation(W.dot(x) + b)
    if do_rescale_output:
      x = x * 2 - 1
    return x
  
  def forward_w_cache(self, x):
    for i, (W, b, activation) in enumerate(zip(self.W, self.b, self.activation_functions)):
      self.training_cache[f"A{i}"] = x
      x = W.dot(x) + b
      self.training_cache[f"Z{i+1}"] = x
      x = activation(x)
    return x * self.output_scale
  
  def backward(self, Y_hat, Y, eps = 0.000000000001):
    m = Y.shape[1]
    Y = Y.reshape(Y_hat.shape)
    dA_prev = -2 * (Y - Y_hat) / m # derivative of MSE cost function
    for layer_idx_prev, layer in reversed(list(enumerate(zip(self.W, self.activation_functions_grad)))):
        layer_idx_curr = layer_idx_prev + 1
        dA_curr = dA_prev
        A_prev = self.training_cache[f"A{layer_idx_prev}"]
        Z_curr = self.training_cache[f"Z{layer_idx_curr}"]
        dZ_curr = layer[1](dA_curr, Z_curr)
        dW_curr = np.dot(dZ_curr, A_prev.T) / m
        db_curr = np.sum(dZ_curr, axis=1, keepdims=True) / m
        dA_prev = np.dot(layer[0].T, dZ_curr)
        self.training_cache[f"dW{layer_idx_curr}"] = dW_curr
        self.training_cache[f"db{layer_idx_curr}"] = db_curr
        
  def get_cost_value(self, Y_hat, Y):
    cost = np.mean((Y_hat - Y) ** 2)
    return cost
  
  def update_params(self):
    for layer_idx in range(len(self.W)):
      if self.activation_function_names[layer_idx] == "identity":
        continue
      self.W[layer_idx] -= self.training_cache[f"dW{layer_idx+1}"] * self.learning_rate         
      self.b[layer_idx] -= self.training_cache[f"db{layer_idx+1}"] * self.learning_rate 
  
  def train(self, X, Y, epochs, verbose=False, callback=None):
    cost_history = []
    # update mean and std of input
    self.input_mean = np.mean(X, axis=1, keepdims=True)
    self.input_std = np.std(X, axis=1, keepdims=True)
    # normalize X
    X_normalized = (X - self.input_mean) / self.input_std
    # get scaling factor to downscale Y to be between -1 and 1, and so that I can rescale later
    self.output_scale = np.max(np.abs(Y))
    for i in range(epochs):
      Y_hat = self.forward_w_cache(X_normalized)
      cost = self.get_cost_value(Y_hat, Y)
      cost_history.append(cost)
      self.backward(Y_hat, Y)
      self.update_params()
      if(i % 50 == 0):
        if(verbose):
          print("Iteration: {:05} - cost: {:.5e}".format(i, cost))
        if(callback is not None):
          callback(i, epochs, cost_history)
    return cost_history

  def summary(self, do_print=True):
    summary_lines = [
      "NumPyNeuralNetwork Summary:",
      f"Input size: {self.input_size}",
      f"Output size: {self.output_size}",
      f"Number of layers: {len(self.W)}",
      "Layer details:"
    ]
    for i, (W, b, activation) in enumerate(zip(self.W, self.b, self.activation_function_names)):
      summary_lines.append(
          f"  Layer {i + 1}: W: {W.shape}, b: {b.shape}, f: {activation}"
      )
    summary_str = "\n".join(summary_lines)
    if do_print:
      print(summary_str)
    return summary_str

def get_friction(lateral_accel_error, bp, v):
  return interp(lateral_accel_error, bp, v)

def clamp(x, lo, hi):
  return max(lo, min(hi, x))

# generates theoretical training data in order to calculate a good starting set of parameters
def generate_trial_lateral_training_data(kf=0.33, friction_factor=0.15, steer_threshold=1.2, friction_threshold=0.3, roll_kf = 0.1):
  fbp = [-friction_threshold, friction_threshold]
  fv = [-friction_factor, friction_factor]
  X = []
  y = []
  for v_ego in np.linspace(1.0, 40.0, 20):
    for la in np.linspace(-4, 4, 100):
      for lj in np.linspace(-3, 3, 10):
        for roll in np.linspace(-0.2, 0.2, 10):
          steer = kf * la
          steer += get_friction(lj, fbp, fv)
          steer -= np.sin(roll) * 9.81 * roll_kf
          steer = clamp(steer, -steer_threshold, steer_threshold)
          X.append([v_ego, la, lj, roll])
          y.append(steer)
  return np.array(X).T, np.array(y).reshape((1, -1))

def main():
  # import data as feather file
  columns = ['steer_cmd', 'v_ego', 'lateral_accel', 'lateral_jerk', 'roll']
  data = feather.read_dataframe("/Users/haiiro/NoSync/CHEVROLET_VOLT_PREMIER_2017_balanced.feather", columns=columns)
  X = data[columns[1:]].to_numpy().T
  y = data[columns[0]].to_numpy().reshape((1, -1))
  print(X.shape)
  print(y.shape)
  
  X, y = generate_trial_lateral_training_data()#kf=0.5, friction_factor=0.25)
  
  model = NumPyNeuralNetwork(training=True, learning_rate=1e5)
  
  def callback_numpy_plot(index, max_index, cost_history=[]):
    plot_title = "NumPy Model - It: {:05}".format(index)
    if cost_history:
        plot_title += f" - loss: {cost_history[-1]:.5f}"
    file_name = "numpy_model_{:05}.png".format(index//50)
    file_path = __file__.replace(".py", f"outplot{index//50:04d}.png")
    la_input = np.linspace(-4, 4, 100)
    vego = 20
    # la_input is the second colulm in the N x 18 input matrix. Need to make the input matrix
    # with the first column being v_ego and the second column being la_input, and the rest zeros
    X_input = np.zeros((X.shape[0], 100))
    X_input[1, :] = la_input
    X_input[0, :] = vego
    # simple scatter/line plot
    plt.clf()
    model_steer = model.evaluate(X_input)
    # prediction_probs = prediction_probs.reshape(prediction_probs.shape[1], 1)
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.scatter(X[1, :], y, s=0.5, alpha=0.1)
    ax1.grid(True, which="both", color="lightgrey", linestyle="--")
    ax1.axhline(y=0, color="grey", linewidth=1.5)
    ax1.axvline(x=0, color="grey", linewidth=1.5)
    ax1.plot(X_input[1, :], model_steer, linewidth=3, color="black")
    X_input[2, :] = -2
    model_steer = model.evaluate(X_input)
    ax1.plot(X_input[1, :], model_steer, linewidth=3, color="orange", label="-2m/s² error")
    X_input[2, :] = 2
    model_steer = model.evaluate(X_input)
    ax1.plot(X_input[1, :], model_steer, linewidth=3, color="red", label="+2m/s² error")
    X_input[2, :] = 0
    X_input[3, :] = 0.1
    model_steer = model.evaluate(X_input)
    ax1.plot(X_input[1, :], model_steer, linewidth=3, color="blue", label="+0.1rad roll")
    X_input[3, :] = -0.1
    model_steer = model.evaluate(X_input)
    ax1.plot(X_input[1, :], model_steer, linewidth=3, color="cyan", label="-0.1rad roll")
    # axis labels (steer command vs lat accel [m/s^2])
    ax1.set_xlabel("lateral acceleration [m/s²]")
    ax1.set_ylabel("steer command")
    # add legend
    ax1.legend(loc="lower right")
    # add title with iteration and cost
    ax1.set_title(plot_title)
    
    # Plot cost history on right-hand-side y-axis
    ax2.set_ylabel("loss")
    cost_x_values = np.linspace(-4, 4, len(cost_history))
    ax2.plot(cost_x_values, np.array(cost_history), linewidth=2, color="grey", label="loss", linestyle="--")
    # round upper y-axis for log scale
    y_max = np.exp(np.ceil(np.log(max(cost_history))))
    ax2.set_ylim(min(cost_history), y_max)
    ax2.set_yscale('log')
    # save figure
    fig.tight_layout()
    fig.savefig(file_path)
    # clear figure from memory
    plt.clf()
    plt.close(fig)
    plt.close('all')
  
  cost_history = model.train(X, y, 5000, True, callback_numpy_plot)
  
  if True:
    # # pretty print W using 0.2g format, and as a valid python list
    W_str = "W = ["
    for layer in model.W:
      W_str += "["
      for row in layer:
        W_str += "["
        for col in row:
          W_str += f"{col:0.3g}, "
        W_str = W_str[:-2] + "], "
      W_str = W_str[:-2] + "], "
    W_str = W_str[:-2] + "]"
    print(W_str)
    
    # # pretty print b using 0.2g format, and as a valid python list,
    # # and transpose to remove the excessive brackets
    b_str = "b = ["
    for layer in model.b:
      b_str += "["
      for row in layer:
        b_str += f"{row[0]:0.3g}, "
      b_str = b_str[:-2] + "], "
    b_str = b_str[:-2] + "]"
    print(b_str)
    
    print(model.activation_function_names)
    
main()