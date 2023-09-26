#!/usr/bin/env python3

# MLP Neural Network adapted from:
# File:         fully_connected_nn.py
# Version:      1.0
# Author:       SkalskiP https://github.com/SkalskiP
# Date:         31.10.2018
# Description:  The file contains a simple implementation of a fully connected neural network.
#               The original implementation can be found in the Medium article:
#               https://towardsdatascience.com/lets-code-a-neural-network-in-plain-numpy-ae7e74410795

import numpy as np
import os
import feather
from matplotlib import pyplot as plt

def identity(Z):
    # actually a leaky identity
    return np.maximum(0.01*Z, Z)

def sigmoid(Z):
    # symmetric sigmoid
    return 1/(1+np.exp(-Z))

def tanh(Z):
    return np.tanh(Z)

def relu(Z):
    return np.maximum(0,Z)

act_func_forward = {
    'identity': identity,
    'sigmoid': sigmoid,
    'tanh': tanh,
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

def tanh_backward(dA, Z):
    return dA * (1 - np.tanh(Z)**2)

def relu_backward(dA, Z):
    dZ = np.array(dA, copy = True)
    dZ[Z <= 0] = 0
    return dZ

act_func_backward = {
    'identity': identity_backward,
    'sigmoid': sigmoid_backward,
    'tanh': tanh_backward,
    'relu': relu_backward
}

def init_layers(nn_architecture, seed = 99):
    np.random.seed(seed)
    number_of_layers = len(nn_architecture)
    params_values = {}
    
    for idx, layer in enumerate(nn_architecture):
        layer_idx = idx + 1
        
        layer_input_size = layer["input_dim"]
        layer_output_size = layer["output_dim"]
        
        if "activation" in layer:
            if layer["activation"] not in act_func_forward or layer["activation"] not in act_func_backward:
                raise Exception('Non-supported activation function')
            params_values['W' + str(layer_idx)] = np.random.randn(
                layer_output_size, layer_input_size) * 0.1
        else: # initialize identity layer weights to zero
            params_values['W' + str(layer_idx)] = np.zeros((layer_output_size, layer_input_size))
        params_values['b' + str(layer_idx)] = np.random.randn(
            layer_output_size, 1) * 0.1
            
    return params_values


def single_layer_forward_propagation(A_prev, W_curr, b_curr, activation="relu"):
    Z_curr = np.dot(W_curr, A_prev) + b_curr
    return act_func_forward[activation](Z_curr), Z_curr


def full_forward_propagation(X, params_values, nn_architecture):
    memory = {}
    A_curr = X
    
    for idx, layer in enumerate(nn_architecture):
        layer_idx = idx + 1
        A_prev = A_curr
        
        activ_function_curr = layer["activation"] if "activation" in layer else "identity"
        W_curr = params_values["W" + str(layer_idx)]
        b_curr = params_values["b" + str(layer_idx)]
        A_curr, Z_curr = single_layer_forward_propagation(A_prev, W_curr, b_curr, activ_function_curr)
        
        memory["A" + str(idx)] = A_prev
        memory["Z" + str(layer_idx)] = Z_curr
       
    return A_curr, memory


def get_cost_value(Y_hat, Y):
    cost = np.mean((Y_hat - Y) ** 2)
    return cost



def single_layer_backward_propagation(dA_curr, W_curr, b_curr, Z_curr, A_prev, activation="identity"):
    m = A_prev.shape[1]
    
    dZ_curr = act_func_backward[activation](dA_curr, Z_curr)
    dW_curr = np.dot(dZ_curr, A_prev.T) / m
    db_curr = np.sum(dZ_curr, axis=1, keepdims=True) / m
    dA_prev = np.dot(W_curr.T, dZ_curr)

    return dA_prev, dW_curr, db_curr


def full_backward_propagation(Y_hat, Y, memory, params_values, nn_architecture, eps = 0.000000000001):
    grads_values = {}
    m = Y.shape[1]
    Y = Y.reshape(Y_hat.shape)
    
    dA_prev = - (np.divide(Y, Y_hat + eps) - np.divide(1 - Y, 1 - Y_hat + eps))
    
    for layer_idx_prev, layer in reversed(list(enumerate(nn_architecture))):
        layer_idx_curr = layer_idx_prev + 1
        activ_function_curr = layer["activation"] if "activation" in layer else "identity"
        
        dA_curr = dA_prev
        
        A_prev = memory["A" + str(layer_idx_prev)]
        Z_curr = memory["Z" + str(layer_idx_curr)]
        
        W_curr = params_values["W" + str(layer_idx_curr)]
        b_curr = params_values["b" + str(layer_idx_curr)]
        
        dA_prev, dW_curr, db_curr = single_layer_backward_propagation(
            dA_curr, W_curr, b_curr, Z_curr, A_prev, activ_function_curr)
        
        grads_values["dW" + str(layer_idx_curr)] = dW_curr
        grads_values["db" + str(layer_idx_curr)] = db_curr
    
    return grads_values


def update(params_values, grads_values, nn_architecture, learning_rate):
    for layer_idx, layer in enumerate(nn_architecture, 1):
        if "activation" not in layer:
            continue
        params_values["W" + str(layer_idx)] -= learning_rate * grads_values["dW" + str(layer_idx)]        
        params_values["b" + str(layer_idx)] -= learning_rate * grads_values["db" + str(layer_idx)]

    return params_values


def train(X, Y, nn_architecture, epochs, learning_rate, verbose=False, callback=None):
    params_values = init_layers(nn_architecture, 2)
    cost_history = []
    
    for i in range(epochs):
        Y_hat, cache = full_forward_propagation(X, params_values, nn_architecture)
        
        cost = get_cost_value(Y_hat, Y)
        cost_history.append(cost)
        
        grads_values = full_backward_propagation(Y_hat, Y, cache, params_values, nn_architecture)
        params_values = update(params_values, grads_values, nn_architecture, learning_rate)
        
        if(i % 50 == 0):
            if(verbose):
                print("Iteration: {:05} - cost: {:.5f}".format(i, cost))
            if(callback is not None):
                callback(i, params_values)
            
    return params_values, cost_history

def main():
    # import data as feather file
    columns = ['steer_cmd', 'v_ego', 'lateral_accel', 'lateral_jerk', 'roll']
    data = feather.read_dataframe("/Users/haiiro/NoSync/CHEVROLET_VOLT_PREMIER_2017_balanced.feather", columns=columns)
    X_train = data[columns[1:]].to_numpy().T
    y_train = data[columns[0]].to_numpy().reshape((1, -1))
    print(X_train.shape)
    print(y_train.shape)
    
    # normalize X_train, first calculating mean and std
    X_train_mean = np.mean(X_train, axis=1, keepdims=True)
    X_train_std = np.std(X_train, axis=1, keepdims=True)
    # then normalize
    X_train_normalized = (X_train - X_train_mean) / X_train_std
    
    # rescale y_train to be between 0 and 1
    y_train_rescaled = (y_train + 1) / 2
    
    NN_ARCHITECTURE = [
        {"input_dim": X_train.shape[0], "output_dim": 25, "activation": "relu"},
        {"input_dim": 25, "output_dim": 1, "activation": "sigmoid"}
    ]
    
    def callback_numpy_plot(index, params):
        plot_title = "NumPy Model - It: {:05}".format(index)
        file_name = "numpy_model_{:05}.png".format(index//50)
        file_path = __file__.replace(".py", f"outplot{index//50}.png")
        la_input = np.linspace(-4, 4, 100)
        vego = 20
        # la_input is the second colulm in the N x 18 input matrix. Need to make the input matrix
        # with the first column being v_ego and the second column being la_input, and the rest zeros
        X_input = np.zeros((X_train.shape[0], 100))
        X_input[1, :] = la_input
        X_input[0, :] = vego
        # Then normalize
        X_input_normalized = (X_input - X_train_mean) / X_train_std
        # simple scatter/line plot
        plt.clf()
        prediction_probs, _ = full_forward_propagation(X_input_normalized, params, NN_ARCHITECTURE)
        prediction_probs = prediction_probs.reshape(prediction_probs.shape[1], 1)
        # rescale prediction_probs to be between -1 and 1
        prediction_probs = prediction_probs * 2 - 1
        plt.scatter(X_train[1, :], y_train, s=0.5, alpha=0.1)
        plt.plot(X_input[1, :], prediction_probs[:, 0], linewidth=3, color="red")
        X_input[2, :] = -2
        X_input_normalized = (X_input - X_train_mean) / X_train_std
        prediction_probs, _ = full_forward_propagation(X_input_normalized, params, NN_ARCHITECTURE)
        prediction_probs = prediction_probs.reshape(prediction_probs.shape[1], 1)
        # rescale prediction_probs to be between -1 and 1
        prediction_probs = prediction_probs * 2 - 1
        plt.plot(X_input[1, :], prediction_probs[:, 0], linewidth=3, color="grey")
        X_input[2, :] = 2
        X_input_normalized = (X_input - X_train_mean) / X_train_std
        prediction_probs, _ = full_forward_propagation(X_input_normalized, params, NN_ARCHITECTURE)
        prediction_probs = prediction_probs.reshape(prediction_probs.shape[1], 1)
        # rescale prediction_probs to be between -1 and 1
        prediction_probs = prediction_probs * 2 - 1
        plt.plot(X_input[1, :], prediction_probs[:, 0], linewidth=3, color="darkgrey")
        X_input[2, :] = 0
        X_input[3, :] = 0.1
        X_input_normalized = (X_input - X_train_mean) / X_train_std
        prediction_probs, _ = full_forward_propagation(X_input_normalized, params, NN_ARCHITECTURE)
        prediction_probs = prediction_probs.reshape(prediction_probs.shape[1], 1)
        # rescale prediction_probs to be between -1 and 1
        prediction_probs = prediction_probs * 2 - 1
        plt.plot(X_input[1, :], prediction_probs[:, 0], linewidth=3, color="blue")
        X_input[3, :] = -0.1
        X_input_normalized = (X_input - X_train_mean) / X_train_std
        prediction_probs, _ = full_forward_propagation(X_input_normalized, params, NN_ARCHITECTURE)
        prediction_probs = prediction_probs.reshape(prediction_probs.shape[1], 1)
        # rescale prediction_probs to be between -1 and 1
        prediction_probs = prediction_probs * 2 - 1
        plt.plot(X_input[1, :], prediction_probs[:, 0], linewidth=3, color="cyan")
        plt.savefig(file_path)
    
    params_values = train(X_train_normalized, y_train_rescaled, NN_ARCHITECTURE, 10000, 0.1, True, callback_numpy_plot)
    
    # print(params_values)
    
main()