#!/usr/bin/python3

#implement a neural network class. The code is heavily inspired by this article:
#http://neuralnetworksanddeeplearning.com/chap1.html
import numpy as np

def zigmoid(z):
    return (1./(1. + np.exp(-z)))

class LearningController:

    def __init__(self, sizes, learning_rate = 0.0001, activation_func = zigmoid): #Give the size of the layers as a list (or list-like)
        self.layers = len(sizes)
        self.sizes = sizes
        self.biases = [ np.random.randn(y, 1) * 0 for y in sizes[1:] ]
        self.weights = [ np.random.randn(y, x) * 0 for x, y in zip(sizes[:-1], sizes[1:])]
        self.learning_rate = learning_rate
        self.activation_func = activation_func

    def feedforward(self, a): # a is the input of the network. returns the output
        for b, w in zip(self.biases, self.weights):
            a = self.activation_func(np.dot(w, a) )
        #print (self.weights)
        return a

    def updateWeights(self, x, reward_change):
        #This is done according to the following paper:
        #Extraction of Reward-Related Feature Space
        #Using Correlation-Based and Reward-Based
        #Learning Methods

        #first, calculate q
        q = abs(min(0, reward_change))

        #then for every weight, calculate the new weight
        for i in range(len(self.weights)):
            for j in range(len(self.weights[i])):
                for k in range(len(self.weights[i][j])):
                    self.weights[i][j][k] += self.learning_rate * np.linalg.norm(x) * q


    def getControl(self, x, reward_change):
        self.updateWeights(x, reward_change)
        return self.feedforward(x)
