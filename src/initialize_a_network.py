import tensorflow as tf
import numpy as np
import random
import sys
# print ("This is the name of the script: ", sys.argv[0])
# print ("The second argument is ", sys.argv[1])
# print ("The third argument is " , sys.argv[2])
# print ("Total number of arguments received is ", len(sys.argv) )

# here is the format in which the arguments are received :
# < script_name > < file to save this to > < inputs > < hidden layer 0 > .... < last hidden layer > < outputs >

number_of_hidden_layers = len(sys.argv) - (1 + 1 + 1 + 1) #For the name of the script and for the name of the file to be saved to
name_of_the_network_file = sys.argv[1]
no_of_inputs = sys.argv[3]
no_of_outputs = sys.argv[-1]

hidden_layers = []
for i in range(number_of_hidden_layers):
    hidden_layers.append(sys.argv[3+i])

my_random_seed= 127

def add_matrix_and_bias_to_file(filename, weights, biases):
    file = open(filename, "a")
    assert( len(weights) == len(biases) , "# outputs in the weight matrix and # in biases don't match ")

    no_of_outputs = len(weights)
    no_of_inputs = len(weights[0])

    for i in range(no_of_outputs):
        for j in range(no_of_inputs):
            x = weights[i, j]
            s = str(x) + '\n'
            file.writelines(s)
        x = biases[i]
        s = str(x) + '\n'
        file.writelines(s)

    file.close()

def random_weights(shape, seed):
    assert(len(shape) == 2 , "length of weight matrix shape is not 2")
    np.random.seed(seed)
    random_weights = np.random.rand(shape[0], shape[1])
    for i in range(shape[0]):
        for j in range(shape[1]):
            random_weights[i][j] = 2 * (random_weights[i][j] - 0.5)

    return random_weights

def random_biases(length, seed):
    np.random.seed(seed)
    random_bias = np.random.rand(length)
    for i in range(len(random_bias)):
        random_bias[i] = 2 * (random_bias[i] - 0.5)
    return random_bias

file.open(name_of_the_network_file,"w")


file.writelines((str(no_of_inputs) + "\n"))
file.writelines((str(no_of_outputs) + "\n"))
file.writelines((str(number_of_hidden_layers) + "\n"))

for i in range(number_of_hidden_layers):
    file.writelines((str(hidden_layers[i]) + "\n"))

file.close()



for i in range(number_of_hidden_layers + 1):
    if(i == 0):
        shape = []
        shape.append(hidden_layers[i])
        shape.append(no_of_inputs)
    else if(i == (number_of_hidden_layers)):
        shape = []
        shape.append(no_of_outputs)
        shape.append(hidden_layers[i-1])
    else:
        shape = []
        shape.append(hidden_layers[i])
        shape.append(hidden_layers[i-1])

    Weights = random_weights(shape, my_random_seed)
    Bias = random_biases(shape[0], my_random_seed)

    add_matrix_and_bias_to_file(name_of_the_network_file, Weights, Bias)
