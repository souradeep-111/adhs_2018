import tensorflow as tf
import numpy as np
import random
import sys

name_of_controller_file = sys.argv[1]
name_of_system_model_file = sys.argv[2]

no_of_state_vars = 0
no_of_control_vars = 0
control_network_hidden_config = []

def read_and_get_the_structure_of_controller_network():
    global no_of_state_vars
    global no_of_control_vars
    global control_network_hidden_config

    data_stash = np.loadtxt(name_of_controller_file)
    pointer = 0
    no_of_state_vars = data_stash[pointer]
    pointer = pointer + 1
    no_of_control_vars = data_stash[pointer]
    pointer = pointer + 1
    no_of_hidden_layers = data_stash[pointer]
    pointer = pointer + 1
    control_network_hidden_config = []
    for i in range(no_of_hidden_layers):
        control_network_hidden_config.append(data_stash[pointer])
        pointer = pointer + 1


def create_the_lines_to_declare_the_control_network():
    # Returns a set of lines which when executed would create the control network variables
    #  also return the names of the different varaibles involved

def create_the_lines_to_initialize_the_control_network_from_file():


def create_the_lines_to_compute the_control_output():

def execute_strings_as_python_script():


def compute_network_output_from_file():
    filename =


# Read the control network's structure, size of hidden layers etc

# Create a function which generates lines to declare weights and bias variables

# Create the python lines which given an input variable, computes the output of the
# control network
