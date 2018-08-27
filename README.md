# ADHS_2018 

The files here are to reproduce the examples in the ADHS 2018 paper, "Learning and Verification of Feedback Control
Systems using Feedforward Neural Networks."
The implementation here uses "sherlock" as a fundamental primitive to solve the reachability problem over Neural Networks.

# sherlock

Sherlock is an output range analysis tool for deep neural networks.
The current version can handle only feedforward neural networks, with
ReLU activation units.

The only library that is needed is the MILP solver Gurobi. It's free
for academic purposes and can be downloaded from here :
http://www.gurobi.com/resources/getting-started/mip-basics


## Instructions to Compile

Please modify the file Makefile.locale to help us find Gurobi.

For a Mac with the latest gurobi8.00 installed your likely settings
will be:

> HOST_ARCH=mac64
> GUROBI_PATH=/Library/gurobi800

For a linux box, your settins will be:

> ARCH=linux64 # if you are using a linux box
> GUROBI_PATH=/opt/gurobi800

You should feel free to modify these two variables. The Makefile will look for Gurobi headers under

> $(GUROBI_PATH)/$(HOST_ARCH)/include

and libraries under

> $(GUROBI_PATH)/$(HOST_ARCH)/include


Once these are set, you should type 
 > make
 
 It should create all the different executables for each of the examples. Which you should be able to run with the right option.
You can chose what option you want by just running it first as : 

> ./run_file_1

For example 1, and so on.
