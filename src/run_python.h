#ifndef run_python_h
#define run_python_h

#include <string>


using namespace std;

void create_a_fully_connected_network(vector< int > hidden_layer_config , string filename );

// Some function which runs python script according to the points received,
// trains a neural network from it according to some MPC setting
void train_neural_network_controller( vector< vector< double > > input_points,  )

#endif
