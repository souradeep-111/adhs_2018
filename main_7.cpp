#include "./src/propagate_intervals.h"

using namespace std;
// So the above data assumes, that this number is
// same for all the networks you have in the setting,
// and also all the networks are single output network
// with the first one giving the output

int main(int argc , char ** argv)
{
	int simulate_system, produce_reachable_sets, prove_invariancy_of_target, option;

	if(argc != 2)
	{
		cout << "Wrong number of command line arguments " << endl;
		cout << "enter options :\n 0 to simulate system , \n 1 to produce reach sets and \n 2 to prove invariancy \n 3 to overlay system trajectories on reach sets " << endl;
		cout << "Exiting.. " << endl;
		exit(0);
	}
	else
	{
		sscanf(argv[1], "%d", &option);
		if(option == 0)
		{
			simulate_system = 1;
			produce_reachable_sets = 0;
			prove_invariancy_of_target = 0;
		}
		else if (option == 1)
		{
			simulate_system = 0;
			produce_reachable_sets = 1;
			prove_invariancy_of_target = 0;
		}
		else if (option == 2)
		{
			simulate_system = 0;
			produce_reachable_sets = 0;
			prove_invariancy_of_target = 1;
		}
		else if (option == 3)
		{
			simulate_system = 1;
			produce_reachable_sets = 1;
			prove_invariancy_of_target = 0;
		}
		else
		{
			cout << "Unrecognised option " << endl;
			exit(0);
		}
	}

	unsigned int acc_in_reach_sets;
	plotting_data system_plots(4);
	acc_in_reach_sets = 3;
	unsigned int steps = 10;

  // Reachability testing of closed loop network with directions

	if(produce_reachable_sets)
	{
		char state_vars_file_1[] = "./network_files/control_ex_19_4d/neural_network_information_1" ;
		char state_vars_file_2[] = "./network_files/control_ex_19_4d/neural_network_information_2" ;
		char state_vars_file_3[] = "./network_files/control_ex_19_4d/neural_network_information_3" ;
		char state_vars_file_4[] = "./network_files/control_ex_19_4d/neural_network_information_4" ;

		network_handler system_1(state_vars_file_1);
		network_handler system_2(state_vars_file_2);
		network_handler system_3(state_vars_file_3);
		network_handler system_4(state_vars_file_4);

		vector< vector< vector< datatype > > > w_1;
		vector< vector< datatype > > b_1;
		vector< vector< vector< datatype > > > w_2;
		vector< vector< datatype > > b_2;
		vector< vector< vector< datatype > > > w_3;
		vector< vector< datatype > > b_3;
		vector< vector< vector< datatype > > > w_4;
		vector< vector< datatype > > b_4;
		vector< vector< vector< datatype > > > w_5;
		vector< vector< datatype > > b_5;


		system_1.return_network_information(w_1, b_1);
		system_2.return_network_information(w_2, b_2);
		system_3.return_network_information(w_3, b_3);
		system_4.return_network_information(w_4, b_4);

		w_5 = w_1;
		b_5 = b_1;
		patch_networks_vertically(w_5,b_5,w_2,b_2);
		patch_networks_vertically(w_5,b_5,w_3,b_3);
		patch_networks_vertically(w_5,b_5,w_4,b_4);

		char name[] = "./network_files/control_ex_19_4d/state_vars";
		write_network_to_file(w_5, b_5, name);

		char state_vars_file[] = "./network_files/control_ex_19_4d/state_vars" ;

		char controller_file[] = "./network_files/control_ex_19_4d/neural_network_controller" ;

		char closed_loop_file[] = "./network_files/control_ex_19_4d/composed_network_non_linear";

		merge_networks(-10, 1, closed_loop_file, state_vars_file, controller_file);


		network_handler system_network(closed_loop_file);
		unsigned extra_directions = 6;
		unsigned int dim = 4;

		vector< vector < datatype > > directions(extra_directions, vector< datatype >(dim));
		directions[0][0] = 1;directions[0][1] = 1;  directions[0][2] = 0;  directions[0][3] = 0;
		directions[1][0] = 1;directions[1][1] = -1; directions[1][2] = 0;  directions[1][3] = 0;
		directions[2][0] = 0;directions[2][1] = 1;  directions[2][2] = 1;  directions[2][3] = 0;
		directions[3][0] = 0;directions[3][1] = 1;  directions[3][2] = -1; directions[3][3] = 0;
		directions[4][0] = 0;directions[4][1] = 0;  directions[4][2] = 1;  directions[4][3] = 1;
		directions[5][0] = 0;directions[5][1] = 0;  directions[5][2] = 1;  directions[5][3] = -1;


		vector< vector< vector< datatype > > > weights;
		vector< vector< datatype > > biases;
		system_network.return_network_information(weights, biases);


		vector< datatype > offset_already(dim);
		offset_already[0] = -20;
		offset_already[1] = -20;
		offset_already[2] = -20;
		offset_already[3] = -100;

		add_directions_to_output(weights, biases, directions,
			sherlock_parameters.constr_comb_offset, offset_already);

		system_network.update_information(weights, biases);

		vector< unsigned int > important_outputs(dim + extra_directions, 1);
		fill(important_outputs.begin() + dim, important_outputs.end(), 0);

		// cout << "No of hidden layers " << system_network.no_of_hidden_layers << endl;
		vector< vector< datatype > > total_directions(dim + extra_directions, vector< datatype >(dim) );
		total_directions[0][0] = 1;total_directions[0][1] = 0;total_directions[0][2] = 0;total_directions[0][3] = 0;
		total_directions[1][0] = 0;total_directions[1][1] = 1;total_directions[1][2] = 0;total_directions[1][3] = 0;
		total_directions[2][0] = 0;total_directions[2][1] = 0;total_directions[2][2] = 1;total_directions[2][3] = 0;
		total_directions[3][0] = 0;total_directions[3][1] = 0;total_directions[3][2] = 0;total_directions[3][3] = 1;

		unsigned int i, j , k, steps;
		i = 0;
		while(i < directions.size())
		{
			total_directions[i + dim] = directions[i];
			i++;
		}

		vector< datatype > scale_vector_input(dim + extra_directions, 0.1);

		vector< datatype > scale_vector_output(dim + extra_directions, 10);

		vector< vector< datatype > > biases_now(dim + extra_directions, vector< datatype >(2));

		biases_now[0][0] = 0.01;biases_now[0][1] = 0.02;
		biases_now[1][0] = -0.04;biases_now[1][1] = -0.03;
		biases_now[2][0] = 0.01;biases_now[2][1] = 0.02;
		biases_now[3][0] = -0.09;biases_now[3][1] = -0.08;


		// Target Set works for 3 steps
		// biases_now[0][0] = -0.01;biases_now[0][1] = 0.01;
		// biases_now[1][0] = -0.02;biases_now[1][1] = 0.02;
		// biases_now[2][0] = -0.01;biases_now[2][1] = 0.01;
		// biases_now[3][0] = -0.01;biases_now[3][1] = 0.01;

		biases_now[4][0] = -10;biases_now[4][1] = 10;
		biases_now[5][0] = -10;biases_now[5][1] = 10;
		biases_now[6][0] = -10;biases_now[6][1] = 10;
		biases_now[7][0] = -10;biases_now[7][1] = 10;
		biases_now[8][0] = -10;biases_now[8][1] = 10;
		biases_now[9][0] = -10;biases_now[9][1] = 10;


		vector< datatype > offset_amount(dim + extra_directions);
		offset_amount[0] = -20;
		offset_amount[1] = -20;
		offset_amount[2] = -20;
		offset_amount[3] = -100;

		fill(offset_amount.begin() + dim, offset_amount.end(), sherlock_parameters.constr_comb_offset);

		vector< vector< datatype > > biases_next(dim + extra_directions, vector< datatype >(2,0));
		vector< vector< datatype > > region_constraints;

		convert_direction_biases_to_constraints(total_directions,
																						biases_now,
																						region_constraints);



		set_info current_set, next_set;
		current_set.region_constr = region_constraints;
		current_set.time_stamp = 0;


		vector< vector< datatype > > target_region;
		vector< datatype > constraint(dim + 1);

		// x_0 < 0.01 ... [ -x_0 + 0 x_1 + 0 x_2 + 0 x_3 + 0.01 > 0 ]
		constraint[0] = -1; constraint[1] = 0; constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_0 > -0.01 ... [ x_0 + 0 x_1 + 0 x_2 + 0 x_3 + 0.01 > 0 ]
		constraint[0] = 1; constraint[1] = 0; constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_1 < 0.02 ... [ 0 x_0 + (-1) x_1 + 0 x_2 + 0 x_3 + 0.02 > 0 ]
		constraint[0] = 0; constraint[1] = -1;constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.02;
		target_region.push_back(constraint);

		// x_1 > -0.02 ... [ 0 x_0 + x_1 + 0 x_2 + 0 x_3 + 0.02 > 0 ]
		constraint[0] = 0; constraint[1] = 1; constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.02;
		target_region.push_back(constraint);

		// x_2 < 0.01 ... [ 0 x_0 + 0 x_1 + (-1) x_2 + 0 x_3 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = -1; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_2 > -0.01 ... [ 0 x_0 + 0 x_1 + x_2 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = 1; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_3 < 0.01 ... [ 0 x_0 + 0 x_1 + 0 x_2 + (-1) x_3 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = 0; constraint[3] = -1; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_3 > -0.01 ... [ 0 x_0 + 0 x_1 + 0 x_2 + x_3 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = 0; constraint[3] = 1; constraint[4] = 0.01;
		target_region.push_back(constraint);


		vector< vector< datatype > > system_reach_set(2, vector< datatype >(4));
		set_info target_set;
		target_set.region_constr = target_region;

		queue < set_info > reach_sets;
		reach_sets.push(current_set);
		vector< set_info > all_reach_sets;


		cout << "Initial Set : " << endl;
		j = 0;
		while(j < (4 + extra_directions))
		{
			cout << "For direction = " << j << " [ " << biases_now[j][0] << " , " << biases_now[j][1]  << " ] ";
			// cout << " size =  [" << biases_next[j][1] - biases_next[j][0] << "] " << endl;
			cout << endl;
			j++;
		}

		cout << endl;
		cout << "Direction Vectors are : " << endl;
		j = 0;
		while(j < 4 + extra_directions)
		{
			cout << "Direction " << j << " = " ;
			cout << "[ " << total_directions[j][0] << " "
					 << total_directions[j][1]<< " "
					 << total_directions[j][2]<< " "
					 << total_directions[j][3] << " ]" << endl;
			j++;
		}
		cout << endl;

		cout << "System Simulation Starts " << endl;
		cout << " ------------------------------------------------------------ " << endl;

		cout << endl;
		unsigned int split_return, split_started = 0;
		unsigned int max_simulation_steps = 0;

		all_reach_sets.push_back(current_set);
		while(!reach_sets.empty())
		{

			cout << "Queue size = " << reach_sets.size() << endl;
			current_set = reach_sets.front();
			reach_sets.pop();

			simulate_accelerated(system_network, acc_in_reach_sets, important_outputs, scale_vector_input,
				 										offset_already, current_set.region_constr, biases_next);
			adjust_offset(biases_next, offset_amount);
			scale_vector(biases_next, scale_vector_input);

			convert_direction_biases_to_constraints(total_directions,
				biases_next, next_set.region_constr);

			next_set.time_stamp = current_set.time_stamp + acc_in_reach_sets;
			cout << "At time = " << next_set.time_stamp << " reach set boundaries are " << endl;
			print_biases(biases_next);

			split_return = split_set(next_set, target_set, reach_sets);

			all_reach_sets.push_back(next_set);
			max_simulation_steps++;

		}

		i = 1;
		while(i < acc_in_reach_sets)
		{

			simulate_accelerated(system_network, i, important_outputs, scale_vector_input, offset_already, region_constraints, biases_next);
			adjust_offset(biases_next, offset_amount);
			scale_vector(biases_next, scale_vector_input);
			convert_direction_biases_to_constraints(total_directions,
				biases_next, current_set.region_constr);

			current_set.time_stamp = i;

			all_reach_sets.push_back(current_set);


			cout << "At time = " << current_set.time_stamp << " reach set boundaries are " << endl;
			print_biases(biases_next);

			cout << "Max simulation steps = " << max_simulation_steps << endl;
			j = 0;
			while(j < (max_simulation_steps-1))
			{
				simulate_accelerated(system_network, acc_in_reach_sets, important_outputs, scale_vector_input, offset_already, current_set.region_constr, biases_next);
				adjust_offset(biases_next, offset_amount);
				scale_vector(biases_next, scale_vector_input);
				convert_direction_biases_to_constraints(total_directions,
					biases_next, next_set.region_constr);

				next_set.time_stamp = current_set.time_stamp + acc_in_reach_sets;

				cout << "At time = " << next_set.time_stamp << " reach set boundaries are " << endl;
				print_biases(biases_next);


				all_reach_sets.push_back(next_set);

				current_set = next_set;
				j++;
			}

			i++;
		}

		system_plots.collect_and_merge_reach_sets(all_reach_sets);


		cout << " ------------------------------------------------------------ " << endl;
		cout << "System Simulation Ends " << endl << endl;


		cout << "Val at the end : " << endl;
		j = 0;
		while(j < (dim + extra_directions))
		{
			cout << "For direction = " << j << " [ " << biases_next[j][0] << " , " << biases_next[j][1] << " ] ";
			// cout << " size =  [" << biases_next[j][1] - biases_next[j][0] << "] " << endl;
			cout << endl;
			j++;
		}

	}

	steps = system_plots.reach_set_time_range;
	//  Simulation of Closed loop dynamics
	if (simulate_system)
	{
		unsigned int dim;
		dim = 4;

		// // Simple simulation of a closed loop network
		char state_vars_file_1[] = "./network_files/control_ex_19_4d/neural_network_information_1" ;
		char state_vars_file_2[] = "./network_files/control_ex_19_4d/neural_network_information_2" ;
		char state_vars_file_3[] = "./network_files/control_ex_19_4d/neural_network_information_3" ;
		char state_vars_file_4[] = "./network_files/control_ex_19_4d/neural_network_information_4" ;

		network_handler system_1(state_vars_file_1);
		network_handler system_2(state_vars_file_2);
		network_handler system_3(state_vars_file_3);
		network_handler system_4(state_vars_file_4);

		vector< vector< vector< datatype > > > w_1;
		vector< vector< datatype > > b_1;
		vector< vector< vector< datatype > > > w_2;
		vector< vector< datatype > > b_2;
		vector< vector< vector< datatype > > > w_3;
		vector< vector< datatype > > b_3;
		vector< vector< vector< datatype > > > w_4;
		vector< vector< datatype > > b_4;
		vector< vector< vector< datatype > > > w_5;
		vector< vector< datatype > > b_5;


		system_1.return_network_information(w_1, b_1);
		system_2.return_network_information(w_2, b_2);
		system_3.return_network_information(w_3, b_3);
		system_4.return_network_information(w_4, b_4);

		w_5 = w_1;
		b_5 = b_1;
		patch_networks_vertically(w_5,b_5,w_2,b_2);
		patch_networks_vertically(w_5,b_5,w_3,b_3);
		patch_networks_vertically(w_5,b_5,w_4,b_4);

		char name[] = "./network_files/control_ex_19_4d/state_vars";
		write_network_to_file(w_5, b_5, name);

		char state_vars_file[] = "./network_files/control_ex_19_4d/state_vars" ;

		char controller_file[] = "./network_files/control_ex_19_4d/neural_network_controller" ;

		char closed_loop_file[] = "./network_files/control_ex_19_4d/composed_network_non_linear";

		merge_networks(-10, 1, closed_loop_file, state_vars_file, controller_file);

		// This section does a simple simulation for a single input thing
		// no compostion involved

		network_handler system_network(closed_loop_file);

		vector< vector< vector< datatype > > > weights;
		vector< vector< datatype > > biases;
		vector< vector< vector< datatype > > > return_weights;
		vector< vector< datatype > > return_biases;

		system_network.return_network_information(weights, biases);
		// vector< unsigned int > important_outputs(3,1);

		// unsigned int h = 0;
		// while(h < 1)
		// {
		// 	patch_networks_horizontally(weights, biases, network_offset,important_outputs,
		// 															weights, biases, return_weights, return_biases);
		// 	weights = return_weights;
		// 	biases = return_biases;
		// 	h++;
		// }

		system_network.update_information(weights, biases);


		vector< vector< vector< datatype > > > weights_0;
		vector< vector< datatype > > biases_0;
		vector< vector< vector< datatype > > > weights_1;
		vector< vector< datatype > > biases_1;
		vector< vector< vector< datatype > > > weights_2;
		vector< vector< datatype > > biases_2;
		vector< vector< vector< datatype > > > weights_3;
		vector< vector< datatype > > biases_3;

		vector< vector< unsigned int > > active_weights;

		vector< datatype > x(4);
		vector< datatype > x_next(4);

		system_network.return_network_information(weights, biases);
		system_network.cast_to_single_output_network(weights, biases, 1);
		weights_0 = weights; biases_0 = biases;

		system_network.return_network_information(weights, biases);
		system_network.cast_to_single_output_network(weights, biases, 2);
		weights_1 = weights; biases_1 = biases;

		system_network.return_network_information(weights, biases);
		system_network.cast_to_single_output_network(weights, biases, 3);
		weights_2 = weights; biases_2 = biases;

		system_network.return_network_information(weights, biases);
		system_network.cast_to_single_output_network(weights, biases, 4);
		weights_3 = weights; biases_3 = biases;

		vector< datatype > scale_vector_input(4, 0.1);
		vector< datatype > scale_vector_output(4, 10);

		x[0] = 0.085;
		x[1] = 0.085;
		x[2] = 0.055;
		x[3] = -0.085;

		vector< vector< datatype > > trace_data;
		vector< datatype > system_point(4);

		unsigned int i, k, rand_num;
		double rand_frac;

		unsigned int no_of_plots = 10;
		vector< vector< datatype > > range_now(4, vector< datatype >(2));
		range_now[0][0] = 0.01;  range_now[0][1] = 0.02;
		range_now[1][0] = -0.04; range_now[1][1] = -0.03;
		range_now[2][0] = 0.01;  range_now[2][1] = 0.02;
		range_now[3][0] = -0.09; range_now[3][1] = -0.08;

		k = 0;
		while(k < no_of_plots)
		{

			rand_num = rand() % no_of_plots;
			rand_frac = (double) rand_num / (double)no_of_plots;
			x[0] = rand_frac * (range_now[0][1] - range_now[0][0]) + range_now[0][0];
			x[1] = rand_frac * (range_now[1][1] - range_now[1][0]) + range_now[1][0];
			x[2] = rand_frac * (range_now[2][1] - range_now[2][0]) + range_now[2][0];
			x[3] = rand_frac * (range_now[3][1] - range_now[3][0]) + range_now[3][0];

			trace_data.clear();
			trace_data.push_back(x);
			i = 0;
			while(i < (steps) /* * acc_in_reach_sets*/)
			{

				x_next[0] = compute_network_output(x, weights_0, biases_0, active_weights);
				x_next[0] -= 20 ;
				x_next[0] *= 0.1;

				x_next[1] = compute_network_output(x, weights_1, biases_1, active_weights);
				x_next[1] -= 20;
				x_next[1] *= 0.1;


				x_next[2] = compute_network_output(x, weights_2, biases_2, active_weights);
				x_next[2] -= 20;
				x_next[2] *= 0.1;

				x_next[3] = compute_network_output(x, weights_3, biases_3, active_weights);
				x_next[3] -= 100;
				x_next[3] *= 0.1;

				system_point = x_next;

				x = x_next;
				// cout << "x = " << x[0] << " "<< x[1] << endl;
				// if(!((i+1) % acc_in_reach_sets))
				// {
				// 	trace_data.push_back(system_point);
				// }
				// else if(acc_in_reach_sets == 1)
				// {
					trace_data.push_back(system_point);
				// }
				i++;
			}
			system_plots.add_system_trace(trace_data);
			k++;
		}

	}

	// Proving invariancy of Target
	if(prove_invariancy_of_target)
	{
		char state_vars_file_1[] = "./network_files/control_ex_19_4d/neural_network_information_1" ;
		char state_vars_file_2[] = "./network_files/control_ex_19_4d/neural_network_information_2" ;
		char state_vars_file_3[] = "./network_files/control_ex_19_4d/neural_network_information_3" ;
		char state_vars_file_4[] = "./network_files/control_ex_19_4d/neural_network_information_4" ;

		network_handler system_1(state_vars_file_1);
		network_handler system_2(state_vars_file_2);
		network_handler system_3(state_vars_file_3);
		network_handler system_4(state_vars_file_4);

		vector< vector< vector< datatype > > > w_1;
		vector< vector< datatype > > b_1;
		vector< vector< vector< datatype > > > w_2;
		vector< vector< datatype > > b_2;
		vector< vector< vector< datatype > > > w_3;
		vector< vector< datatype > > b_3;
		vector< vector< vector< datatype > > > w_4;
		vector< vector< datatype > > b_4;
		vector< vector< vector< datatype > > > w_5;
		vector< vector< datatype > > b_5;


		system_1.return_network_information(w_1, b_1);
		system_2.return_network_information(w_2, b_2);
		system_3.return_network_information(w_3, b_3);
		system_4.return_network_information(w_4, b_4);

		w_5 = w_1;
		b_5 = b_1;
		patch_networks_vertically(w_5,b_5,w_2,b_2);
		patch_networks_vertically(w_5,b_5,w_3,b_3);
		patch_networks_vertically(w_5,b_5,w_4,b_4);

		char name[] = "./network_files/control_ex_19_4d/state_vars";
		write_network_to_file(w_5, b_5, name);

		char state_vars_file[] = "./network_files/control_ex_19_4d/state_vars" ;

		char controller_file[] = "./network_files/control_ex_19_4d/neural_network_controller" ;

		char closed_loop_file[] = "./network_files/control_ex_19_4d/composed_network_non_linear";

		merge_networks(-10, 1, closed_loop_file, state_vars_file, controller_file);


		network_handler system_network(closed_loop_file);
		unsigned extra_directions = 6;
		unsigned int dim = 4;

		vector< vector < datatype > > directions(extra_directions, vector< datatype >(dim));
		directions[0][0] = 1;directions[0][1] = 1;  directions[0][2] = 0;  directions[0][3] = 0;
		directions[1][0] = 1;directions[1][1] = -1; directions[1][2] = 0;  directions[1][3] = 0;
		directions[2][0] = 0;directions[2][1] = 1;  directions[2][2] = 1;  directions[2][3] = 0;
		directions[3][0] = 0;directions[3][1] = 1;  directions[3][2] = -1; directions[3][3] = 0;
		directions[4][0] = 0;directions[4][1] = 0;  directions[4][2] = 1;  directions[4][3] = 1;
		directions[5][0] = 0;directions[5][1] = 0;  directions[5][2] = 1;  directions[5][3] = -1;


		vector< vector< vector< datatype > > > weights;
		vector< vector< datatype > > biases;
		system_network.return_network_information(weights, biases);


		vector< datatype > offset_already(dim);
		offset_already[0] = -20;
		offset_already[1] = -20;
		offset_already[2] = -20;
		offset_already[3] = -100;

		add_directions_to_output(weights, biases, directions,
			sherlock_parameters.constr_comb_offset, offset_already);

		system_network.update_information(weights, biases);

		vector< vector< vector< datatype > > > return_weights;
		vector< vector< datatype > > return_biases;
		unsigned int acc_number = 8;

		system_network.return_network_information(weights, biases);

		vector< unsigned int > important_outputs(dim + extra_directions, 1);
		fill(important_outputs.begin() + dim, important_outputs.end(), 0);

		vector< datatype > scaling_factor(dim + extra_directions, 0.1);


		if(acc_number > 1)
		{
			patch_networks_horizontally(weights, biases, scaling_factor, offset_already, important_outputs,
																	weights, biases, return_weights, return_biases);

			acc_number -= 2;
			unsigned int h = 0;
			while(h < (acc_number))
		 {
				patch_networks_horizontally(return_weights, return_biases, scaling_factor, offset_already, important_outputs,
																	weights, biases, return_weights, return_biases);
				h++;
		 }
		weights = return_weights;
		biases = return_biases;

		}

		system_network.update_information(weights, biases);
		// cout << "No of hidden layers " << system_network.no_of_hidden_layers << endl;
		vector< vector< datatype > > total_directions(dim + extra_directions, vector< datatype >(dim) );
		total_directions[0][0] = 1;total_directions[0][1] = 0;total_directions[0][2] = 0;total_directions[0][3] = 0;
		total_directions[1][0] = 0;total_directions[1][1] = 1;total_directions[1][2] = 0;total_directions[1][3] = 0;
		total_directions[2][0] = 0;total_directions[2][1] = 0;total_directions[2][2] = 1;total_directions[2][3] = 0;
		total_directions[3][0] = 0;total_directions[3][1] = 0;total_directions[3][2] = 0;total_directions[3][3] = 1;

		unsigned int i, j , k, steps;
		i = 0;
		while(i < directions.size())
		{
			total_directions[i + dim] = directions[i];
			i++;
		}

		vector< datatype > scale_vector_input(dim + extra_directions, 0.1);

		vector< datatype > scale_vector_output(dim + extra_directions, 10);

		vector< vector< datatype > > biases_now(dim + extra_directions, vector< datatype >(2));
		// biases_now[0][0] = 0.08;biases_now[0][1] = 0.09;
		// biases_now[1][0] = 0.08;biases_now[1][1] = 0.09;
		// biases_now[2][0] = 0.05;biases_now[2][1] = 0.06;
		// biases_now[3][0] = -0.09;biases_now[3][1] = -0.08;

		// Target Set works for 3 steps
		biases_now[0][0] = -0.01;biases_now[0][1] = 0.01;
		biases_now[1][0] = -0.02;biases_now[1][1] = 0.02;
		biases_now[2][0] = -0.01;biases_now[2][1] = 0.01;
		biases_now[3][0] = -0.01;biases_now[3][1] = 0.01;


		biases_now[4][0] = -10;biases_now[4][1] = 10;
		biases_now[5][0] = -10;biases_now[5][1] = 10;
		biases_now[6][0] = -10;biases_now[6][1] = 10;
		biases_now[7][0] = -10;biases_now[7][1] = 10;
		biases_now[8][0] = -10;biases_now[8][1] = 10;
		biases_now[9][0] = -10;biases_now[9][1] = 10;


		vector< datatype > offset_amount(dim + extra_directions);
		offset_amount[0] = -20;
		offset_amount[1] = -20;
		offset_amount[2] = -20;
		offset_amount[3] = -100;

		fill(offset_amount.begin() + dim, offset_amount.end(), sherlock_parameters.constr_comb_offset);

		vector< vector< datatype > > biases_next(dim + extra_directions, vector< datatype >(2,0));
		vector< vector< datatype > > region_constraints;

		convert_direction_biases_to_constraints(total_directions,
																						biases_now,
																						region_constraints);



		set_info current_set, next_set;
		current_set.region_constr = region_constraints;
		current_set.time_stamp = 0;

		vector< vector< datatype > > target_region;
		vector< datatype > constraint(dim + 1);

		// x_0 < 0.01 ... [ -x_0 + 0 x_1 + 0 x_2 + 0 x_3 + 0.01 > 0 ]
		constraint[0] = -1; constraint[1] = 0; constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_0 > -0.01 ... [ x_0 + 0 x_1 + 0 x_2 + 0 x_3 + 0.01 > 0 ]
		constraint[0] = 1; constraint[1] = 0; constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_1 < 0.02 ... [ 0 x_0 + (-1) x_1 + 0 x_2 + 0 x_3 + 0.02 > 0 ]
		constraint[0] = 0; constraint[1] = -1;constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.02;
		target_region.push_back(constraint);

		// x_1 > -0.02 ... [ 0 x_0 + x_1 + 0 x_2 + 0 x_3 + 0.02 > 0 ]
		constraint[0] = 0; constraint[1] = 1; constraint[2] = 0; constraint[3] = 0; constraint[4] = 0.02;
		target_region.push_back(constraint);

		// x_2 < 0.01 ... [ 0 x_0 + 0 x_1 + (-1) x_2 + 0 x_3 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = -1; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_2 > -0.01 ... [ 0 x_0 + 0 x_1 + x_2 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = 1; constraint[3] = 0; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_3 < 0.01 ... [ 0 x_0 + 0 x_1 + 0 x_2 + (-1) x_3 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = 0; constraint[3] = -1; constraint[4] = 0.01;
		target_region.push_back(constraint);

		// x_3 > -0.01 ... [ 0 x_0 + 0 x_1 + 0 x_2 + x_3 + 0.01 > 0 ]
		constraint[0] = 0; constraint[1] = 0; constraint[2] = 0; constraint[3] = 1; constraint[4] = 0.01;
		target_region.push_back(constraint);

		set_info target_set;
		target_set.region_constr = target_region;

		queue < set_info > reach_sets;
		reach_sets.push(current_set);

		cout << "Initial Set : " << endl;
		j = 0;
		while(j < (4 + extra_directions))
		{
			cout << "For direction = " << j << " [ " << biases_now[j][0] << " , " << biases_now[j][1]  << " ] ";
			// cout << " size =  [" << biases_next[j][1] - biases_next[j][0] << "] " << endl;
			cout << endl;
			j++;
		}

		cout << endl;
		cout << "Direction Vectors are : " << endl;
		j = 0;
		while(j < 4 + extra_directions)
		{
			cout << "Direction " << j << " = " ;
			cout << "[ " << total_directions[j][0] << " "
					 << total_directions[j][1]<< " "
					 << total_directions[j][2]<< " "
					 << total_directions[j][3] << " ]" << endl;
			j++;
		}
		cout << endl;

		cout << "System Simulation Starts " << endl;
		cout << " ------------------------------------------------------------ " << endl;

		cout << endl;

		while(!reach_sets.empty())
		{

			// cout << "Queue size = " << reach_sets.size() << endl;
			current_set = reach_sets.front();
			reach_sets.pop();

			j = 0;
			while(j < (dim + extra_directions))
			{
				// cout << "Doing with output no = " << j << endl;
				system_network.return_interval_output(current_set.region_constr,
																						biases_next[j], j+1); // PLus 1 because internally the count starts from 1
				j++;
			}
			adjust_offset(biases_next, offset_amount);
			scale_vector(biases_next, scale_vector_input);

			cout << "At time = " << current_set.time_stamp << " reach set boundaries are " << endl;
			j = 0;
			while(j < (dim + extra_directions))
			{
				cout << "For direction = " << j << " [ " << biases_next[j][0] << " , " << biases_next[j][1] << " ] ";
				// cout << " size =  [" << biases_next[j][1] - biases_next[j][0] << "] " << endl;
				cout << endl;
				j++;
			}

			convert_direction_biases_to_constraints(total_directions,
				biases_next, next_set.region_constr);

			next_set.time_stamp = current_set.time_stamp + 1;

			split_set(next_set, target_set, reach_sets);

		}


		cout << " ------------------------------------------------------------ " << endl;
		cout << "System Simulation Ends " << endl << endl;

		cout << "Val at the end : " << endl;
		j = 0;
		while(j < (dim + extra_directions))
		{
			cout << "For direction = " << j << " [ " << biases_next[j][0] << " , " << biases_next[j][1] << " ] ";
			// cout << " size =  [" << biases_next[j][1] - biases_next[j][0] << "] " << endl;
			cout << endl;
			j++;
		}

	}

	if(option == 0)
	system_plots.plot(1);

	if(option == 1)
	system_plots.plot(2);

	if(option == 3)
	system_plots.plot(3);


  return 0;
}
