#include "BellmanSolver.hpp"
#include <iostream>
#include <vector>
#include <float.h>  
#include <cmath>
#include <algorithm>
#include <fstream>

#include <csignal>  // for std::raise(SIGINT), interupt for debugging

BellmanSolver::BellmanSolver()
{
    // calculate number of points in X_admissable and U_admissable sets
    num_of_x_points = pow(ProblemData.x_ql + 1, ProblemData.n);
    num_of_u_points = pow(ProblemData.u_ql + 1, ProblemData.m);

    std::cout << "number of x points: " << num_of_x_points << std::endl;
    std::cout << "number of u points: " << num_of_u_points << std::endl;
    std::cout << "number of time steps: " << ProblemData.N + 1 << std::endl;

    // calculate quantized step sizes 
    for (int i = 0; i < ProblemData.n; i++)
    {
        x_qstep.push_back((ProblemData.x_ub[i] - ProblemData.x_lb[i]) / ProblemData.x_ql);
    }
    for (int i = 0; i < ProblemData.m; i++)
    {
        u_qstep.push_back((ProblemData.u_ub[i] - ProblemData.u_lb[i]) / ProblemData.u_ql);
    } 

    // construct X_admissable set
    X_admissable = CreateAdmissableSet(ProblemData.n, num_of_x_points, x_qstep, 
        ProblemData.x_ql, ProblemData.x_ub, ProblemData.x_lb);
    
    //printSet(X_admissable, ProblemData.n);

    // construct U_admissable set
    U_admissable = CreateAdmissableSet(ProblemData.m, num_of_u_points, u_qstep, 
        ProblemData.u_ql, ProblemData.u_ub, ProblemData.u_lb);
    
    //printSet(U_admissable, ProblemData.m);

    // create a grid of discrete state values for each time step with a node in each point 
    CreateStateSpaceGrid();
}

void BellmanSolver::printSet(std::vector<std::vector<double>> SET, int dim)
{
    for (auto v: SET)
    {
        for (int i = 0; i < dim; i++)
        {
            std::cout << v[i] << ", "; 
        }
        std::cout << std::endl;
    }
    std::cout << "Set size: " << SET.size() << std::endl;
}

std::vector<std::vector<double>> BellmanSolver::CreateAdmissableSet(int dim, int total_points, 
    std::vector<double> qSteps, int q_levels, std::vector<double> upper_bounds, std::vector<double> lower_bounds)
{
    // create a set of all possiable combinations of quantised values, given the 
    std::vector<std::vector<double>> AdmissableSet;
        
    // list of numbers, one for each dimension to know when to increment in that dimension (via %)
    // starting from lower_bounds, increment the j postion every (q_levels + 1)^j counts. 
    std::vector<double> Increment_Condition;  
    for (int i = 0; i < dim ; i++)
    {
        Increment_Condition.push_back(pow((q_levels + 1), i));
    }
        
    // use modulus operator to increment through all combination of state values
    std::vector<double> temp_array;
    for (int i = 0; i < total_points; i++)  
    {   
        if (i == 0)
        {
            AdmissableSet.push_back(lower_bounds);   // start from lower bounds
        } else
        {
            for (int j = 0; j < dim; j++)
            {
                if (fmod(i, Increment_Condition[j]) == 0.0)
                {
                    temp_array.push_back(AdmissableSet[i-1][j] + qSteps[j]);  // increment
                } 
                else
                {
                    temp_array.push_back(AdmissableSet[i-1][j]); // don't increment
                }

                if (fmod(i, Increment_Condition[j]*(q_levels + 1)) == 0.0)  // check if the next (j+1) value is going to increment.
                {
                    temp_array.pop_back();
                    temp_array.push_back(lower_bounds[j]);  // loop back 
                }
            }
            AdmissableSet.push_back(temp_array);
            temp_array.clear();    
        }
    }
    return AdmissableSet;
} // cleaner way? create a vector containing all quantised values, one for each dimension. Take Cartesian product to get all combinations.

void BellmanSolver::CreateStateSpaceGrid()
{
    // create empty grid of correct size (24 bytes per node), 2D vector [N x #nodes] of nodes types
    node defualNode = {0.0, 0, 0, false};
    StateSpaceGrid.resize(ProblemData.N + 1, std::vector(num_of_x_points, defualNode)); 

    std::cout << "Bytes per node: " << sizeof(node) << std::endl;
    std::cout << "State Space Grid (MiB): " << sizeof(node)*(StateSpaceGrid[0].capacity() * StateSpaceGrid.capacity()) / 1048576.0 << std::endl;
}

int BellmanSolver::NearestStateValueIndex(std::vector<double> x)
{
    // round to the nearest discrete state value
    double discrete_value;
    int count_idx = 0;
    int step_count = 0;
    for (int i = 0; i < ProblemData.n; i++)
    {
        // get the closest qunatised state value
        discrete_value = x_qstep[i] * round(x[i] / x_qstep[i]);

        // if value out of bounds, return -1
        if (discrete_value < ProblemData.x_lb[i] || discrete_value >  ProblemData.x_ub[i])
        {
            return -1;
        }

        // count steps in each dim to deduce the index (see how X_admissable is created to understand logic)
        step_count = abs((discrete_value - ProblemData.x_lb[i]) / x_qstep[i]);

        // scale count bv (x_ql + 1)^i 
        count_idx += step_count * pow(ProblemData.x_ql + 1, i);
    }
    return count_idx;
}   

void BellmanSolver::RunDynamicProgramingAlgo()  
{
    // built based on flow chart given in [p.74, "Introduction to Optimal Control" by Donal E.Kirk]
    double COSMIN;
    double C_STAR; 
    double N = ProblemData.N;
    std::vector<double> next_state; 
    int next_state_idx;
    int final_state_idx;

    for (int k = 0; k < N + 1; k++)  // iterate time stages
    {
        for (int x_idx = 0; x_idx < num_of_x_points; x_idx++)  // iterate state vectors in X_admissable
        {
            if (k == 0)  
            {
                if (ProblemData.free_final_state)
                {
                    // compute and save terminal cost for all x[N] state values (free x*[N])
                    StateSpaceGrid[N - k][x_idx].opt_cost = ProblemData.cost_equation(X_admissable[x_idx], {}, true); 
                    StateSpaceGrid[N - k][x_idx].reachable = true;
                }
                else 
                {
                    //fixed x*[N]
                    final_state_idx = NearestStateValueIndex(ProblemData.final_x);
                    StateSpaceGrid[N - k][final_state_idx].opt_cost = ProblemData.cost_equation(X_admissable[final_state_idx], {}, true);
                    StateSpaceGrid[N - k][final_state_idx].reachable = true;
                }
                

            }
            else
            {
                // reset minimum cost to a large number
                COSMIN = DBL_MAX; 

                for (int u_idx = 0; u_idx < num_of_u_points; u_idx++)  // iterate control vectors in U_admissable
                {
                    next_state = ProblemData.next_state_equation(X_admissable[x_idx], U_admissable[u_idx]);
                    next_state_idx = NearestStateValueIndex(next_state);

                    // ensure resulting next state is within bounds and reachable
                    if (next_state_idx != -1 && StateSpaceGrid[N - k + 1][next_state_idx].reachable) 
                    {   

                        C_STAR = ProblemData.cost_equation(X_admissable[x_idx], U_admissable[u_idx], false) 
                                                                    + StateSpaceGrid[N - k + 1][next_state_idx].opt_cost;

                        if (C_STAR < COSMIN)
                        {
                            COSMIN = C_STAR;
                            StateSpaceGrid[N - k][x_idx].opt_cost = C_STAR;
                            StateSpaceGrid[N - k][x_idx].opt_u_idx = u_idx;
                            StateSpaceGrid[N - k][x_idx].opt_next_x_idx = next_state_idx;
                            StateSpaceGrid[N - k][x_idx].reachable = true;
                        }
                    }


                }
            }
        }                 
    }
    std::cout << "done. " <<std::endl;
}// interpolation cost? mean cost from closest (L2 distance?) nodes.

void BellmanSolver::PrintResults()
{
    // initial state values
    int inital_state_idx = NearestStateValueIndex(ProblemData.initial_x);

    // print percentage of reachable nodes in our processed state space grid
    int total_nodes = 0;
    int reachable_nodes = 0;
    for (int k = 0; k < ProblemData.N + 1; k++)
    {
        for (int x_idx = 0; x_idx < num_of_x_points; x_idx++)
        {
            if (StateSpaceGrid[k][x_idx].reachable)
            {   
                reachable_nodes ++;
            }
            total_nodes ++;
        }
    }
    std::cout<< "State Space Grid is " << ((double)reachable_nodes/(double)total_nodes)*100 << "\% filled." << std::endl;

    // check there is a valid path from given intital state value
    if (!StateSpaceGrid[0][inital_state_idx].reachable)
    {
        std::cout << "ERROR: can't reach any terimnal states from the given initial state." << std::endl;
        std::cout << "(nearest) x[0] = ";
        for (int i = 0; i < ProblemData.n; i++)
        {
            std::cout << X_admissable[inital_state_idx][i] << ", ";
        }
        std::cout << std::endl;
    }
    else
    {        
        // print optimal cost
        double optimal_cost = StateSpaceGrid[0][inital_state_idx].opt_cost;
        std::cout << "Optimal cost = " << optimal_cost << std::endl;

        // extract optimal state/control values from processed state-sapce-grid (by "tracing" it forward)
        std::vector<std::vector<double>> opt_states;
        std::vector<std::vector<double>> opt_controls;
        int current_x_index = inital_state_idx;

        for (int k = 0; k < ProblemData.N + 1; k++)
        {   
            opt_states.push_back(X_admissable[current_x_index]);
            opt_controls.push_back(U_admissable[StateSpaceGrid[k][current_x_index].opt_u_idx]);

            if (k == ProblemData.N)
            {
                opt_controls.pop_back();  // remove last control value (don't need control after the final state has been reached!)
            }

            current_x_index = StateSpaceGrid[k][current_x_index].opt_next_x_idx;  // jump to next optimal state node
        }

        // export optimal control policy and optimal state trajectory to txt files
        std::ofstream x_out_file("results/opt_x.txt");
        std::ofstream u_out_file("results/opt_u.txt");

        for (int k = 0; k < ProblemData.N + 1; k++)
        {
            // [time stamp] [x_1]..[x_n] 
            x_out_file << k * ProblemData.delta_time << " ";
            for (int i = 0; i < ProblemData.n; i++)
            {
                x_out_file << opt_states[k][i] << " ";
            }
            x_out_file << "\n";

            // [time stamp] [u_1]..[u_m]
            if (k != ProblemData.N)
            {
                u_out_file << k * ProblemData.delta_time << " ";
                for (int i = 0; i < ProblemData.m; i++)
                {
                    u_out_file << opt_controls[k][i] << " ";
                }
                u_out_file << "\n";
            }

        }
        x_out_file.close();
        u_out_file.close();
    }

}
