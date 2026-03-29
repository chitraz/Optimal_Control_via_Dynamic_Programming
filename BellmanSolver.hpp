#pragma once

#include "ProblemDefination.hpp"

// a node in our state-space-time grid
struct node {   
    
    double opt_cost;     // optimal cost from this node to target node
    int opt_u_idx;       // optimal control (index for U_admissable)
    int opt_next_x_idx;  // next optimal state (index for X_admissable)
    bool reachable;      // backward reachability flag
};

class BellmanSolver
{
    public:
        BellmanSolver();
        void printSet(std::vector<std::vector<double>> SET, int dim);
        std::vector<std::vector<double>> CreateAdmissableSet(int dim, int total_points, 
            std::vector<double> qSteps, int q_levels, std::vector<double> upper_bounds, std::vector<double> lower_bounds);
        void CreateStateSpaceGrid();
        int NearestStateValueIndex(std::vector<double> x);

        void RunDynamicProgramingAlgo();
        
        void PrintResults();
        
    private:
        ProblemDefination ProblemData;

        int num_of_x_points;  // number of discrete points in X_admissable
        int num_of_u_points;  // number of discrete points in U_admissable

        std::vector<double> x_qstep;  // quantized step sizes in each state dimension 
        std::vector<double> u_qstep;  // quantized step sizes in each control dimension 

        std::vector<std::vector<double>> X_admissable;  // set of all admissable state values 
        std::vector<std::vector<double>> U_admissable;  // set of all admissable control values

        std::vector<std::vector<node>> StateSpaceGrid;  // state-space-time grid [N x #nodes]
};

