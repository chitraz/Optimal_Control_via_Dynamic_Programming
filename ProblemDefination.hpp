#pragma once
#include <vector>

class ProblemDefination
{
    public:
        ProblemDefination();
        std::vector<double> next_state_equation(std::vector<double> current_x, std::vector<double> current_u);
        double cost_equation(std::vector<double> current_x, std::vector<double> current_u, bool terminal);

        int n;     // state space dimension
        int x_ql;  // number of state quantisation levels

        int m;     // control space dimension
        int u_ql;  // number of control quantisation levels
       
        int T;     // final time 
        int N;     // number of discrete time steps
        
        std::vector<double> x_lb, x_ub;  // state value constraints: lower and upper bounds
        std::vector<double> u_lb, u_ub;  // control value constraints: lower and upper bounds

        std::vector<double> initial_x, final_x;  // state trajectory boundary condition

        double delta_time;  // quantized time step

        bool free_final_state;  // flag: true => x*[N] is free, false => x*[N] is fixed.
};