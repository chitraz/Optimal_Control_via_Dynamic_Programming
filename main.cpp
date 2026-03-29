/*
Author: chitraz
Last Updated: 27/03/2026 
----------------------------------- Description -----------------------------------
Addresses optimal control problems of the form:
        minimize J(x,u,t) = g(x(0),x(T)) + ∫_0_T_[f(x,u,t)]dt
            s.t. dx/dt=h(x,u,t) (dynamics)
                x_min <= x <= x_max (bounded state values)
                u_min <= u <= u_max (bounded control values)

J:cost functional
g:boundary cost function
f:running cost function
h:system's state dynamics (1st order)
functions can be non-linear

Both state/control values and time steps are discretized, giving: 
Discrete-time, Finite-horizon form:
        minimize J(x,u,t) = g(x(0),x(T)) + SUM_0_T_[f(x,u,t)*deltaT] (cost eq)
            s.t. x(t+1) = x(t) + h(x,u,t)*deltaT (next-state eq)
                x in X_admissible (bounded, discrete state values)
                u in U_admissible (bounded, discrete control values)

Solves this problem by making use of optimal sub-structures:
	J* = min{J(0->T)} = min{J(0) + min{J(1) + min{J(2) + min{ J(3) + ... } } } }

recursive Bellman equation => Working backwards in time, find: 
			(stage T)   [J*(T), x*(T)]
			(stage T-1) [J*(T-1)=min{J(T-1) + J*(T)}, x*(T-1), u*(T-1)]
			(stage T-2) [J*(T-2)=min{J(T-2) + J*(T-1)}, x*(T-2), u*(T-2)]
			(stage T-3) [J*(T-3)=min{J(T-3) + J*(T-2)}, x*(T-3), u*(T-3)]
			...

outputs:
	- sequence of optimal controls (u*) to achieve minimum cost (J*) with optimal state trajectory (x*) 
*/
#include <iostream>
#include "BellmanSolver.hpp"
#include "BellmanSolver.cpp"
#include "ProblemDefination.hpp"
#include "ProblemDefination.cpp"

int main(){
    // Solve linear regulator problem defined in "ProblemDefination"
    BellmanSolver solver;
	solver.RunDynamicProgramingAlgo();
	
	// query the processed state-sapce-grid with initial state vector, get optimal state/control trajectories
	// save results into text files in /results/ 
	solver.PrintResults();

    return 0;
}