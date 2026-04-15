# Optimal Control via Dynamic Programming

A C++ implementation that solves the recursive Bellman quation for constrained, non-linear, discrete-time, finite-horizon optimal control problems of the form:
<br />


$$\begin{align}
&\text{minimise} \ J_{0,N} = h(x[N])  + \sum_{k=0}^{N-1} g(x[k], u[k]) \\
&\text{subject to} \ \ x[k+1] = a(x[k], u[k]) \\
\end{align}$$

$$\begin{align}
&x_{lb} \le x \le x_{ub} \\
&u_{lb} \le u \le u_{ub} \\
\end{align}$$

where:
- $J_{0, N}$ is the cost functional
- $x$ is the $n$-d state variable
- $u$ is the $m$-d control variable
- $h(.)$ is the terminal cost function
- $g(.)$ is the running cost function
- $a(.)$ is the state dynamics

Time is discretized into $N$ equally spaced time increments between $t=0$ to final time $t=T$. <br />
  
Both state and control values are discrete and bounded. And  <br />


<br />
The recurrence equation:  <br /> <br />

$$
J_{N-K,N}^{ * }(x[N-K]) = \min_{u[N-K]} \Bigl[ g(x[N-K], u[N-K]) + J_{N-(K-1), N}^{ * }(a(x[N-K], u[N-K]) ) \Big] \\ \\
$$

for $K = 1, 2, 3, ..., N$ with inital stage ($K=0$) given by $J_{N,N}^{ * }(x[N]) = h(x[N])$.


## Linear Regulator example 



```C++
ProblemDefination::ProblemDefination()
{
    n = 2;
    x_ql = 100;

    m = 1;
    u_ql = 100;

    T = 5; 
    N = 100;

    x_lb = {0, 0};
    x_ub = {4, 4}; 
    u_lb = {-3};
    u_ub = {0};

    initial_x = {1.75, 3.75};
    final_x = {0.0, 0.0};   // if x*[N] is free, leave this empty {}

    free_final_state = final_x.empty();
    delta_time = (double)T / (double)N;
}

std::vector<double> ProblemDefination::next_state_equation(std::vector<double> current_x, std::vector<double> current_u)
{
    std::vector<double> next_x; 

    next_x.push_back((current_u[0] - current_x[0]) * delta_time + current_x[0]);
    next_x.push_back(2.0*(current_u[0] - current_x[1]) * delta_time + current_x[1]);

    return next_x;
}

double ProblemDefination::cost_equation(std::vector<double> current_x, std::vector<double> current_u, bool terminal)
{
   	if (terminal) 
    {
		return 0.0; // terminal cost
	}
	return (pow(current_x[0], 2.0) + pow(current_x[1], 2.0) + pow(current_u[0], 2.0)) * delta_time;   // running cost
}
```

