/**************************************************************************************************
 * @ File: MPC.cpp
 * @ Author: MarkGao
 * @ Date: 2020-05-15
 * @ Email: 819699632@qq.com
 * @ Version: 1.0
 * @ Description:
**************************************************************************************************/
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
using Eigen::VectorXd;
using std::cout;
using std::endl;

// Set the timestep length and duration
size_t N = 20;
const double dt = 0.1;
const double Lf = 2.67;
const double ref_v = 15;    // mph

// Initialized the size(Index) of each variable. Since the size of variable always change, the size_t is used.
// State index , since the Ipopt needs vector inputs. Each variable should be saperated with N points interval.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;

// Error index
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;

// Another index for minimize the value gap between sequential actuation in vehicle moving.
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval
{
public:
	// Fitted polynomial coefficients
    VectorXd coeffs;
    FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    void operator()(ADvector &fg, const ADvector &vars)
    {
		// TODO: implement MPC

        // Assign weights for cost items.
		const int weight_cte = 2010;
		const int weight_epsi = 2010;
		const int weight_v = 1;
		const int weight_delta = 12;
		const int weight_a = 11;
		const int weight_delta_change = 110;
		const int weight_a_change = 11;

        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;

        for(int t=0; t<N; t++)
        {
			fg[0] += weight_cte * CppAD::pow(vars[cte_start + t], 2);
			fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t], 2);
			fg[0] += weight_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Minimize the use of actuators.
        for(int t=0; t<N-1; t++)
        {
			fg[0] += weight_delta * CppAD::pow(vars[delta_start + t], 2);
			fg[0] += weight_a * CppAD::pow(vars[a_start + t], 2);
		}

		// Minimize the value gap between sequential actuations.
        for(int t=0; t<N-2; t++)
        {
			fg[0] += weight_delta_change * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += weight_a_change * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

        // Setup Model Constraints
        // Initial constraints
        // the fg[0] is cost value, so we need to feed variables into fg from fg[1+ variable_index]
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// Then set the rest of constraints
		// for loop start from 1 since the initial condition is pre-defined in initial contraints
        for(int t=1; t<N; t++)
        {
            // State at time t+1 means next time step
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];

            // State at time t means current step
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];

			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];

			// polynominal function. Value in x0, and the order is 3 , f0 will get the trajectory of Y. 
			AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
			//AD<double> psides0 = CppAD::atan(coeffs[1]);
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

            // The idea here is to constraint this value to be 0.
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}



/**************************************************************************************************
 * @ Function: Solve()
 * @ Description:       // Description of the function
 * @ Calls:                       // Functions being called by this function
 * @ Input:                       // Description of input parameters, include the function, value range of each parameter
 * @ Output:                    // Description of output parameters
 * @ Return:                    // Description of return value
 * @ Others:
**************************************************************************************************/
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
	bool ok = true;
	typedef CPPAD_TESTVECTOR(double) Dvector;

    // State vector holds all current values need for vars below
	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	// Setting the number of model variables (includes both states and inputs).
	// N * state vector size + (N - 1) * 2 actuators (For steering & acceleration)
	size_t n_vars = N * 6 + (N - 1) * 2;
	// Setting the number of constraints
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
    for(int i=0; i<n_vars; i++)
    {
		vars[i] = 0.0;
	}

	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);
	// Sets lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits to the max negative and positive values.
    for(int i=0; i<delta_start; i++)
    {
		vars_lowerbound[i] = -bound;
		vars_upperbound[i] = bound;
	}

    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    for(int i=delta_start; i<a_start; i++)
    {
		vars_lowerbound[i] = -(PI / 180) * 25;
		vars_upperbound[i] = (PI / 180) * 25;
	}

	// Acceleration/decceleration upper and lower limits.
    for(int i=a_start; i<n_vars; i++)
    {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
    for(int i=0; i<n_constraints; i++)
    {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	// Start lower and upper limits at current values
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options,
                                          vars,
                                          vars_lowerbound, vars_upperbound,
                                          constraints_lowerbound, constraints_upperbound,
                                          fg_eval,
                                          solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;
    cout<<"Cost = "<<cost<<endl;

	// Return the first actuator values, along with predicted x and y values to plot in the simulator.
	vector<double> solved_results;
	solved_results.push_back(solution.x[delta_start]);
	solved_results.push_back(solution.x[a_start]);
    for(int i=0; i<N; ++i)
    {
		solved_results.push_back(solution.x[x_start + i]);
		solved_results.push_back(solution.x[y_start + i]);
	}

	return solved_results;
}
