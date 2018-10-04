#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, string pid_name_) {
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	i = 0;
	n_steps = 1000;
	step = 0;
	state = 0;
	lowest_cte = 5000000;
	p = {Kp_, Ki_, Kd_};
	dp = {p[0] / 10, p[1] / 10, p[2] / 10};
	update = {1, 1, 1};
	prev_cte = 0;
	pid_name = pid_name_;
}

void PID::UpdateError(double cte, bool twiddle) {
	step += 1;
	if(step == 1){
		prev_cte = cte;
	}
	if (twiddle && update[i] == 1 && step >= n_steps / 2) {
		std::cout << pid_name << ": p[" << i << "] = " << p[i];
		p[i] += dp[i];
		update[i] = 0;
		std::cout << ", dp[" << i << "] = " << dp[i] << ", Lets first try p[" << i << "] += dp[" << i << "] = " << p[i] << std::endl;
	}
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte;
    //std::cout << pid_name << ": " << -Kp*p_error << ", " << -Ki*i_error << ", " << -Kd*d_error << std::endl;
	epoch_cte += cte * cte;
	if (step < n_steps / 2) {
		lowest_cte = epoch_cte / step;
	}

	if (twiddle && (step % n_steps == 0) && (step > 0)) {
		epoch_cte /= n_steps;
		std::cout << pid_name << ": i = " << i << ", step = " << step;
		std::cout << ", epoch_cte = " << epoch_cte << ", lowest_cte = " << lowest_cte << std::endl;
		if (epoch_cte < lowest_cte) {
			std::cout << pid_name << ": Success! accepting p[" << i << "] = " << p[i] << ", updating dp[" << i << "] from " << dp[i];

			lowest_cte = epoch_cte;
			dp[i] *= 1.1;
			std::cout << " to " << dp[i];
			state = 0;
			update[i] = 1;
			i = (i + 1) % 3;
			std::cout << ", moving to i = " << i << std::endl;
			std::cout << "---------------------------------------------------\n";
		} else {
			if (state == 0) {
				p[i] -= 2 * dp[i];
				std::cout << pid_name << ": Oops, now lets try p[" << i << "] -= 2*dp[" << i << "] = " << p[i] << std::endl;
				state = 1;
			} else if (state == 1) {
				std::cout << pid_name << ": Failure, reverting p[" << i << "] from " << p[i] << " to ";
				p[i] += dp[i];
				std::cout << p[i] << " and changing dp[" << i << "] from " << dp[i] << " to ";
				dp[i] *= 0.9;
				std::cout << dp[i];
				update[i] = 1;
				i = (i + 1) % 3;
				state = 0;
				std::cout << ", moving to i = " << i << std::endl;
				std::cout << "---------------------------------------------------\n";
			}
			epoch_cte = 0;
		}
	}
	prev_cte = cte;
}

double PID::TotalError() {
}

