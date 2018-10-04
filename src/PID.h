#ifndef PID_H
#define PID_H
//#include <iostream>
#include <vector>
#include <string>
using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  string pid_name;
  /*
  * Twiddle variables
  */
  int step;
  int n_steps;
  int i;
  int state;

  double epoch_cte;
  double lowest_cte;
  double prev_cte;
  std::vector<int> update;
  std::vector<double> p;
  std::vector<double> dp;




  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, string pid_name);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, bool twiddle);
  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
