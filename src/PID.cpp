#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  PID_parameters[0] = Kp_;
  PID_parameters[1] = Ki_;
  PID_parameters[2] = Kd_;

  dParameters[0] = Kp_ * 0.1;
  dParameters[1] = Ki_ * 0.1;
  dParameters[2] = Kd_ * 0.1;

  p_error = 0.0;
  count = 1;
  max_count = 1500;
  error = 0.0;
  best_error = 0.0;
  rmse = 0.0;

  tolerance = 0.01;
  udpate_ready = false;
  init = false;
  second_check = false;
  iterations = 0;
  dp_count = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // Error update
  d_error = cte - p_error;

  i_error += cte;
  p_error = cte;

  error += cte * cte;
  count++;

  double sum_dp = dParameters[0] + dParameters[1] + dParameters[2];

  if(count > max_count)
  {
    rmse = error / static_cast<double>(count);
    error = 0.0;
    count = 1;
    if(init)
    {
      udpate_ready = true;
    } else{
      best_error = rmse;
//      std::cout << "Init best_error: " << best_error << std::endl;
      PID_parameters[dp_count] += dParameters[dp_count];
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count())/1000.0
              << " s" << std::endl;
    std::cout << "RMSE: " << rmse << "  sum_dp: " << sum_dp << std::endl;
    std::cout << "Kp: " << PID_parameters[0] << " Ki: " << PID_parameters[1] << " Kd: " << PID_parameters[2] << std::endl;
    std::cout << "dP: " << dParameters[0] << " dI: " << dParameters[1] << " dD: " << dParameters[2] << std::endl;
    init = true;
    begin = end;
  }




  if (udpate_ready && sum_dp > tolerance)
  {


//    std::cout << "Iteration: " << iterations << std::endl;
//    iterations++;

    if(!second_check)
    {
      if (rmse < best_error)
      {
        best_error = rmse;
        std::cout << "best_error: " << best_error << std::endl;
        dParameters[dp_count] *= 1.1;
      }
      else{
        PID_parameters[dp_count] -= 2 * dParameters[dp_count];
        udpate_ready = false;
        second_check = true;
        return;
      }
    } else{
      if (rmse < best_error)
      {
        best_error = rmse;
        std::cout << "best_error: " << best_error << std::endl;
        dParameters[dp_count] *= 1.1;
      } else{
        PID_parameters[dp_count] += dParameters[dp_count];
        dParameters[dp_count] *= 0.9;
      }
      second_check = false;
    }

    dp_count++;
    if(dp_count > 2)
      dp_count = 0;

    PID_parameters[dp_count] += dParameters[dp_count];

  }

  udpate_ready = false;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // TODO: Add your total error calc here!
  
  return -PID_parameters[0] * p_error - PID_parameters[2] * d_error - PID_parameters[1] * i_error;
}