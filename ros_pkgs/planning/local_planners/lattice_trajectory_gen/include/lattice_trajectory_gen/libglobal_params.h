/*
* @author ParitoshKelkar 
*
* @version 
*
* @brief ** self explanatory **
*
*/
#ifndef LIBGLOBAL_PARAMS_H
#define LIBGLOBAL_PARAMS_H

namespace global_params
{

  float max_kappa = 0.39;
  float min_kappa = -0.39;

  float kappa_dot_max = 0.1021;
  float kappa_dot_min = -0.1021;

  float max_acc = 2;
  float max_decc = -6;

  float a_scl = 0.1621;
  float b_scl = -0.0049;
  
  float v_scl = 4;
  float kappa_v = 0.1485;
  float v_safety = 2;

};

#endif /* LIBGLOBAL_PARAMS_H */
