/*
* @author ParitoshKelkar 
*
* @version 
*
* @brief  this contains all the functions needed for the integration 
*         of the vehicle motion 
*
*/
#ifndef LIBTRAJ_MOTION_MODEL_H
#define LIBTRAJ_MOTION_MODEL_H

#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <Eigen/Dense>

#include <lattice_trajectory_gen/libtraj_gen_common.h>

namespace common =  libtraj_gen_common;

namespace libtraj_motion_model
{

  std::ofstream fhandler;

  double max_kappa = 0.39;
  double min_kappa = -0.39;

  double kappa_dot_max = 0.1021;
  double kappa_dot_min = -0.1021;

  double max_acc = 2;
  double max_decc = -6;

  double a_scl = 0.1621;
  double b_scl = -0.0049;
  
  double v_scl = 4;
  double kappa_v = 0.1485;
  double v_safety = 2;
  double safety_factor = 1;

  /**
  * @brief  computes horizon and runs motion model until then 
  *
  * @param  const VehicleState start 
  *         const VehicleState goal 
  *         double dt 
  *         const CubicSpline curvature 
  *
  * @return VehicleState
  * 
  * @throws Exception
  */
  common::VehicleState motionModel(const common::VehicleState, const common::VehicleState, double,const common::CubicSpline);


  /**
  * @brief  ** self explanatory **
  *
  * @param  const VehicleState
  *         double
  *         double
  *         const CubicSpline
  *
  * @return float
  * 
  * @throws Exception
  */
  double getNextCurvature(const common::VehicleState,double,double,const common::CubicSpline);


  /**
  * @brief  clamps the speed, curvature etc
  *
  * @param  common::VehicleState
  *
  * @return void
  * 
  * @throws 
  */
  void speedControlLogic(common::VehicleState&);

  /**
  * @brief  clamped response to control inputs
  *
  * @param  common::VehicleState - current state
  *         common::VehicleState - next state
  *         double - dt
  *
  * @return   common::VehicleState
  * 
  * @throws Exception
  */
  common::VehicleState responseToControls(common::VehicleState, common::VehicleState,double);


  /**
  * @brief approximates the inverse jacobian and 
  *        solves for the correction parameter 
  *
  * @param  
  *
  * @return   common::CubicSpline
  * 
  * @throws Exception TODO cant calculate inverse of jacobian 
  */
  common::CubicSpline generateCorrection(const common::VehicleState&, const common::VehicleState&, const common::VehicleState&, double, common::CubicSpline);


  /**
  * @brief  implements simple cubic polynomial 
  *         cont in acc 
  *
  * @param  const common::CubicSpline - curvature 
  *         const common::VehicleState - goal 
  *         const common::VehicleState - start
  *         double - elapsedTime
  *
  * @return double - nextVel
  * 
  * @throws Exception
  */
  double getNextVelocity(const common::CubicSpline, const common::VehicleState, const common::VehicleState, double);



};

#endif /* LIBTRAJ_MOTION_MODEL_H */
