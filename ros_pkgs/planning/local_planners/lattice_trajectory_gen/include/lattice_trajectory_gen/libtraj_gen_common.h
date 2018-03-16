/*
* @author ParitoshKelkar 
*
* @version 
*
* @brief 
*
*/
#ifndef LIBLATTICE_TRAJECTORY_GEN_H
#define LIBLATTICE_TRAJECTORY_GEN_H

#include <iostream>
#include <ros/ros.h>
#include <math.h>

namespace libtraj_gen_common
{


  double x_tolerance = 0.05; //m 
  double y_tolerance = 0.05; //m 
  double theta_tolerance = 0.1; //radians
  double kappa_tolerance = 0.01; // rad/m <?>

 /*
  *this declaration contains the same variable as mentioned in 
  *the dissesration of Howard - diff from the a,b,c,s representation of 
  *Nagy Kelly 
  */
  struct CubicSpline
  {
    double p0;
    double p1;
    double p2;
    double p3;
    double s;

    CubicSpline()
    {
    }
  };

  struct VehicleState 
  {
    double x;
    double y;
    double theta; // radians 
    double kappa;
    double vel;

    VehicleState(double px, double py, double ptheta, double pkappa, double pvel)
    {
      x = px;
      y = py;
      theta = ptheta;
      kappa = pkappa;
      vel = pvel;
    }

    VehicleState()
    {
    }
    
  };

  // without the inline keyword, error was "multiple definition of function". not sure as to why this is resolved with 'inline' keyword 
  // https://stackoverflow.com/questions/12802536/c-multiple-definitions-of-operator - for more information 
  inline std::ostream& operator<<(std::ostream &output, const libtraj_gen_common::CubicSpline& s)
  {
     return output<<" Length "<<s.s<<"; p0="<<s.p0<<"; p1 = "<<s.p1<<"; p2 = "<<s.p2<<";p3 = "<<s.p3<<"\n";
  }

  inline std::ostream& operator<<(std::ostream &output, const libtraj_gen_common::VehicleState& veh)
  {
    return output<<"x="<<veh.x<<"; y="<<veh.y<<"; theta="<<veh.theta<<"; kappa="<<veh.kappa<<"; vel="<<veh.vel<<"\n";
  }


  /**
  * @brief  ** self explanatory **
  *
  * @param  const VehicleState
  *         const VehicleState
  *
  * @return CubicSpline
  * 
  * @throws Exception
  */
  CubicSpline initCurvature(const VehicleState, const VehicleState);


  /**
  * @brief  ** self explanatory **
  *
  * @param  const VehicleState
  *         const VehicleState
  *
  * @return bool
  * 
  * @throws Exception
  */
  bool hasConverged(const VehicleState, const VehicleState);
  

};

#endif /* LIBLATTICE_TRAJECTORY_GEN_H */
