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
    // the knot points 
    double p0;
    double p1;
    double p2;
    double p3;
    double s;

    // the coeff of the cubic equation
    double a_p;
    double b_p;
    double c_p;
    double d_p;

    CubicSpline()
    {
    }

    CubicSpline(double k0, double k1, double k2, double k3, double s_total)
    {
      p0 = k0; p1 = k1; p2 = k2; p3 = k3; s = s_total;
      // equally spaced points for better stable parameterization 
      a_p = p0;
      b_p = (-0.5)*(11*p0 -18*p1 + 9*p2 -2*p3)/(s);
      c_p = (4.5)*(2*p0 - 5*p1 + 4*p2 - p3)/pow(s,2);
      d_p = (-4.50)*(p0 - 3*p1 + 3*p2 - p3)/pow(s,3);
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

  // without the inline keyword, error was "multiple definition of function".
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
  * @brief  taken from the Nagy Kelly 2001 paper. 
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

  /**
  * @brief  -pi/2 -> pi/2 wrap around 
  *
  * @param  double - angle in radians
  *
  * @return double
  * 
  * @throws Exception
  */
  double constrainAngle(double);
  

};

#endif /* LIBLATTICE_TRAJECTORY_GEN_H */
