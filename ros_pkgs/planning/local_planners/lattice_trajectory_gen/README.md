# Trajectory Generation - Cubic Splines  

This readme describes the approach used in the lattice_trajectory_gen to generate cubic splines for non-holonomic trajectory generation. 

The algorithm and implementation is based on the papers of Nagy, Kelly, Howard, McNaughton.  [Autoware](https://github.com/CPFL/Autoware), [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), were used for software, implementation references. The papers are listed in the [References](#References) section below.

The '**prototype**'  folder contains octave scripts for the code that is implemented in **C++**. 



## Concept 

The motion planning/trajectory generation problem for autonomous vehicles involves generating a set of control commands that transition the vehicle from one feasible pose to another, satisfying constraints enforced by the environment or problem definition. 

The use of splines, curves for feasible trajectory generation has been a topic of research over many years. For this implementation I chose to focus on the a particular approach that is mentioned in [2].  First mentioned in [1] this concept has been studied and matured over the years.

The idea is to control not only the **pose(X,Y,$\theta$)** but also the **posture(X,Y,$\theta$,$\kappa$)** where $\kappa$ is the curvature of the vehicle. The only control variables accessible to us w.r.t the vehicle are **throttle** and **steering**. The problem statement now is to find a set of control inputs that maneuver the car in a manner so as to satisfy the boundary posture constraints while respecting the feasibility of  the maneuvers. A simplified version of the relevant state equations[1] : 
$$
\dot{x}=V(t)cos(\theta(t))\\
\dot{y}=V(t)sin(\theta(t)) \\
\dot{\theta}=\kappa(t)V(t)\\ 
\dot{\kappa}=\dot{\alpha}(t)/L
$$
$L$ is the wheelbase, 

$V(t)$ is the longitudinal velocity 

$\dot{\alpha}(t)$ is the rate of steering input.

Here, we see that we have $2$ control inputs for $4$ state output variables. 

The trajectory of the vehicle satisfying the necessary constraints is represented by a cubic spline. The shape of the trajectory of a vehicle depends largely on the angular velocity/steering rate than the longitudinal velocity. Keeping this in mind, the cubic spline is a representation of the curvature of the vehicle along the trajectory. [1]
$$
\kappa(s) = \kappa_0 + as + bs^2 + cs^3
$$
where $s$ is the arc length. The state equations are now represented as[1]
$$
\theta(s)=\theta_0 + \int_{0}^{S}\kappa(s) = \theta_0 + \kappa_0s + \frac{as^2}{2} + \frac{bs^3}{3} +  \frac{cs^4}{4}\\
x(s)=x_0 + \int_{0}^{S}cos\theta(s)\\
y(s)=y_0 + \int_{0}^{S}sin\theta(s)
$$
We essentially have $4$ control variables $\vec{p}=(a,b,c,s)$ and $4$ output variables $\vec{X} =(x, y, \theta, \kappa)$. [1]
$$
x =f(a,b,c,s)\\
y =f(a,b,c,s)\\
\theta =f(a,b,c,s)\\
\kappa=f(a,b,c,s)\\
$$
Writing this system in vector form, 
$$
\vec{{X}} = f(\vec{p}),\\
\Rightarrow  \vec{\Delta {x}} = (\frac{\partial }{\partial \vec{p} }f)\Delta \vec{p} \\
\Rightarrow  \vec{\Delta {p}} = (\frac{\partial }{\partial \vec{p}}f)^{-1}\vec{\Delta x } \\
$$
Using an initial guess of control parameters $\vec{p}$,  a final state $\vec{X}$ is calculated that is close to the final state $\vec{X_{desired}}$ . Referencing the above equations, once $\vec{\Delta x}$ is calculated, the jacobian of the forward integration function $f$ is inverted and multiplied with $\vec{\Delta x}$. The $\vec{\Delta p}$ calculated is added to the initial guess and the process is repeated till $\vec{X} = \vec{X_{desired}}$  within some convergence criteria. 

One of the advantages of this method of representation is that the cubic spline is continuous to the third differential of curvature that leads to smooth steer velocities and torque. 



## Approach 

The research papers [1]-[5] describe implementations of this concept modified/improved in different ways. 

As described in [4],[5] the parameter set as shown in Equation 2 face a discrepancy in in magnitude as the coefficient of $s^3$ has to be much smaller than the coefficient of $s$. This is later problematic when it comes to inverting the Jacobian. For numerical stability, the spline is reparameterized as 
$$
\kappa(s) = a(p) + b(p)s + c(p)s^2 + d(p)s^3
$$
where $\vec{p}=(p_0,p_1,p_2,p_3,s)$ and the parameters are constrained to be equal to the path curvature at equal intervals of $s$. i.e,
$$
p_0 = \kappa_0 \\
p_1 = \kappa(s/3)\\
p_2 = \kappa(2s/3)\\
p_3 = \kappa(S) = \kappa_f
$$
Since the $\kappa$ of the vehicle is bounded by its steering limits, the parameters $p_0..p_3$ are of the same magnitude leading to a more stable path model. 

The coefficients of the spline can be solved using Equation set 7 as described in [5].  
$$
a(p) = \kappa_0=p_0\\
d(p)=\kappa_f = p3
$$
Some of these coefficients are trivially solved and the free parameters left to solve for are 
$$
\vec{p_{free}} = (p_1,p_2,s)
$$
The optimization consists of forward integration, using an initial approximate for the control variables. 

As suggested in most of the research, a LUT(look-up-table) is used to determine the initial guess. 

The LUT maps the relation between control parameters and state space. 

In [2],[5] the forward integration is carried out using Fresnel integrals($x(s), y(s)$) and closed form solution for $\theta$ and $\kappa$ . In [3] however a motion model of the vehicle is used for forward integration. This particular implementation uses the motion model approach.

## References  

[1] Nagy, Bryan, and Alonzo Kelly. "Trajectory generation for car-like robots using cubic curvature polynomials." *Field and Service Robots* 11 (2001).

[2] Howard, Thomas M., and Alonzo Kelly. "Optimal rough terrain trajectory generation for wheeled mobile robots." *The International Journal of Robotics Research* 26.2 (2007): 141-166

[3] Ferguson, Dave, Thomas M. Howard, and Maxim Likhachev. "Motion planning in urban environments." *Journal of Field Robotics* 25.11‐12 (2008): 939-960.

[4] Howard, Thomas M. *Adaptive model-predictive motion planning for navigation in complex environments*. No. CMU-RI-TR-09-32. CARNEGIE-MELLON UNIV PITTSBURGH PA ROBOTICS INST, 2009.

[5] McNaughton, Matthew. *Parallel algorithms for real-time motion planning*. Carnegie Mellon University, 2011.



