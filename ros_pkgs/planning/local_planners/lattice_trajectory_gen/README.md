# Trajectory Generation - Cubic Splines  

This readme describes the approach used in the lattice_trajectory_gen to generate cubic splines for vehicle navigation. 

The algorithm and implementation is based on the papers of Nagy, Kelly, Howard, McNaughton.  [Autoware](https://github.com/CPFL/Autoware), [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), were used for software, implementation references. The papers are listed in the **References** section below.

The '**prototype**'  folder contains octave scripts for the code that is implemented in **C++**. 



## Concept 

The motion planning/trajectory generation problem for autonomous vehicles involves generating a set of control commands that transition the vehicle from one feasible pose to another, satisfying constraints enforced by the environment or problem definition. 

The use of splines, curves for feasible trajectory generation has been a topic of research over many years. For this implementation I chose to focus on the a particular approach that is mentioned in **Motion Planning for Urban Environments**.  First mentioned in **Nagy,Kelly 2001** this concept has been studied and matured over the years.

The idea is to control not only the **pose(X,Y,$\theta$)** but also the **posture(X,Y,$\theta$,$\kappa$)** where $\kappa$ is the curvature. The only control variables accessible to us w.r.t autonomous vehicles are **throttle** and **steering**. The problem statement now is to find a set of control inputs that maneuver the car in a manner so as to satisfy the boundary posture constraints.  