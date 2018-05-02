% this approach seperates the velocity gneration from the traj
% generation. Approach described in Kelly and Dolan works in 2012 
% motion planning in urban env uses vel to do fmm prediction 
% this uses numerical quadrature  and other methods to estimate the 
% fmm solution 

dt = 0.1; % 10Hz 
globalParams;

start.sx = 0;
start.sy = 0;
start.theta = 0;
start.kappa = 0.0;
start.vel = 0.1;


goal.sx = 6;
goal.sy = 4;
goal.theta = 0;
goal.kappa = 0.2;
goal.vel = 2;



init_curvature = initKellyCurvature(start,goal);
integrated_state = computeIntegrals(start,init_curvature,goal);

% generate correction 
dt = 0.1; % not actually used in the function
disp('Generating Correction');
param = generateCorrection_nonVel(start,goal,integrated_state,dt,init_curvature);
p.kappa_0 = start.kappa;
p.kappa_1 = init_curvature.kappa_1 + param(2);
p.kappa_2 = init_curvature.kappa_2 + param(3);
p.kappa_3 = goal.kappa;
p.s = init_curvature.s+ param(1);
new_state = computeIntegrals(start,p,goal);
disp('Done Correction');
