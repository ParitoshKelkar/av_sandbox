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


% tunable segment break up param - for approximating fresnel
segmentFresnel = 10;

init_curvature = initKellyCurvature(start,goal);

integratedState = computeIntegrals(start,init_curvature,goal);



