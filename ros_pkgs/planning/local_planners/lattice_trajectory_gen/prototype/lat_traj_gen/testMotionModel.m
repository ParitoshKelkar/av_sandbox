% this script is to validate the motion model only 
 
start.sx = 0;
start.sy = 0;
start.theta = 0;
start.kappa = 0.2;
start.vel = 3;


goal.sx = 16;
goal.sy = 5;
goal.theta = 1.57;
goal.kappa = 0.2;
goal.vel = 3;

dt = 0.1;

% the initial guess is from LUT 
init_curvature = makeCubicSpline(0.2,0,-0.2,0.2,20);
%init_curvature = makeCubicSpline(0.0,0,0.0,0,6);

[integrated_state,state_hist] = motionModel(start,goal,dt,init_curvature);
disp("Done Prediction");
disp(integrated_state);
plotter

%% plot individual 
%f = figure()
%plot(x_hist, y_hist,'r', 'LineWidth',3);

