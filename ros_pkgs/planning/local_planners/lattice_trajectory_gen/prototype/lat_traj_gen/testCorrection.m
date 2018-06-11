% this script is to validate the optimazation 
% of the spline
 
start.sx = 0;
start.sy = 0;
start.theta = 0;
start.kappa = 0.0;
start.vel = 3;


goal.sx = 16;
goal.sy = 5;
goal.theta = 1.57;
goal.kappa = 0.2;
goal.vel = 3;

dt = 0.1;

% the initial guess is from LUT 
init_curvature = makeCubicSpline(start.kappa,0,0.0,goal.kappa,20);
%init_curvature = makeCubicSpline(0.0,0,0.0,0,6);

[integrated_state,state_hist] = motionModel(start,goal,dt,init_curvature);
disp("Done Prediction");
disp(integrated_state);

disp("Generating correction");

iter = 0;
while (iter < 5)
disp("iter");
param = generateCorrection(start,goal,integrated_state,dt,init_curvature);

p.p0 = start.kappa;
p.p1 = init_curvature.p1 + param(2);
p.p2 = init_curvature.p2 + param(3);
p.s =(init_curvature.s + param(1));
p.p3 = goal.kappa;

p = makeCubicSpline(p.p0, p.p1, p.p2,p.p3,p.s);

[new_state,state_hist] = motionModel(start,goal,dt,p);
init_curvature = p;
disp(param)
disp(p);
disp(new_state);
integrated_state = new_state;

iter = iter + 1;
end


