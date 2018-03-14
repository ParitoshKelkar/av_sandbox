% this function computes the prediction of the trajectory over a particular set of {start,goal} states

dt = 0.1; % 10Hz 
globalParams;

start.sx = 0;
start.sy = 0;
start.theta = 0;
start.kappa = 0.0;
start.vel = 0.1;


%goal.sx = 6;
%goal.sy = 4;
%goal.theta = 0;
%goal.kappa = 0.2;
%goal.vel = 0.1;

goal.sx = 1;
goal.sy = 1;
goal.theta = 0;
goal.kappa = 0.1;
goal.vel = 0.1;

t = dt;
horizon = sqrt((start.sx - goal.sx)^2 + (start.sy - goal.sy)^2)/goal.vel;
current_state = start;
state_hist =[start];

% initial params 

p_init = initParams(start,goal)
disp("Initializing integration");
[integrated_state,state_hist] = motionModel(start,goal,dt,p_init);
disp("Done Prediction");
disp(integrated_state);

disp("Generating correction");

iter = 0;
while (iter < 5)
disp("iter");
param = generateCorrection(start,goal,integrated_state,dt,p_init);

p.kappa_0 = start.kappa;
p.kappa_1 = p_init.kappa_1 - param(2);
p.kappa_2 = p_init.kappa_2 - param(3);
p.kappa_3 = goal.kappa;
p.s = p_init.s- param(1);

[new_state,state_hist] = motionModel(start,goal,dt,p);
p_init = p;
disp(param)
disp(p);
disp(new_state);
integrated_state = new_state;

iter = iter + 1;

end
