function delta_param = generateCorrection(start,goal,integrated_state,dt,p_init)

% using forward differencing 
% approx jacobian 
p1_perturb = 0.02;
p2_perturb = 0.02;
s_perturb = 0.50;

J = zeros(3,3);

delta_integrated_state = [goal.sx - integrated_state.sx; goal.sy - integrated_state.sy; goal.theta  - integrated_state.theta];
% first parameter col 
p_s_perturb = makeCubicSpline(p_init.p0, p_init.p1, p_init.p2,p_init.p3, p_init.s+s_perturb);
col1_state = motionModel(start,goal,dt,p_s_perturb);
delta_perturb_state = [goal.sx - col1_state.sx; goal.sy - col1_state.sy; goal.theta  - col1_state.theta];

delta_J_state = delta_perturb_state - delta_integrated_state;
J(:,1) = (1/s_perturb)*delta_J_state;


% second parameter col 
p_p1_perturb = makeCubicSpline(p_init.p0, p_init.p1 + p1_perturb, p_init.p2, p_init.p3, p_init.s);
col2_state = motionModel(start,goal,dt,p_p1_perturb);
delta_perturb_state = [goal.sx - col2_state.sx; goal.sy - col2_state.sy; goal.theta  - col2_state.theta];

delta_J_state = delta_perturb_state - delta_integrated_state;
J(:,2) = (1/p1_perturb)*delta_J_state;

% third parameter col 
p_p2_perturb = makeCubicSpline(p_init.p0, p_init.p1, p_init.p2 + p2_perturb, p_init.p3, p_init.s);
col3_state = motionModel(start,goal,dt,p_p2_perturb);
delta_perturb_state = [goal.sx - col3_state.sx; goal.sy - col3_state.sy; goal.theta  - col3_state.theta];

delta_J_state = delta_perturb_state - delta_integrated_state;
J(:,3) = (1/p2_perturb)*delta_J_state;

% the difference of the predicted state vs goal 
delta_state.sx  = goal.sx - integrated_state.sx;
delta_state.sy  = goal.sy - integrated_state.sy;
delta_state.theta  = goal.theta - integrated_state.theta;
delta_state.kappa  = goal.kappa - integrated_state.kappa;


%delta_param = inv(J)*[delta_state.sx;delta_state.sy;delta_state.theta;delta_state.kappa];
delta_param = -inv(J)*[delta_state.sx;delta_state.sy;delta_state.theta;];


end

