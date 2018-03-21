
function delta_param = generateCorrection(start,goal,integrated_state,dt,p_init)

% approx jacobian 
% the jacobian is of the form - 4 x 4; the state doesnt include - following nagy kelly 
% verify mc naughton implementation - kappa not included 
a_perturb = 0.001;
b_perturb = 0.001;
c_perturb = 0.001;
s_perturb = 0.01;

J = zeros(3,3);
% second parameter col 
p_a_perturb = p_init;
p_a_perturb.kappa_1 = p_a_perturb.kappa_1 + a_perturb;
col1_state = motionModel(start,goal,dt,p_a_perturb);
%J(:,1) = (1/a_perturb)*[col1_state.sx;col1_state.sy;col1_state.theta;col1_state.kappa];
delta_J_state = [integrated_state.sx - col1_state.sx; integrated_state.sy - col1_state.sy; integrated_state.theta - col1_state.theta];
J(:,2) = (1/a_perturb)*delta_J_state;


% third parameter col 
p_b_perturb = p_init;
p_b_perturb.kappa_2 = p_b_perturb.kappa_2 + b_perturb;
col2_state = motionModel(start,goal,dt,p_b_perturb);
%J(:,2) = (1/b_perturb)*[col2_state.sx;col2_state.sy;col2_state.theta;col2_state.kappa];
delta_J_state = [integrated_state.sx - col2_state.sx; integrated_state.sy - col2_state.sy; integrated_state.theta - col2_state.theta];
J(:,3) = (1/b_perturb)*delta_J_state;


% third parameter col 
%{
 {p_c_perturb = p_init;
 {p_c_perturb.kappa_3 = p_c_perturb.kappa_3 + c_perturb;
 {col3_state = motionModel(start,goal,dt,p_c_perturb);
 {J(:,3) = (1/c_perturb)*[col3_state.sx;col3_state.sy;col3_state.theta;col3_state.kappa];
 %}


% first parameter col 
p_s_perturb = p_init;
p_s_perturb.s = p_s_perturb.s + s_perturb;
col4_state = motionModel(start,goal,dt,p_s_perturb);
%J(:,4) = (1/s_perturb)*[col4_state.sx;col4_state.sy;col4_state.theta;col4_state.kappa];
%J(:,4) = (1/s_perturb)*[col4_state.sx;col4_state.sy;col4_state.theta;];
delta_J_state = [integrated_state.sx - col4_state.sx; integrated_state.sy - col4_state.sy; integrated_state.theta - col4_state.theta];
J(:,1) = (1/s_perturb)*delta_J_state;


% the difference of the predicted state vs goal 
delta_state.sx  = goal.sx - integrated_state.sx;
delta_state.sy  = goal.sy - integrated_state.sy;
delta_state.theta  = goal.theta - integrated_state.theta;
delta_state.kappa  = goal.kappa - integrated_state.kappa;
delta_state.vel = 0.1;

disp(integrated_state);

%delta_param = inv(J)*[delta_state.sx;delta_state.sy;delta_state.theta;delta_state.kappa];
delta_param = inv(J)*[delta_state.sx;delta_state.sy;delta_state.theta;];


end

