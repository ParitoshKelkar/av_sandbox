%% this script will be used to prototype the MJT trajectories 
%% described in Werling et.all 

% readFile 
map_data = dlmread("udacity_map.csv"," ");

% truncate data to a particular point 
% also truncate 's' value
% will be calculated manually 
truncate_pt = 20;

map_data = map_data(1:truncate_pt,1:2);

% use polyfit to iteratively fit points 
% use interpolation to increase resolution of map 

% first calculate incremental dist along ref line 
s = [0];
for iter = 2 : size(map_data,1)
  
  temp_s = sqrt( (map_data(iter,1) - map_data(iter-1,1))^2 +...
            (map_data(iter,2) - map_data(iter-1,2))^2); 
  s = [s;temp_s];

end

% generate initial fit to data 
% interpolate 
s = cumsum(s);
[x_s,coeffs_x] = fit_1D(map_data(1:4,1),s(1:4));
[y_s, coeffs_y] = fit_1D(map_data(1:4,2),s(1:4));


% use goal on ref line 
% current frenet state 
current_frenet_state.s = 0;
current_frenet_state.d = 1;

current_frenet_state.s_dot = 20; % m/s 
current_frenet_state.d_dot = 0; % m/s 

current_frenet_state.s_ddot = 0; % m/s 
current_frenet_state.d_ddot = 0; % m/s 


goal_frenet_state.s = 45;
goal_frenet_state.d = -1;

goal_frenet_state.s_dot = 20; % m/s 
goal_frenet_state.d_dot = 0; % m/s 

goal_frenet_state.s_ddot = 0; % m/s 
goal_frenet_state.d_ddot = 0; % m/s 

t = 2; % secs

frenet_path = calculate_simple_jmt(current_frenet_state, goal_frenet_state,t);

%frenet_path.s = linspace(0,80,80);
%frenet_path.d = ones(1,80);

% convert to cartesian frame 
frenet_path.x = [];
frenet_path.y = [];


for iter_cnv = 1 : length(frenet_path.s)
  [x_new, y_new] = convertToCartesian(frenet_path.s(iter_cnv), frenet_path.d(iter_cnv), coeffs_x, coeffs_y, t);
  frenet_path.x = [frenet_path.x; x_new];
  frenet_path.y = [frenet_path.y; y_new];
end



% simple plot 
plot(x_s, y_s, 'or','LineWidth',3);
hold on
plot(frenet_path.x, frenet_path.y, 'og','LineWidth',3);

% random obstacle posn 
obs_pos.s = 20;
obs_pos.d = -1;

% generate number of trajectories 
% choose the one with min cost 

apprx_goal_s = 45;
apprx_goal_d = -1;


% not sampling other than position (vel and time not sampled) 

goal_frenet_state.s_dot = 20; % m/s 
goal_frenet_state.d_dot = 0; % m/s 

goal_frenet_state.s_ddot = 0; % m/s 
goal_frenet_state.d_ddot = 0; % m/s 

%frenet_path = []; % list of frenet paths 



%for iter_s = apprx_goal_s - 10 : 2 : apprx_goal_s+ 10
  %for iter_d = apprx_goal_d - 2 : 1 : apprx_goal_d + 2 

    %goal_frenet_state.s = iter_s;
    %goal_frenet_state.d = iter_d;

    %% also fills in 1st and 2nd derv of motion 
    %% frenet_path_temp.sdot/s_ddot & d_dot/d_ddot 
    %frenet_path_temp = calculate_simple_jmt(current_frenet_state, goal_frenet_state, t);
    %frenet_path_temp.x = [];
    %frenet_path_temp.y = [];

    %for iter_cnv = 1 : length(frenet_path_temp.s)
      %[x_new, y_new] = convertToCartesian(frenet_path_temp.s(iter_cnv), frenet_path_temp.d(iter_cnv), coeffs_x, coeffs_y);
      %frenet_path_temp.x = [frenet_path_temp.x; x_new];
      %frenet_path_temp.y = [frenet_path_temp.y; y_new];
    %end

    %% compute costs 
    %frenet_path = [frenet_path;frenet_path_temp];


  %end
%end


 %% elimintate traj that are not feasible before needing to evaluate cost 
 %frenet_path = discardNonFeasiblePaths(frenet_path);

    %frenet_path_temp.proximity_cost = calculate_proximity_cost(frenet_path_temp,obs_pos);
    %frenet_path_temp.jerk_cost = calculate_jerk_cost(frenet_path_temp);
    %frenet_path_temp.feasibility_cost = calculate_feasibility_cost(frenet_path_temp, current_state);
    
