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

frenet_path = calculate_simple_jmt(current_frenet_state, goal_frenet_state);

%frenet_path.s = linspace(0,80,80);
%frenet_path.d = ones(1,80);

% convert to cartesian frame 
frenet_path.x = [];
frenet_path.y = [];

for iter_cnv = 1 : length(frenet_path.s)
  [x_new, y_new] = convertToCartesian(frenet_path.s(iter_cnv), frenet_path.d(iter_cnv), coeffs_x, coeffs_y);
  frenet_path.x = [frenet_path.x; x_new];
  frenet_path.y = [frenet_path.y; y_new];
end



% simple plot 
plot(x_s, y_s, 'or','LineWidth',3);
hold on
plot(frenet_path.x, frenet_path.y, 'og','LineWidth',3);


