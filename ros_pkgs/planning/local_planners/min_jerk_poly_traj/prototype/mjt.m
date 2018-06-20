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

% check xy-frenet conversion 


