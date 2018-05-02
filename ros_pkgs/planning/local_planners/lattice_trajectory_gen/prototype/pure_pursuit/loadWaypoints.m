% @brief - responsible for loading custom waypoints 
% 

% spacing of waypoints 
res = 1; %mtrs 
lookahead_dist = 7; %mtrs 

% straight_line 
waypoints = [0,0,0];
num_waypts = 40;
iter = 0;

while (iter <= num_waypts)
  new_waypoint = waypoints(end,1:2) + [ 1 0 ];
  new_wp_heading = atan2(new_waypoint(2)-waypoints(end,2), new_waypoint(1)-waypoints(end,1)); % radians 
  waypoints = [waypoints; [new_waypoint, new_wp_heading]];
  iter = iter + 1;
end

disp('Done generating waypoints');

