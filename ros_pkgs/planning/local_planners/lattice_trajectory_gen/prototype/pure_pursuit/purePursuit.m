% @brief - pure pursuit basic implementation
% assumes that the waypoints are loaded in 'wawypoints'
% mainly follows the steps in Coulter CMU - 1992 


% step 1 - get current veh location in global frame
current_veh_pose = [1,1,0.5];


% get closest path point 
% structures like the kdtree could be used 
prev_closest_waypoint = 0; 
min_dist = -1; 
% going to assume mostly straight roads 





