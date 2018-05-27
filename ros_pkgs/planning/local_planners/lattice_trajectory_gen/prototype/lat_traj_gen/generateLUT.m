% this script uses the method described in 5.1.4 in mcNaughton 
% to generate the LUT 

% define range of param values 
tic
kappa_resolution = 0.2;
length_resolution = 15;
vel_resolution = 5; % m/s
dt = 0.1;

p1 = [-0.20:kappa_resolution:0.20];
p2 = p1;
p0 = p1;
p3 = p1;

s = [5:length_resolution:25]; 

vel = [1:vel_resolution:10]; 

% generate every combination 
[p0_comb, p1_comb,p2_comb, p3_comb, s_comb,v_comb] = ndgrid(p0,p1,p2,p3,s,vel);
all_comb = [p0_comb(:), p1_comb(:), p2_comb(:), p3_comb(:), s_comb(:), v_comb(:)];

sampledTrajList = {}; 

% iterate through every combination 
for i = 1 : size(all_comb,1)
  spline = makeCubicSpline(all_comb(i,1),all_comb(i,2), all_comb(i,3), all_comb(i,4),all_comb(i,5));

  % use this spline for running motion model 
  start.sx = start.sy = start.theta = 0;
  start.kappa = all_comb(i,1); % init curvature
  start.vel = all_comb(i,6); % vel 

  [final_state,state_hist] = sampleTrajectory(start,dt,spline);

  sampledTrajList{i}.final_state = final_state;
  sampledTrajList{i}.state_hist = state_hist;
  sampledTrajList{i}.spline = spline;

end

disp("done sampling all trajectories for parameter ranges");


% generate table for possible end targets
pos_resolution = 15;
theta_resolution = pi/2;

x = [1 : pos_resolution : 25];
y = [-10 : pos_resolution : 10];
theta = [ -pi/2 : theta_resolution : pi/2];
% use p0 and p3 from earlier 

[x_comb, y_comb, theta_comb, k0_comb, v_comb] = ndgrid(x,y,theta,p0,vel);
all_target_states = [x_comb(:), y_comb(:), theta_comb(:), k0_comb(:), kf_comb(:), v_comb(:)];
target_state_to_param_tracker = 1000*ones(size(all_target_states,1),2);

% iterate over sampled traj and associate them with a table idx 
for iter_traj = 1 : size(sampledTrajList,2)
   
  for iter_LUT = 1 : size(all_target_states,1)
      
          if (all_target_states(iter_LUT,4) == sampledTrajList{iter_traj}.spline.p0 && ...
              all_target_states(iter_LUT,5) == sampledTrajList{iter_traj}.spline.p3 && ...
              all_target_states(iter_LUT,6) == sampledTrajList{iter_traj}.final_state.vel)

              trajScore = calcTrajScoreLUT([all_target_states(iter_LUT,1), all_target_states(iter_LUT,2), all_target_states(iter_LUT,3)], sampledTrajList{iter_traj}.final_state);
              if trajScore < target_state_to_param_tracker(iter_LUT,2)
                      target_state_to_param_tracker(iter_LUT,1) = iter_traj;
                      target_state_to_param_tracker(iter_LUT,2) = trajScore;
              end
          end

  end


end

% plotting 
plotTargetStateSampledTraj;

toc
