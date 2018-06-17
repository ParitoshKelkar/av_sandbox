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
pos_resolution = 10;
theta_resolution = pi/4;

x = [5 : pos_resolution : 25];
y = [-10 : pos_resolution : 10];
theta = [ -pi/4 : theta_resolution : pi/4];
% use p0 and p3 from earlier 

plot_vel = 6;
plot_p0 = 0;

[x_comb, y_comb, theta_comb, k0_comb,kf_comb v_comb] = ndgrid(x,y,theta,plot_p0,p3,plot_vel);
all_target_states = [x_comb(:), y_comb(:), theta_comb(:), k0_comb(:), kf_comb(:),v_comb(:)];
target_state_to_param_tracker = 1000*ones(size(all_target_states,1),2);

% iterate over sampled traj and associate them with a table idx 
for iter_LUT = 1 : size(all_target_states,1)
   
  for iter_traj = 1 : size(sampledTrajList,2)
      
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


% now to optimize every traj in sampledTrajList to meet target end pt constraints 
% iterate through target_state_to_param_tracker 
disp("Optimizing");
res = zeros(1,length(target_state_to_param_tracker));

for iter = 1 : length(target_state_to_param_tracker)

  if (target_state_to_param_tracker(iter,1) == 1000)
    continue;
  end

  if (all_target_states(iter,6) == plot_vel && all_target_states(iter,4) == plot_p0) 

    curvature = sampledTrajList{target_state_to_param_tracker(iter,1)}.spline;
    start.sx = 0; start.sy = 0; start.theta = 0; start.kappa = all_target_states(iter,4); start.vel = all_target_states(iter,6);
    goal.kappa = curvature.p3; goal.sx = all_target_states(iter,1);goal.sy =  all_target_states(iter,2); goal.theta = all_target_states(iter,3); goal.vel = all_target_states(iter,6);

    %if (iter > 281)
      %disp('here')
    %end

    [optimized_curvature,res_flag] = optimizeTraj(start,goal,curvature);

    sampledTrajList{target_state_to_param_tracker(iter,1)}.spline = optimized_curvature;
    if (res_flag == 1)
      [final_state,state_hist] = sampleTrajectory(start,dt,optimized_curvature);
      sampledTrajList{target_state_to_param_tracker(iter,1)}.final_state = final_state;
      sampledTrajList{target_state_to_param_tracker(iter,1)}.state_hist = state_hist;
    end

    res(iter) = res_flag;

  end


  disp(iter)

end

% plotting 
plotTargetStateSampledTraj;
toc