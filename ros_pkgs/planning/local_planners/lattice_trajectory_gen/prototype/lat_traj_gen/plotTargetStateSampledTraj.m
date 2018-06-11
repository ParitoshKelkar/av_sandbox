% assumes that the target_state_to_param_tracker is initialized and is 
% a valid solution 
% also assumes that sampledTrajList and all_target_states are available and are valid 
% also assumes res to be present indicating valid optimized trajectories

f = figure();
hold on;

% plot only for one velocity 
plot_vel = 6;
plot_p0 = 0;

for i = 1 : size(target_state_to_param_tracker,1)

  if (res(i) == 0 || all_target_states(i,6) ~= plot_vel || all_target_states(i,4) ~= plot_p0) 
    continue;
  end

  traj_num = target_state_to_param_tracker(i,1);
  
  plot(all_target_states(i,1), all_target_states(i,2),'ob','MarkerSize',5.5, 'MarkerFaceColor','b');
  hold on;

  state_hist = sampledTrajList{traj_num}.state_hist;

  x_hist = cell2mat({state_hist.sx});
  y_hist = cell2mat({state_hist.sy});

  plot(x_hist,y_hist,'-r','LineWidth',1.5);

end
