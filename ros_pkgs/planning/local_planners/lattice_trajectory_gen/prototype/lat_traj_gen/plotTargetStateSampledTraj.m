% assumes that the target_state_to_param_tracker is initialized and is 
% a valid solution 
% also assumes that sampledTrajList and all_target_states are available and are valid 

f = figure();
hold on;

for i = 1 : size(target_state_to_param_tracker,1)
  traj_num = target_state_to_param_tracker(i,1);
  
  plot(all_target_states(i,1), all_target_states(i,2),'*b','MarkerSize',3.5);
  hold on;

  state_hist = sampledTrajList{traj_num}.state_hist;

  x_hist = cell2mat({state_hist.sx});
  y_hist = cell2mat({state_hist.sy});

  plot(x_hist,y_hist,'-r','LineWidth',1.5);

end
