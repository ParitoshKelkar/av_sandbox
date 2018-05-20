% assumes that the sampled Traj list is already present 
% and is named sampledTrajList 

f = figure();
hold on;
for i = 1 : size(sampledTrajList,2)  
  state_hist = sampledTrajList{i}.state_hist;

  x_hist = cell2mat({state_hist.sx});
  y_hist = cell2mat({state_hist.sy});
  %theta_hist = cell2mat({state_hist.theta});
  %kappa_hist = cell2mat({state_hist.kappa});
  %vel_hist = cell2mat({state_hist.vel});

  plot(x_hist,y_hist,'-b','LineWidth',1.5);
  hold on;
  plot(x_hist(end),y_hist(end),'*r','MarkerSize',3.5);

end
