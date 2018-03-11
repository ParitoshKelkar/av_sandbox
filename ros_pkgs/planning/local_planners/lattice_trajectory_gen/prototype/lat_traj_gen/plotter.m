% 

x_hist = cell2mat({state_hist.sx});
y_hist = cell2mat({state_hist.sy});
theta_hist = cell2mat({state_hist.theta});
kappa_hist = cell2mat({state_hist.kappa});
vel_hist = cell2mat({state_hist.vel});

plot(x_hist,'-r');
hold on;
plot(y_hist,'-b');
plot(theta_hist,'-k');
plot(kappa_hist,'-g');
plot(vel_hist,'y');

legend('x','y','theta','kappa','vel');

