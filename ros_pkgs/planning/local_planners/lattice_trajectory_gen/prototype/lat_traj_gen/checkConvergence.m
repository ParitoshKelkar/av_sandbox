function converged = checkConvergence(s1,s2)
%% s1 and s2 are states - irrelevant as to which is goal/start/integrated state

 x_th = 0.1;
 y_th = 0.1;
 theta_th = pi/20;
 converged = false;

 if ( abs(s1.sx-s2.sx) < x_th && abs(s1.sy - s2.sy) < y_th && abs(s1.theta - s2.theta) < theta_th )
   converged =  true;
 end

end
