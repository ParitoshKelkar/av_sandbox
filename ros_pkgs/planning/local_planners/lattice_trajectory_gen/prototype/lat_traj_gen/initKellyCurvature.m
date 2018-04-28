function [curvature] = initKellyCurvature(start,goal) 

   sx_f = goal.sx;
   sy_f = goal.sy;
   d_theta = abs(goal.theta);
   theta_f = goal.theta;
   kappa_0 = start.kappa;
   kappa_f = goal.kappa;
  
  d = sqrt((sx_f^2) +(sy_f^2));
        
  s = d * (((d_theta^2))/5.0 + 1.0) + (2.0/5.0)*d_theta;
        

  b = (3/((s^2))) * (kappa_0 + kappa_f) + (6*theta_f/((s^3)));
  a = 6*(theta_f)/(s^2) - 2*kappa_0/s  + 4*kappa_f/s;

  % the curvature param is of the form a,b,c,d,s
  % a = init condition - kappa0 
  % remaining unknowns 

  % Kelly 2001 and rewrites the equation with different 
  % constants compared to Kelly 2003.  
  curvature.kappa_0 =  start.kappa;% a w.r.t Kelly 2003 
  curvature.kappa_1 = a; % b w.r.t Kelly 2003 
  curvature.kappa_2 = b;
  curvature.kappa_3 = 0; % 'c' defined as 0 in Kelly 2001 
  curvature.s = s;
  
end

