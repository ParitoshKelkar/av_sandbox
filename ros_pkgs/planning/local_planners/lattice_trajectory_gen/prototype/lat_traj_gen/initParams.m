function [curvature] = initParams(start,goal) 

   sx_f = goal.sx;
   sy_f = goal.sy;
   d_theta = abs(goal.theta);
   theta_f = goal.theta;
   kappa_0 = start.kappa;
   kappa_f = goal.kappa;
  
  d = sqrt((sx_f^2) +(sy_f^2));
        
  s = d * (((d_theta^2))/5.0 + 1.0) + (2.0/5.0)*d_theta;
        

 b = (3/((s^2))) * (kappa_0 + kappa_f) + (6*theta_f/((s^3)));

 %disp(d); disp(s);
 %disp(b);

  si=0.00;
 curvature.kappa_0 = start.kappa;
 curvature.kappa_1=(1.00/49.00)*(8.00*b*si - 26.00*kappa_0 - 0);
 curvature.kappa_2=0.25*(-2.00*kappa_0 +5.00*curvature.kappa_1);
 curvature.kappa_3 = goal.kappa;
 curvature.s = s;

        %curvature.kappa_1=(1.00/49.00)*(8.00*b*si - 8.00*b*curvature.s - 26.00*curvature.kappa_0 - curvature.kappa_3);
        %curvature.kappa_2=0.25*(curvature.kappa_3 -2.00*curvature.kappa_0 +5.00*curvature.kappa_1);
  
end

