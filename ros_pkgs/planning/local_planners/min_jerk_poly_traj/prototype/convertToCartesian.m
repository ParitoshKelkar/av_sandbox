function [x_new,y_new] = convertToCartesian(s,d, coeff_xs, coeff_ys)
 
  % first calculate ref x and ref y 
  % for 's' and 'd=0' coordinates 

  rx = evaluate_poly(1,2,coeff_xs,s,2);
  ry = evaluate_poly(2,2,coeff_ys,s,2);

  % calculate component of d in global cartesian frame 

  % calculate ref yaw 
  % dx/dy = dx/ds * ds/dy  -> slope - then convert to radians
  rdx = evaluate_derv_cubic(coeff_xs,s);
  rdy = evaluate_derv_cubic(coeff_ys,s);
  r_dxdy = rdx/rdy; % value of slope

  % calculate angle subtended 
  r_yaw = atan2(rdy,rdx); % radians 

  % y vector addition
  x_new = rx + d*cos(r_yaw + pi/2);
  y_new = ry + d*sin(r_yaw + pi/2);

end
