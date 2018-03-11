function [kappa] = getNextCurvature(current_state,dt,t,p) 
 
  % this is for stable cubic splines - 4 parameters with 2 boundary conditions 
  % mc naughton pg 77
  % the curvature points are spaced equally along the spline<?>
  kappa_0 = p.kappa_0;
  kappa_1 = p.kappa_1;
  kappa_2 = p.kappa_2;
  kappa_3 = p.kappa_3;
  s = p.s;
  si = 0;
  st = current_state.vel*t;

  a = kappa_0;
  b = (-0.50)*(-2*kappa_3 + 11*kappa_0 - 18*kappa_1 + 9*kappa_2)/(s-si);
  c = (4.50)*(-kappa_3 + 2*kappa_0 - 5*kappa_1 +4*kappa_2)/((s-si)^2);
  d = (-4.50)*(-kappa_3 + kappa_0 - 3*kappa_1 + 3*kappa_2)/((s-si)^3);
  k_next_cmd = a + b*st + c*(st^2) + d*(st^3);

  kappa = k_next_cmd;
end



