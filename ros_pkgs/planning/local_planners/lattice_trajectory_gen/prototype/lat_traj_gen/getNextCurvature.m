function [kappa] = getNextCurvature(current_state,dt,t,p) 
 
  % this is for stable cubic splines - 4 parameters with 2 boundary conditions 
  % mc naughton pg 77
  % the curvature points are spaced equally along the spline<?>
  s = p.s;
  si = 0;
  st = current_state.vel*t;

  a = p.a_p;
  b = p.b_p;
  c = p.c_p;
  d = p.d_p;
  k_next_cmd = a + b*st + c*(st^2) + d*(st^3);

  kappa = k_next_cmd;
end



