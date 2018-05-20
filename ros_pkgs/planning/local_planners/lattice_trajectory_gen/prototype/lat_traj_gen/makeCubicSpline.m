function spline = makeCubicSpline(p0,p1,p2,p3,s)

  % this is for stable cubic splines - 4 parameters with 2 boundary conditions 
  % mc naughton pg 77
  % the curvature points are spaced equally along the spline<?>
  kappa_0 = p0;
  kappa_1 = p1;
  kappa_2 = p2;
  kappa_3 = p3;
  si = 0;

  a_p = kappa_0;
  b_p = (-0.50)*(-2*kappa_3 + 11*kappa_0 - 18*kappa_1 + 9*kappa_2)/(s-si);
  c_p = (4.50)*(-kappa_3 + 2*kappa_0 - 5*kappa_1 +4*kappa_2)/((s-si)^2);
  d_p = (-4.50)*(-kappa_3 + kappa_0 - 3*kappa_1 + 3*kappa_2)/((s-si)^3);

  % calculated spline eq coeff 
  spline.a_p = a_p;
  spline.b_p = b_p;
  spline.c_p = c_p;
  spline.d_p = d_p;

  % params - spline knot points
  spline.p0 = p0;
  spline.p1 = p1;
  spline.p2 = p2;
  spline.p3 = p3;

  spline.s = s;


end 
