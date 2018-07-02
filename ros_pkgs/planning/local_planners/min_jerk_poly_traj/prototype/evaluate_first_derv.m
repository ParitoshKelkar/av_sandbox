function derv = evaluate_first_derv(coeffs, t)
  
  % precomputed equation for quinitic only 
  derv = coeffs(2) + 2*coeffs(3)*(t) + 3*coeffs(4)*(t^2) + 4*coeffs(5)*(t^3) + 5*coeffs(6)*(t^4);

end
