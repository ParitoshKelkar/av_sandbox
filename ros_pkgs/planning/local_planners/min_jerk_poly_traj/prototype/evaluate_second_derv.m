function derv = evaluate_second_derv(coeffs, t)
  
  % precomputed equation for quinitic only 
  derv = 2*coeffs(3) + 6*coeffs(4)*(t) + 12*coeffs(5)*(t^2) + 20*coeffs(6)*(t^3);
end
