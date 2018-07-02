function derv = evaluate_third_derv(coeffs, t)
  
  % precomputed equation for quinitic only 
  derv = 6*coeffs(4) + 24*coeffs(5)*t + 60*coeffs(6)*(t*t);
end
