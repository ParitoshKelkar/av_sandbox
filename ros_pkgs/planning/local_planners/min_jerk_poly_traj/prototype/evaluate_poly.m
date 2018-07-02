function val = evaluate_poly(coeffs,ti)
  
  val = 0;

  for iter = 1 : length(coeffs)
    val = val + coeffs(iter)*(ti^(iter-1));
  end
end
