function val = evaluate_poly(start, goal, coeffs,ti,T)
  
  val = 0;

  %val = coeffs(1) + coeffs(2)*(ti) + coeffs(3)*((ti)^2) + coeffs(4)*(ti)^3 + coeffs(5)*(ti)^4 + coeffs(6)*((ti)^5);

  %val = start(1) + (start(1) - goal(1))*(15*((ti/T)^4) - 6*((ti/T)^5) - 10*(ti/T)^3);


  % generic enough to be used in any poly expanision
  for iter = 1 : length(coeffs)
    val = val + coeffs(iter)*(ti^(iter-1));
  end
end
