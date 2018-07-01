function val = evaluate_derv_cubic(coeffs,s)
%% used only for the x(s) and y(s) equations 

 val = coeffs(2) + 2*coeffs(3)*s + 3*coeffs(4)*(s^2);

end
