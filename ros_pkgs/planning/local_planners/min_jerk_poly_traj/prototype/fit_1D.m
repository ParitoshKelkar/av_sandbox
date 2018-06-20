function [xs, coeffs] = fit_1D(x,s)
 % this function fits polyfit to 
 % the 's' Frenet dist 
 coeffs = polyfit(s,x,3);
 coeffs = fliplr(coeffs); % rev order 

 ds = 0.5;
 xs = [];
 for iter = 0 : ds :s(end)
   temp_xs = coeffs(1) + coeffs(2)*iter + coeffs(3)*(iter^2) + coeffs(4)*(iter^3);
   xs = [xs;temp_xs];
 end

end
