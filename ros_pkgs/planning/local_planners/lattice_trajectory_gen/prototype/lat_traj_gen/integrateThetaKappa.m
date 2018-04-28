function [theta,kappa] = integrateThetaKappa(a,b,c,d,s) 

 theta = a*s + 0.5*b*s*s + c*(s^3)/3 + d*(s^4)/4; 

 kappa = a + b*s + c*(s^2) + d*(s^3);   

end
