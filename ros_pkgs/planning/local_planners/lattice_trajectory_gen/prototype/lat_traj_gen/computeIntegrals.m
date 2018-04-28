function [state] = computeIntegrals(start, init_curvature, goal) 

 a = start.kappa;
 b = init_curvature.kappa_1;
 c = init_curvature.kappa_2;
 d = init_curvature.kappa_3;
 s = init_curvature.s;

 % perform closed form evaluations for theta and kappa 
 [state.theta,state.kappa] = integrateThetaKappa(a,b,c,d,s);

 % approximate fresnel 
 [state.x,state.y]  = approximateFresnel(start,init_curvature,goal);



end
