function [x,y] = approximateFresnel(start, init_curvature, goal) 

 a = start.kappa;
 b = init_curvature.kappa_1;
 c = init_curvature.kappa_2;
 d = init_curvature.kappa_3;
 s = init_curvature.s;

 x_integrand = 0;
 y_integrand = 0;

 integrand_split = 20; 
 deltaLength = s/integrand_split;

 fresnel_iter = 1;

 while (fresnel_iter < integrand_split )

   if (mod(fresnel_iter,2) == 0)
     multiplier = 2;
   else
     multiplier = 4;
   end

   [temp_theta, temp_kappa] = integrateThetaKappa(a,b,c,d,fresnel_iter*deltaLength); 

   x_integrand = x_integrand + multiplier*cos(temp_theta);
   y_integrand = y_integrand + multiplier*sin(temp_theta);

   fresnel_iter = fresnel_iter + 1;

 end

 [final_theta,final_kappa] = integrateThetaKappa(a,b,c,d,s);

  x_integrand = x_integrand + cos(start.theta) + cos(final_theta);
  y_integrand = y_integrand + sin(start.theta) + sin(final_theta);


  x = (deltaLength/3)*x_integrand;
  y = (deltaLength/3)*y_integrand;


end
