%generate velocity given the total path length and the approximate time taken  

% known(s)
tf = 17.2; % secs 
v0 = 0;
vf = 5;
a0 = 0;
af = 0;
sf = 17.2; %mtrs

% matrix to be inverted 
%A = [1 0 0 0; 0 0 0 0];
%A = [1, 0,  0,    0 ;
     %0, 1 , 0 ,   0 ;
     %1,tf ,tf^2, tf^3;
     %0, 1 ,2*tf,3*(tf^2); 
     %tf,0.5(tf^2),(0.333)*(tf^3),(0.25)*(tf^4) ];

A = [ 1 0  0 0;  0 1 0  0 ; 1 tf tf*tf tf*tf*tf; 0 1 2*tf 3*tf*tf];



% invert matrix to get coefficients 
coeff = inv(A)*[0 v0 sf vf]';


dt = 0.1;
elapsed_time = 0.0;
vel_hist = [];
 
while (elapsed_time < tf) 
  v_new = coeff(2) + 2*coeff(3)*elapsed_time + 3*coeff(4)*(elapsed_time*elapsed_time);
  vel_hist = [vel_hist,v_new];
  elapsed_time = elapsed_time + dt;
end


plot(vel_hist,'r','LineWidth',3);


