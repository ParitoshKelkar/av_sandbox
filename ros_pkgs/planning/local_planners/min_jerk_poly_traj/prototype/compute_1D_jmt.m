function vec = compute_1D_jmt(start_state, goal_state, T)
  
  alpha0 = start_state(1); 
  alpha1 = start_state(2); 
  alpha2 = start_state(3)/2; 

  A = [T^3, T^4, T^5; 
       3*(T^2), 4*(T^3), 5*(T^4); 
       6*T, 12*(T^2), 20*(T^3)];


  B = [goal_state(1); goal_state(2); goal_state(3)];
  C1 = alpha0 + alpha1*T + alpha2*(T^2);
  C2 = alpha1 + 2*alpha2*T;
  C3 = 2*alpha2;

  B_new = B-[C1;C2;C3];

  %temp_vec = inv(A)*B_new;
  temp_vec = A \ B_new;


  vec = [alpha0; alpha1; alpha2; temp_vec];
  

end
