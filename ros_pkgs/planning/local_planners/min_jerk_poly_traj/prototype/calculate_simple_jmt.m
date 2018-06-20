function [frenet_path] = calculate_simple_jmt(start_frenet_state, goal_frenet_state)

 t = 8;  % random guess but some heuristic could be placed here 
 dt = 0.1; 

 s0 = start_frenet_state.s; 
 s0_dot = start_frenet_state.s_dot; 
 s0_ddot = start_frenet_state.s_ddot; 

 sf = goal_frenet_state.s; 
 sf_dot = goal_frenet_state.s_dot; 
 sf_ddot = goal_frenet_state.s_ddot; 


 d0 = start_frenet_state.d; 
 d0_dot = start_frenet_state.d_dot; 
 d0_ddot = start_frenet_state.d_ddot; 

 df = goal_frenet_state.d; 
 df_dot = goal_frenet_state.d_dot; 
 df_ddot = goal_frenet_state.d_ddot; 

 coeffs_s = compute_1D_jmt([s0,s_dot, s_ddot],[sf, sf_dot, sf_ddot], t);
 coeffs_d = compute_1D_jmt([d0,d_dot, d_ddot],[df, df_dot, df_ddot], t);

 frenet_path.s = [];
 frenet_path.d = [];
 frenet_path.jerk_d = 0;
 frenet_path.jerk_s = 0;

 for iter_dt = 0 : dt : t
   s_iter = evaluate_poly(coeffs_s,iter_dt);
   d_iter = evaluate_poly(coeffs_d,iter_dt);

   frenet_path.s = [frenet_path.s;s_iter];
   frenet_path.d = [frenet_path.d;d_iter];
   frenet_path.jerk_d = frenet_path.jerk_d + jerk_d_iter_dt;
   frenet_path.jerk_s = frenet_path.jerk_s + jerk_s_iter_dt;

 end


end

function vec = compute_1D_jmt(start_state, goal_state, T)
  
  alpha0 = start(1); 
  alpha1 = start(2); 
  alpha2 = start(3)/2; 

  A = [T^3, T^4, T^5; 
       3*(T^2), 4*(T^4), 5*(T^5); 
       6*T, 12*(T^2), 20*(T^3)];

  vec = [alpha3; alpha4; alpha5];

  B = [goal_state(1); goal_state(2); goal_state(3)];
  C1 = alpha0 + alpha1*T + 0.5*alpha*(T^2);
  C2 = alpha1 + alpha2*T;
  C3 = alpha2;

  B_new = B-[C1;C2;C3];

  vec = inv(A)*B_new;

end

function val = evaluate(coeffs,ti)
  val = coeffs(1) + coeffs(2)*ti + coeffs(3)*(ti^2) + coeffs(4)*(ti^3)+coeffs(5)*(ti^4) + coeffs(6)*(ti^6);
end