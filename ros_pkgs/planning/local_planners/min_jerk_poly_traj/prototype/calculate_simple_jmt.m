function [frenet_path] = calculate_simple_jmt(start_frenet_state, goal_frenet_state,t)

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

 coeffs_s = compute_1D_jmt([s0,s0_dot, s0_ddot],[sf, sf_dot, sf_ddot], t);
 coeffs_d = compute_1D_jmt([d0,d0_dot, d0_ddot],[df, df_dot, df_ddot], t);

 frenet_path.s = [];
 frenet_path.d = [];

 frenet_path.s_dot = [];
 frenet_path.d_dot = [];

 frenet_path.s_ddot = [];
 frenet_path.d_ddot = [];

 frenet_path.jerk_s = [];
 frenet_path.jerk_d = [];


 T = t;
 for iter_dt = 0 : dt : t
   s_iter = evaluate_poly(coeffs_s,iter_dt);
   d_iter = evaluate_poly(coeffs_d,iter_dt);

   sdot_iter = evaluate_first_derv(coeffs_s, iter_dt);
   ddot_iter = evaluate_first_derv(coeffs_d, iter_dt);

   s_ddot_iter = evaluate_second_derv(coeffs_s, iter_dt); 
   d_ddot_iter = evaluate_second_derv(coeffs_d, iter_dt); 

   frenet_path.s = [frenet_path.s;s_iter];
   frenet_path.d = [frenet_path.d;d_iter];

   frenet_path.s_dot = [frenet_path.s_dot;sdot_iter];
   frenet_path.d_dot = [frenet_path.d_dot;ddot_iter];

   frenet_path.s_ddot = [frenet_path.s_ddot;s_ddot_iter];
   frenet_path.d_ddot = [frenet_path.d_ddot;d_ddot_iter];

   frenet_path.jerk_s = [frenet_path.jerk_s ; evaluate_third_derv(coeffs_s,iter_dt)];
   frenet_path.jerk_d = [frenet_path.jerk_d ; evaluate_third_derv(coeffs_d,iter_dt)];


 end


end


