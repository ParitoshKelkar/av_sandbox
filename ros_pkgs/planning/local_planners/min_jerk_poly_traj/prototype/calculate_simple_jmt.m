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

 for iter_dt = 0 : dt : t
   frenet_path.s = [frenet_path.s;s_iter];
   frenet_path.d = [frenet_path.d;d_iter];
   frenet_path.jerk_d = frenet_path.jerk_d + jerk_d_iter_dt;
   frenet_path.jerk_s = frenet_path.jerk_s + jerk_s_iter_dt;
 end



end
