function score = calcTrajScoreLUT(state,traj_final_state)

  theta_th = pi/4; % radians 
  x_th = y_th = 5; % mtrs 

  score =  sqrt( ((state(1)-traj_final_state.sx)/x_th)^2 +((state(2)-traj_final_state.sy)/y_th)^2 +  ((state(3)-traj_final_state.theta)/theta_th)^2);

end
