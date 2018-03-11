function [next_state_dt] = speedControlLogic(next_state_dt)

  global a_scl;
  global b_scl;
  global maxKappa;
  global v_safety;
  global v_scl;

  vel_cmd = abs(next_state_dt.vel);
  vel_cmd_max = max(v_scl,(next_state_dt.kappa - a_scl)/b_scl);
  kappa_scl = min(maxKappa, a_scl+b_scl*vel_cmd);
  if (abs(next_state_dt.kappa) > kappa_scl || abs(next_state_dt.kappa) == kappa_scl)
          vel_cmd = abs(v_safety*vel_cmd_max);
  end
  
  next_state_dt.vel = vel_cmd;
    
  %disp(vel_cmd);
end   
  

