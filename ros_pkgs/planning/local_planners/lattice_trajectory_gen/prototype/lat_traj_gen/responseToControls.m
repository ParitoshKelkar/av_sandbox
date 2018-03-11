function [next_state_dt] = responseToControls(current_state,next_state_dt,dt) 
  
  global kappaDotMax;
  global kappaDotMin;

  global minKappa;
  global maxKappa;

  kappaDot = (-current_state.kappa + next_state_dt.kappa)/dt;
  kappaDot = min(kappaDot,kappaDotMax);
  kappaDot = max(kappaDot,kappaDotMin);

  next_state_dt = speedControlLogic(next_state_dt);

  kappa = current_state.kappa + kappaDot*dt;
  kappa = min(kappa,maxKappa);
  kappa = max(kappa,minKappa);

  accCmd = (next_state_dt.vel - current_state.vel)/dt;

  global maxAcc;
  global maxDecc;


  accCmd = min(accCmd,maxAcc);
  accCmd = max(accCmd,maxDecc);

  vel = current_state.vel + accCmd*dt;

  next_state_dt.vel = vel;
  next_state_dt.kappa = kappa;

end
