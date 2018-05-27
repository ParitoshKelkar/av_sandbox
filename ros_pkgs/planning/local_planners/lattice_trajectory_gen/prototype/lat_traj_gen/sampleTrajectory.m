function [final_state, state_hist] = sampleTrajectory(start,dt,p)
 
vel = start.vel;
current_state = start;
horizon = p.s/vel;
T = 0; iter = 0;
dist_covered = 0;
state_hist = [current_state];

  while (T < horizon) 

    sx = current_state.sx;
    sy = current_state.sy;
    theta = current_state.theta;
    kappa  = current_state.kappa;
    vel = current_state.vel;
    
    % get next positions 
    sx = sx + vel*cos(theta)*dt;
    sy = sy + vel*sin(theta)*dt;
    theta  = theta + kappa*dt*vel;
    
    % get next curvature 
    kappa = getNextCurvature(current_state,dt,T,p);

    next_state_dt.sx = sx;
    next_state_dt.sy = sy;
    next_state_dt.theta = theta;
    next_state_dt.kappa = kappa;
    next_state_dt.vel = vel;


    % TODO - uncomment later 
    %next_state_dt = responseToControls(current_state,next_state_dt,dt);


    current_state = next_state_dt;
    T = T + dt;
    state_hist =[state_hist;current_state];
    iter=iter+1;
   

  end
  final_state = current_state;

end
