function [next_state_dt,state_hist] = motionModel(start,goal,dt,p) 
  
current_state = start;
horizon = p.s/goal.vel;
T = 0;
iter = 0;
state_hist = [current_state];

  while (T < horizon) 


    disp(iter);

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

    % get next velocity command 
    % const vel model 
    vel = vel; 

    next_state_dt.sx = sx;
    next_state_dt.sy = sy;
    next_state_dt.theta = theta;
    next_state_dt.kappa = kappa;
    next_state_dt.vel = vel;


    %next_state_dt = responseToControls(current_state,next_state_dt,dt);
    next_state_dt.vel = 0.1;
    current_state = next_state_dt;
    T = T + dt;
    state_hist =[state_hist;current_state];
    disp('kappa');
    disp(next_state_dt.kappa);
    iter=iter+1;
   

  end

end







