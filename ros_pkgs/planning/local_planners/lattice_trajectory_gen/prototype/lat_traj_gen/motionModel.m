function [next_state_dt,state_hist] = motionModel(start,goal,dt,p) 
  
current_state = start;
horizon = p.s/goal.vel;
T = 0;
iter = 0;
dist_covered = 0;
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
    dist_covered = dist_covered + pdist([sx sy; current_state.sx current_state.sy],'euclidean');
    vel = getNextVelocity(current_state, goal, dt,T,p,dist_covered); 

    if (vel > 0.12)
      disp(vel)
    end

    next_state_dt.sx = sx;
    next_state_dt.sy = sy;
    next_state_dt.theta = theta;
    next_state_dt.kappa = kappa;
    next_state_dt.vel = vel;


    if (iter > 717 )
      disp(iter);
    end
    next_state_dt = responseToControls(current_state,next_state_dt,dt);

    if (next_state_dt.vel > 0.12)
      disp(vel)
    end

    next_state_dt.vel = 0.1;
    current_state = next_state_dt;
    T = T + dt;
    state_hist =[state_hist;current_state];
    iter=iter+1;
   

  end
  disp("just finished motion model");

end







