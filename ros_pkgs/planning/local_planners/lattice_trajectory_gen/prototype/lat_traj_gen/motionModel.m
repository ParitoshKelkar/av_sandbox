function [next_state_dt,state_hist] = motionModel(start,goal,dt,p) 
  
vel = goal.vel; % const vel model
current_state = start;
horizon = p.s/goal.vel;
ds = 0.1;
num_steps = p.s/ds;
dt = horizon/(num_steps); 
T = 0; iter = 0;
dist_covered = 0;
state_hist = [current_state];
T = dt;

  while (T <horizon) 


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
    theta = normalizeAngle(theta,0);
    
    % get next curvature 
    %kappa = getNextCurvatureFromCsv(iter+1);
    kappa = getNextCurvature(current_state,dt,T,p);
    

    % get next velocity command 
    % const vel model 

    next_state_dt.sx = sx;
    next_state_dt.sy = sy;
    next_state_dt.theta = theta;
    next_state_dt.kappa = kappa;
    next_state_dt.vel = vel;


    % TODO - uncomment later 
    %next_state_dt = responseToControls(current_state,next_state_dt,dt);



    current_state = next_state_dt;
    state_hist =[state_hist;current_state];
    iter=iter+1;
    T = T + dt;
   

  end
  
  disp("just finished motion model");

end







