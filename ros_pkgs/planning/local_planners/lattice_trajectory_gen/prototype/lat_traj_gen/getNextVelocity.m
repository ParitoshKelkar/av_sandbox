function v_new = getNextVelocity(current_state,goal,dt,T,p,dist_covered)

  tf = p.s/goal.vel;
  elapsed_time = T;

  A = [ 1 0 0 0;  0 1 0  0 ; 1 tf tf*tf tf*tf*tf; 0 1 2*tf 3*tf*tf];



  % invert matrix to get coefficients 
  coeff = inv(A)*[0 current_state.vel p.s - dist_covered goal.vel]';

  


  vel_hist = [];

  v_new = coeff(2) + 2*coeff(3)*elapsed_time + 3*coeff(4)*(elapsed_time*elapsed_time);


  if (v_new > 0.15)
    disp(v_new)
  end

  
   
  %while (elapsed_time < tf) 
    %v_new = coeff(2) + 2*coeff(3)*elapsed_time + 3*coeff(4)*(elapsed_time*elapsed_time);
    %vel_hist = [vel_hist,v_new];
    %elapsed_time = elapsed_time + dt;
  %end


  %plot(vel_hist,'r','LineWidth',3);
end
