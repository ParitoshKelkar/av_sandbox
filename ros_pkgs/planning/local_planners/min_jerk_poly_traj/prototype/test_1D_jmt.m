% just to test 1D jmt equations 

start_state = [0; 10; 2];
final_state = [20; 20; 0];
T = 15;

coeffs = compute_1D_jmt(start_state, final_state,T);

dt = 0.1;
xs = [];

%coeffs = [10;10;0;-0.04;0.02;-0.002];

for iter_dt = 0 : dt : T 
  if iter_dt > 14.9
    disp('here')    
  end
  xs = [xs;evaluate_poly(start_state, final_state,coeffs,iter_dt,T)];
end
