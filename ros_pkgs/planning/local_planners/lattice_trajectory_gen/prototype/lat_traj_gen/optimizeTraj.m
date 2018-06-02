function [curvature,res] = optimizeTraj(start,goal,curvature)
% to be called from generateLUT given start, goal, init_param
% curvature is the spline already constructed with the guess params

  curv_original = curvature;

  dt = 0.1;

  iter = 0;
  res = 0;
  while (iter < 5)

    [integrated_state,state_hist] = motionModel(start,goal,dt,curvature);

    if isempty(integrated_state)
      curvature = curv_original; 
      res = 0;
      break;
    end

    if checkConvergence(integrated_state,goal)
      res = 1;
      break;
    end

    param = generateCorrection(start,goal,integrated_state,dt,curvature);

    p.p0 = start.kappa;
    p.p1 = curvature.p1 + param(2);
    p.p2 = curvature.p2 + param(3);
    p.s =(curvature.s + param(1));
    p.p3 = goal.kappa;

    if p.s < 0
      curvature = curv_original; 
      res = 0;
      break;
    end

    p = makeCubicSpline(p.p0, p.p1, p.p2,p.p3,p.s);

    [new_state,state_hist] = motionModel(start,goal,dt,p);
    curvature = p;
    integrated_state = new_state;

    iter = iter + 1;
  end

end
