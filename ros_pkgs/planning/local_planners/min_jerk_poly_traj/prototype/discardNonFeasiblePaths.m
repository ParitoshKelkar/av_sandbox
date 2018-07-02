function fp = discardNonFeasiblePaths(fp)

  for iter = 1 : size(fp,1)
    [kappa,acc,vel] = getPathMetric(fp(iter));
  end

end
