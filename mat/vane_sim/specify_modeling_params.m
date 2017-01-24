% surface triangulation params
ptsFitParams.padding = 5;
ptsFitParams.nodeResn = 1;
ptsFitParams.smoothness = 5e-4;
ptsFitParams.maxDistToProjn = 1.5;
triParams.ptsFitParams = ptsFitParams;

triParams.maxResidualForHit = 1;

modelingParams.triParams = triParams;

%% volume ellipsoid params
ellipsoidParams.maxDistForHit = 3.5;

modelingParams.ellipsoidParams = ellipsoidParams;

%% write
relPathModelingParams = '../data/modeling_params';
save(relPathModelingParams,'modelingParams');