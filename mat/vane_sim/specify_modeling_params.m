maxDistForHit = 3.5;

%% pack
modelingParams.maxDistForHit = maxDistForHit;

%% write
relPathModelingParams = '../data/modeling_params';
save(relPathModelingParams,'modelingParams');