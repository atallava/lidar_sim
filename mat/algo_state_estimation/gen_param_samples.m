% max_iters, max_correspond_dist, ransac_iters, ransac_threshold,
% log_min_transform_eps, log_min_objective_eps, log_voxel_size

nParams = 7;
paramLims = [10 50; 3 15; 1 20; 0.5 10; -4 -1; -4 -1; -3 0];
paramLims(:,1) = paramLims(:,1);
paramLims(:,2) = paramLims(:,2);
paramRanges = paramLims(:,2)-paramLims(:,1);
paramIsInteger = [1 0 1 0 0 0 0];

%% sample
nSamples = 100;
paramSamples = zeros(nSamples,nParams);

for id = find(paramIsInteger)
    paramSamples(:,id) = randsample(paramLims(id,1):paramLims(id,2),nSamples,'true');
    paramSamples(:,id) = floor(paramSamples(:,id));
end

for id = find(~paramIsInteger)
    paramSamples(:,id) = rand(nSamples,1)*(paramLims(id,2)-paramLims(id,1)) + paramLims(id,1);
end

%% write 
relPathSamples = genRelPathParamSamples();
saveParamSamples(relPathSamples, paramSamples);
fprintf('Written param samples to %s\n', relPathSamples);