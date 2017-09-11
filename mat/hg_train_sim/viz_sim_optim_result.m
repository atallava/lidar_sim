genRelPathOptimProgress = @(instanceIdx) ...
    sprintf('~/lidar_sim/cpp/data/sim_optim/instance_%s/optim_progress.txt',instanceIdx);

%%
instanceIdx = '13070917';
relPathOptimProgress = genRelPathOptimProgress(instanceIdx);
optimProgress = loadSimOptimProgress(relPathOptimProgress);

%%
% figure;
% plot(optimProgress.objArray);
% xlabel('fn eval count');
% ylabel('obj');

figure;
tInMin = optimProgress.tArray/60;
plot(tInMin,optimProgress.objArray,'.-');
xlabel('t (min)');
ylabel('obj');