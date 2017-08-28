relPathSimOptimProgress = "~/lidar_sim/cpp/data/sim_optim/optim_progress.txt";
optimProgress = loadSimOptimProgress(relPathSimOptimProgress);

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