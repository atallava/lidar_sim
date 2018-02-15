% check how vehicle axes are aligned relative to path

%% load
relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'tLog','poseLog');

%% plot xy
idsStart = 10000;
idsEnd = 50000;
idsSkipForTf = 1000;
count = 1;
posnsToPlot = [];
TfsToPlot = {};
for i = idsStart:idsEnd
    pose = poseLog(i,:);
    T = getImuTransfFromImuPose(pose);
    posnsToPlot(count,:) = T(1:3,4);
    if mod(count-1,idsSkipForTf) == 0
        TfsToPlot{end+1} = T;
    end
    count = count+1;
end

%% viz
hfig = figure(); hold on;
axes3Length = 10;
for i = 1:length(TfsToPlot)
    T = TfsToPlot{i};
    drawAxes3(hfig, T, axes3Length);
end

plot3(posnsToPlot(:,1), posnsToPlot(:,2), posnsToPlot(:,3), '-');

text(posnsToPlot(1,1), posnsToPlot(1,2), posnsToPlot(1,3), ...
    sprintf('id start: %d', idsStart));
text(posnsToPlot(end,1), posnsToPlot(end,2), posnsToPlot(end,3), ...
    sprintf('id end: %d', idsEnd));

axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');


