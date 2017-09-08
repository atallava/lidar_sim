relPathPoses = '../data/pose_log.mat';
load(relPathPoses,'poseLog','tLog');

%%
nPoses = size(poseLog,1);
xy = zeros(nPoses,2);
for i = 1:nPoses
    imuTransf = getImuTransfFromImuPose(poseLog(i,:));
    xy(i,:) = imuTransf(1:2,4);
end

%%
hfig = figure(); axis equal;
lineWidth = 7;
lineColor = [0 0 0];
plot(xy(:,1),xy(:,2),'linewidth',lineWidth,'color',lineColor);

xlabel('x (m)'); ylabel('y (m)');

fontSize = 50;
set(gca,'FontSize',fontSize);