%% load section pts
relPathSectionPts = '../data/section_pts_03_world_frame_subsampled_1e5';
load(relPathSectionPts,'pts');

%% load imu poses
relPathPoseLog = '../data/pose_log';
load(relPathPoseLog,'poseLog','tLog');

%% laser calib
relPathLaserCalibParams = '../data/laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

%% section times
tStart = 1403045836.830185000;
tEnd = 1403045902.775886000;
tResn = 1; 
tKeypoints = tStart:tResn:tEnd;
nKeypoints = length(tKeypoints);

%% viz
scatter3(pts(:,1),pts(:,2),pts(:,3),'.');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); 

%% set cam params to manual
set(gca,'CameraPositionMode','manual');
set(gca,'CameraTargetMode','manual');

cameraPositions = zeros(nKeypoints,3);
cameraTargets = zeros(nKeypoints,3);

% go to those camera poses to see if they make sense
for i = 1:length(tKeypoints)
t = tKeypoints(i);
poseId = indexOfNearestTime(t,tLog);
imuPose = poseLog(poseId,:);
T_imu_world = getImuTransfFromImuPose(imuPose);
T_laser_world = getLaserTransfFromImuPose(imuPose,laserCalibParams.extrinsics.TLaserImu);

[cameraPosition,cameraTarget] = getCameraParamsFromTransf(T_imu_world);

set(gca,'cameraPosition',cameraPosition);
set(gca,'cameraTarget',cameraTarget);
pause(1);
title(sprintf('%.2f',t-tStart));

% log
cameraPositions(i,:) = cameraPosition;
cameraTargets(i,:) = cameraTarget;
end

viewDirns = cameraTargets-cameraPositions;
rowNorms = sum(viewDirns.^2,2);
viewDirns = bsxfun(@rdivide,viewDirns,rowNorms);

%%
% scatter3(pts(:,1),pts(:,2),pts(:,3),'b.');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); 
hold on;
% plot3(cameraPositions(:,1),cameraPositions(:,2),cameraPositions(:,3),'r','linewidth',5);
% plot3(cameraTargets(:,1),cameraTargets(:,2),cameraTargets(:,3),'g','linewidth',5);

quiver3(cameraPositions(:,1),cameraPositions(:,2),cameraPositions(:,3), ...
    viewDirns(:,1),viewDirns(:,2),viewDirns(:,3),'r');

%% write out
relPathOutTxt = '../data/section_03_camera_transfs.txt';
writeCameraTransforms(relPathOutTxt,tKeypoints-tStart,cameraPositions,viewDirns);

%%
relPathOutMat = '../data/section_03_camera_transfs';
save(relPathOutMat,'tStart','tKeypoints','cameraPositions','viewDirns');
