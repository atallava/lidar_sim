%% rvctoolbox
someUsefulPaths;
relPathRvcStartup = [pathToM '/rvctools/startup_rvc.m'];
run(relPathRvcStartup);

%%
v = 10;
duration = 230/v;

tStart = 0;
tEnd = duration;

tResn = 0.1; 
tKeypoints = tStart:tResn:tEnd;
nKeypoints = length(tKeypoints);

%% calculate camera params
cameraPositions = zeros(nKeypoints,3);
cameraTargets = zeros(nKeypoints,3);

for i = 1:length(tKeypoints)
t = tKeypoints(i);

x = 0; y = v*t; z = 1;
cameraPosition = [x y z];
cameraTarget = [x y+0.1 z];

cameraPositions(i,:) = cameraPosition;
cameraTargets(i,:) = cameraTarget;
end

viewDirns = calcViewDirns(cameraPositions,cameraTargets);

%% viz
% scatter3(pts(:,1),pts(:,2),pts(:,3),'b.');
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); 
hold on;

quiver3(cameraPositions(:,1),cameraPositions(:,2),cameraPositions(:,3), ...
    viewDirns(:,1),viewDirns(:,2),viewDirns(:,3),'r');

%% write out
relPathOutCsv = '../data/rviz/section_42_camera_poses.csv';
writeCameraTransforms(relPathOutCsv,tKeypoints-tStart,cameraPositions,viewDirns);

 