% load section data
relPathSection = '../../data/taylorJune2014/sections/laser_frame/section_03.xyz';
nPacketsToRead = 10;
section = loadSection(relPathSection,nPacketsToRead);

relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog');
poseTLog = tLog;

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

%% distinguishable colors
someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);
rayColors = distinguishable_colors(length(section.packetIds));

%%
% look at ray alphas and thetas for a single packet
% 12 thetas at resn 3e-3 rad
% 30 alphas, some error with spec-sheet choice
rayLengthToPlot = 10;
for i = 5%length(section.packetIds)
    t = section.packetTimestamps(i);
    pts = getSectionPtsAtTime(section,t);
    rayOrigin = [0 0 0]; % since points are in laser frame

    [rayTheta,rayAlpha] = deal(zeros(1,size(pts,1)));
    for j = 1:size(pts,1)
        rayDirn = calcRayDirn(rayOrigin,pts(j,:));
        rayPts = genPtsRay(rayOrigin,rayDirn,rayLengthToPlot);
        [rayTheta(j),rayAlpha(j),~] = cart2sph(rayDirn(1),rayDirn(2),rayDirn(3));
%         plot3(rayPts(:,1),rayPts(:,2),rayPts(:,3),'linewidth',2,'color',rayColors(i,:));
%         hold on;
    end    
end
% axis equal;
% grid on;
% xlabel('x'); ylabel('y'); zlabel('z');
