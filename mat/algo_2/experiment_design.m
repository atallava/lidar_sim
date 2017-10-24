% units:
% distance: m, time: s, angle: rad

vCruise = 5; % regular vehicle speed

wSens = 62.7; % sensor rotation speed
tPacket = 5.5e-4; % lidar packet period
fPacket = 1/tPacket; % lidar packet frequency
dYawPacket = 0.0347; % packet yaw resolution 
nZPacket = 384; % readings per packet
fracnHit = 0.8; % fraction readings which are hit

%% decision variables
% previously 10
sPerp = 5; % lateral dist to object
sPath = 638; % total length of path
sVox = 0.1; % voxel side length
sQv = 3; % 3 % query vol side length
% previously 0.1
vObjPass = 0.5; % speed when passing object
sMax = 10; % max distance from object to start log
% previously 10
nInstPerObj = 5; 
% previously 10
nObj = 40;

%% 
nVoxPerQv = floor(sQv/sVox)^3;

alphaMin = atan2(sPerp,sMax);
betaMin = calcViewAngle(sMax,sQv,sPerp);
betaMax = calcViewAngle(0,sQv,sPerp);
betaAvg = calcAvgViewAngle(sMax,sQv,sPerp,vObjPass);

tObjPass = 2*sMax/vObjPass; % duration of passing object
nPacketsObjPass = tObjPass*fPacket; % total packets per object pass
nPassPacketsInQv = nPacketsObjPass*(betaAvg/(2*pi)); % restricting to those in viewing angle
nPassHitsInQv = nPassPacketsInQv*nZPacket*fracnHit; % this is for the full pass
nHitsPerInstance = nPassHitsInQv/nInstPerObj; 
nHitsPerVoxel = nHitsPerInstance/nVoxPerQv; 

nInstances = nObj*nInstPerObj;

sObjPasses = 2*sMax*nObj;
sCruise = (sPath-sObjPasses);

tFull = sObjPasses/vObjPass + sCruise/vCruise;

fprintf('n voxels per query volume: %d\n',nVoxPerQv);
fprintf('n instances: %.2f\n',nInstances);
fprintf('n hits per instance: %.2f\n',nHitsPerInstance);
fprintf('n hits per voxel: %.2f\n',nHitsPerVoxel);
fprintf('s obj passes: %.2f, s cruise: %.2f, t full: %.2f min\n', ...
    sObjPasses,sCruise,tFull/60);

% n instances, ~100
% n hits per voxel, ~30
% n hits per instance, 4e6