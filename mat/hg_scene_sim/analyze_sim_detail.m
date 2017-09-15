% given sim detail, viz some packets and rays

%% load helpers
genRelPathEllipsoidModels = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_ellipsoid_models', ...
    sectionId,simVersion);

genRelPathTriModels = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles', ...
    sectionId,simVersion,sectionId,blockId);

%% load
sectionId = 4;
simType = 'hg';
simVersion = '080917';
queryType = 'slice';

% ellipsoid models
% exploiting that at time of creation via mat, all ellipsoids were saved
relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId,simVersion);
load(relPathEllipsoidModels,'ellipsoidModels');

% triangles
relPathModelsDir = genRelPathHgModelsDirMat(sectionId,simVersion);
triBlockIds = getTriModelBlockIds(relPathModelsDir,sectionId);
nTriBlocks = length(triBlockIds);
triModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    
    relPathTriModels = genRelPathTriModels(sectionId,simVersion,blockId);
    container = load(relPathTriModels,'triModels');
    triModelCell{i} = container.triModels;
end
triModels = stitchTriModels(triModelCell);

% sim detail
relPathSimDetail = genRelPathSimDetailMat(sectionId,simType,simVersion,queryType);
load(relPathSimDetail,'simDetail');

%% pick packet
nPackets = length(simDetail.rayPitchesCell);
packetIdx = 195;
% packetIdx = randsample(nPackets,1);

% extract packet info
rayOrigin = simDetail.rayOrigins(packetIdx,:);
rayPitches = simDetail.rayPitchesCell{packetIdx};
rayYaws = simDetail.rayYawsCell{packetIdx};
realPtsAll = simDetail.realPtsAllCell{packetIdx};
realHitFlag = simDetail.realHitFlagCell{packetIdx};
realHitFlag = logical(realHitFlag);
simPtsAll = simDetail.simPtsAllCell{packetIdx};
simHitFlag = simDetail.simHitFlagCell{packetIdx};
simHitFlag = logical(simHitFlag);

rayDirns = calcRayDirnsFromSph(rayPitches,rayYaws);

trueHitRayIds = find(realPtsAll & simPtsAll);
falseHitRayIds = find(~realPtsAll & simPtsAll);
falseMissRayIds = find(realPtsAll & ~simPtsAll);
trueMissRayIds = find(~realPtsAll & ~simPtsAll);

%% viz packet
hfig = figure(); axis equal;
grid on; box on;

% drawPacket(hfig,rayOrigin,rayDirns,realPtsAll,realHitFlag,'b');
% drawPacket(hfig,rayOrigin,rayDirns,simPtsAll,simHitFlag,'r');
drawPts(hfig,realPtsAll(realHitFlag,:),'b');
drawPts(hfig,simPtsAll(simHitFlag,:),'r');

% draw models
modelNbrRadius = 1;
simPtsHit = simPtsAll(simHitFlag,:);
realPtsHit = realPtsAll(realHitFlag,:);
triModelsNbr = createTriModelsNbr(triModels,[simPtsHit; realPtsHit],modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,[simPtsHit; realPtsHit],modelNbrRadius);

drawEllipsoids(hfig,ellipsoidModelsNbr);
drawTriModels(hfig,triModelsNbr,'ground');

title(sprintf('packet idx: %d',packetIdx));

%% pick ray
nRays = size(rayDirns,1);
% rayIdx = randsample(nRays,1);
rayIdx = 49;
% rayIdx = randsample(trueHitRayIds,1);
% rayIdx = randsample(falseHitRayIds,1);
% rayIdx = randsample(falseMissRayIds,1);

thisRayDirn = rayDirns(rayIdx,:);
thisRealPt = realPtsAll(rayIdx,:);
thisRealHitFlag = realHitFlag(rayIdx);
thisSimPt = simPtsAll(rayIdx,:);
% thisSimPt = [-405.667 344.584 2.00723]; % when want to viz specific point
thisSimHitFlag = simHitFlag(rayIdx);

% draw models
if ~thisSimHitFlag && ~thisRealHitFlag
    thisRayLength = 1;
else
    thisSimLength = thisSimHitFlag*norm(rayOrigin-thisSimPt);
    thisRealLength = thisRealHitFlag*norm(rayOrigin-thisRealPt);
    thisRayLength = max(thisSimLength,thisRealLength);
end
thisRayPts = genPtsRay(rayOrigin,thisRayDirn,thisRayLength);

modelNbrRadius = 2.0;
triModelsNbr = createTriModelsNbr(triModels,thisRayPts,modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,thisRayPts,modelNbrRadius);

%% viz ray
hfig = figure(); axis equal;
grid on; box on;

drawPacket(hfig,rayOrigin,thisRayDirn,thisRealPt,thisRealHitFlag,'b');
drawPacket(hfig,rayOrigin,thisRayDirn,thisSimPt,thisSimHitFlag,'r');
 
drawEllipsoids(hfig,ellipsoidModelsNbr);
drawTriModels(hfig,triModelsNbr,'ground');

title(sprintf('packet idx: %d, ray idx: %d',packetIdx,rayIdx));

%% adjust for paper fig
title('');
addAxisCartesianLabels(hfig,3,'m');
fontSize = 15;
set(gca,'FontSize',fontSize);

% p219r313
% grid off;
% az = -44; el = 16; 
% view(az,el);
% % use these to generate axes
% xlim([-132.3170 -114.5878]);
% ylim([361.1068  375.0900]);
% zlim([-10.0041    3.9791]);

% p9r155
% az = 175; el = 3; 
% view(az,el);
% xlim([-376.9543 -305.0173])
% ylim([313.1444  369.8818]);
% zlim([-27.7384   28.9990]);

% p9r212
% xlim([-306.4402 -289.5041]);
% ylim([353.2215  366.5791]);

% p195r49
grid off;
az = -1.6; el = 9.2;
view(az,el);
% use these to generate fig axes
xlim([-154.7055 -141.5450]);
ylim([379.2514  389.6312]);
zlim([-8.9204    1.4594]);





