% given sim detail, viz some packets and rays

%% load helpers
genRelPathSceneTriModels = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models_reduced', ...
    sectionId,simVersion);

genRelPathGroundTriModelsDir = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/', ...
    sectionId,simVersion);

genRelPathGroundTriModels = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/mm_sim/version_%s/section_%02d_block_%02d_ground_triangles', ...
    sectionId,simVersion,sectionId,blockId);

%% load
sectionId = 4;
simType = 'mm';
simVersion = '280817';
queryType = 'slice';

% object triangles
% todo: change names. sceneObjectTriModelsCell. sceneTriModels
% todo: ideally should have used triModel for triangles. then triModels has
% the better meaning slot
relPathSceneTriModels = genRelPathSceneTriModels(sectionId,simVersion);
load(relPathSceneTriModels,'sceneTriModels');
sceneTriModels = stitchTriModels(sceneTriModels);

% ground triangles
relPathModelsDir = genRelPathGroundTriModelsDir(sectionId,simVersion);
groundTriBlockIds = getTriModelBlockIds(relPathModelsDir,sectionId);
nGroundTriBlocks = length(groundTriBlockIds);
groundTriModelCell = cell(1,nGroundTriBlocks);
for i = 1:nGroundTriBlocks
    blockId = groundTriBlockIds(i);
    
    relPathTriModels = genRelPathGroundTriModels(sectionId,simVersion,blockId);
    container = load(relPathTriModels,'triModels');
    groundTriModelCell{i} = container.triModels;
end
groundTriModels = stitchTriModels(groundTriModelCell);

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
groundTriModelsNbr = createTriModelsNbr(groundTriModels,[simPtsHit; realPtsHit],modelNbrRadius);
sceneTriModelsNbr = createTriModelsNbr(sceneTriModels,[simPtsHit; realPtsHit],modelNbrRadius);

drawTriModels(hfig,sceneTriModelsNbr,'veg');
drawTriModels(hfig,groundTriModelsNbr,'ground');

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
groundTriModelsNbr = createTriModelsNbr(groundTriModels,thisRayPts,modelNbrRadius);
sceneTriModelsNbr = createTriModelsNbr(sceneTriModels,thisRayPts,modelNbrRadius);

%% viz ray
hfig = figure(); axis equal;
grid on; box on;

drawPacket(hfig,rayOrigin,thisRayDirn,thisRealPt,thisRealHitFlag,'b');
drawPacket(hfig,rayOrigin,thisRayDirn,thisSimPt,thisSimHitFlag,'r');

drawTriModels(hfig,sceneTriModelsNbr,'veg');
drawTriModels(hfig,groundTriModelsNbr,'ground');

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
% xlim([-155.4852  -81.4725]);
% ylim([319.2443  377.6188]);
% zlim([-28.3097   30.0649]);

% p9r155
% az = 175; el = 3; 
% view(az,el);
% xlim([-376.9543 -305.0173])
% ylim([313.1444  369.8818]);
% zlim([-27.7384   28.9990]);

% p9r212
% grid off;
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







