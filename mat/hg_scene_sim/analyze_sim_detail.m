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
packetIdx = 695;
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

%% viz packet pts only
hfig = figure(); axis equal;
grid on; box on;

drawPts(hfig,realPtsAll(realHitFlag,:),'b');
drawPts(hfig,simPtsAll(simHitFlag,:),'r');

% draw models
modelNbrRadius = 0.7;
simPtsHit = simPtsAll(simHitFlag,:);
realPtsHit = realPtsAll(realHitFlag,:);
triModelsNbr = createTriModelsNbr(triModels,[simPtsHit; realPtsHit],modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,[simPtsHit; realPtsHit],modelNbrRadius);

drawEllipsoids(hfig,ellipsoidModelsNbr);
drawTriModels(hfig,triModelsNbr,'ground');

title(sprintf('packet idx: %d',packetIdx));

%% viz packet
hfig = figure(); axis equal;
grid on; box on;

drawPacket(hfig,rayOrigin,rayDirns,realPtsAll,realHitFlag,'b');
drawPacket(hfig,rayOrigin,rayDirns,simPtsAll,simHitFlag,'r');

% draw models
modelNbrRadius = 0.5;
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
rayIdx = 207;
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

modelNbrRadius = 3.5;
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







