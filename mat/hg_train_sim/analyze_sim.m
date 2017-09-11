% todo: cleanup
% view 

%% load helpers
genRelPathTriangleModels = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s,section_%02d_block_%02d_ground_triangles', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathEllipsoidModels = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathTriModels = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathHgModelsDir = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s',sectionId,simVersion);

genRelPathSliceRealPts = @(sectionId,simVersion,queryType) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/version_%s/%s_real_pts', ...
    sectionId,simVersion,queryType);

genRelPathSliceSimPts = @(sectionId,simVersion,queryType) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/version_%s/%s_sim_pts', ...
    sectionId,simVersion,queryType);

genRelPathSliceSimDetail = @(sectionId,simVersion,queryType) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/%s_sim_detail', ...
    sectionId,simVersion,queryType);

%% load
% ellipsoid models
sectionIdOfModels = 3;
simVersion = '250417';
relPathModelsDir = genRelPathHgModelsDir(sectionIdOfModels,simVersion);
ellipsoidBlockIds = getEllipsoidModelBlockIds(relPathModelsDir,sectionIdOfModels);
nTriBlocks = length(ellipsoidBlockIds);
ellipsoidModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = ellipsoidBlockIds(i);
    
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionIdOfModels,simVersion,blockId);
    container = load(relPathEllipsoidModels,'ellipsoidModels');
    ellipsoidModelCell{i} = container.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelCell);

triBlockIds = getTriModelBlockIds(relPathModelsDir,sectionIdOfModels);
nTriBlocks = length(triBlockIds);
triModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    
    relPathTriModels = genRelPathTriModels(sectionIdOfModels,simVersion,blockId);
    container = load(relPathTriModels,'triModels');
    triModelCell{i} = container.triModels;
end
triModels = stitchTriModels(triModelCell);

%% pts
% real pts
sectionIdForSim = 8;
queryType = 'section';
relPathRealPts = genRelPathSliceRealPts(sectionIdForSim,queryType);
realPts = loadPts(relPathRealPts);

% sim pts
relPathSimPts = genRelPathSliceSimPts(sectionIdForSim,queryType);
simPts = loadPts(relPathSimPts);

%% sim detail
relPathSimDetail = genRelPathSliceSimDetail(sectionIdForSim,simVersion,queryType);
load(relPathSimDetail,'simDetail');

%% viz pts marginalized
hfig = figure(); axis equal;
box on; grid on;
drawPts(hfig,realPts,'b');
drawPts(hfig,simPts,'r');

% draw models
modelNbrRadius = 0.5;
triModelsNbr = createTriModelsNbr(triModels,simPts,modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,simPts,modelNbrRadius);

drawEllipsoids(hfig,ellipsoidModelsNbr);
drawTriModels(hfig,triModelsNbr,'ground');

%% pick packet
nPackets = length(simDetail.rayPitchesCell);
packetIdx = 2699;
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

title(sprintf('packet id: %d',packetIdx));

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

title(sprintf('packet id: %d',packetIdx));

%% pick ray
% rayIdx = randsample(size(rayDirns,1),1);
rayIdx = 169;
thisRayDirn = rayDirns(rayIdx,:);
thisRealPt = realPtsAll(rayIdx,:);
thisRealHitFlag = realHitFlag(rayIdx);
thisSimPt = simPtsAll(rayIdx,:);
% thisSimPt = [-405.667 344.584 2.00723];
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

title(sprintf('packet id: %d, ray id: %d',packetIdx,rayIdx));







