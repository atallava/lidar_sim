%% load helpers
genRelPathTriangleModels = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sectionId,blockId);

genRelPathEllipsoidModels = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/section_%02d_block_%02d_non_ground_ellipsoids', ...
    sectionId,sectionId,blockId);

genRelPathTriModels = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sectionId,blockId);

genRelPathModelsDir = @(sectionId) ...
    sprintf('../data/sections/section_%02d/hg_sim',sectionId);

genRelPathSliceRealPts = @(sectionId,queryType) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/%s_real_pts',sectionId,queryType);

genRelPathSliceSimPts = @(sectionId,queryType) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/%s_sim_pts',sectionId,queryType);

genRelPathSliceSimDetail = @(sectionId,queryType) ...
    sprintf('../data/sections/section_%02d/hg_sim/%s_sim_detail',sectionId,queryType);

%% load
% ellipsoid models
sectionIdOfModels = 3;
relPathModelsDir = genRelPathModelsDir(sectionIdOfModels);
ellipsoidBlockIds = getEllipsoidModelBlockIds(relPathModelsDir,sectionIdOfModels);
nTriBlocks = length(ellipsoidBlockIds);
ellipsoidModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = ellipsoidBlockIds(i);
    
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionIdOfModels,blockId);
    container = load(relPathEllipsoidModels,'ellipsoidModels');
    ellipsoidModelCell{i} = container.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelCell);

triBlockIds = getTriModelBlockIds(relPathModelsDir,sectionIdOfModels);
nTriBlocks = length(triBlockIds);
triModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    
    relPathTriModels = genRelPathTriModels(sectionIdOfModels,blockId);
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
relPathSimDetail = genRelPathSliceSimDetail(sectionIdForSim,queryType);
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
packetIdx = floor(3374/5*2);
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
% rayId = randsample(size(rayDirns,1),1);
rayId = floor(384/3*3);
thisRayDirn = rayDirns(rayId,:);
thisRealPt = realPtsAll(rayId,:);
thisRealHitFlag = realHitFlag(rayId);
thisSimPt = simPtsAll(rayId,:);
thisSimHitFlag = simHitFlag(rayId);

% draw models
if ~thisSimHitFlag && ~thisRealHitFlag
    thisRayLength = 1;
else
    thisSimLength = thisSimHitFlag*norm(rayOrigin-thisSimPt);
    thisRealLength = thisRealHitFlag*norm(rayOrigin-thisRealPt);
    thisRayLength = max(thisSimLength,thisRealLength);
end
thisRayPts = genPtsRay(rayOrigin,thisRayDirn,thisRayLength);

modelNbrRadius = 0.5;
triModelsNbr = createTriModelsNbr(triModels,thisRayPts,modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,thisRayPts,modelNbrRadius);

%% viz ray
hfig = figure(); axis equal;
grid on; box on;

drawPacket(hfig,rayOrigin,thisRayDirn,thisRealPt,thisRealHitFlag,'b');
drawPacket(hfig,rayOrigin,thisRayDirn,thisSimPt,thisSimHitFlag,'r');

drawEllipsoids(hfig,ellipsoidModelsNbr);
drawTriModels(hfig,triModelsNbr,'ground');

title(sprintf('packet id: %d, ray id: %d',packetIdx,rayId));







