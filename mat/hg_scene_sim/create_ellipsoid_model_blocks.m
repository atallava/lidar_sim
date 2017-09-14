% construct_scene_objects saves all ellipsoids in a mat
% sectionModelSim.cpp takes blocks of ellipsoids as input
% this script is the bridge

%% rel path helpers
genRelPathImuPosnNodes = @(sectionId) ...
    sprintf('../../cpp/data/sections/section_%02d/imu_posn_nodes.txt',sectionId);

% scene ellipsoids
genRelPathSceneEllipsoidModelsMat = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_ellipsoid_models',sectionId,simVersion);

genRelPathBlockNodeIdsNonGround = @(sectionId) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/blocks_info/block_node_ids_non_ground.txt',sectionId);

genRelPathEllipsoidsBlock = @(sectionId,simVersion,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids.txt', ...
    sectionId,simVersion,sectionId,blockId);

%% load
sectionId = 1;
simVersion = '130917';
relPathImuPosnNodes = genRelPathImuPosnNodes(sectionId);
imuPosnNodes = loadPts(relPathImuPosnNodes);

relPathEllipsoids = genRelPathSceneEllipsoidModelsMat(sectionId,simVersion);
load(relPathEllipsoids,'ellipsoidModels');

%% make blocks
% NNs for ellipsoid centers in nodes
clockLocal = tic();
ellipsoidCenters = getEllipsoidCenters(ellipsoidModels);
nnIds = knnsearch(imuPosnNodes,ellipsoidCenters);

% for each node, which centers map to it
nNodes = size(imuPosnNodes,1);
nEllipsoids = size(ellipsoidCenters,1);
nodeEllipsoidsMap = cell(1,nNodes);
for i = 1:nEllipsoids
    nodeId = nnIds(i);
    nodeEllipsoidsMap{nodeId} = [nodeEllipsoidsMap{nodeId} i];
end

nodeEllipsoidsCount = zeros(1,nNodes);
for i = 1:nNodes
    nodeEllipsoidsCount(i) = ...
        length(nodeEllipsoidsMap{i});
end

% ellipsoids per block
nEllipsoidsPerBlock = 300;

% make blocks
cumsumNodes = cumsum(nodeEllipsoidsCount);
blockNodeIds = divideCumsumIntoBlocks(cumsumNodes,nEllipsoidsPerBlock);
% cpp indexing is from 0
blockNodeIds = blockNodeIds-1;

% write block node ids
relPathBlockNodeIds = genRelPathBlockNodeIdsNonGround(sectionId);
savePts(relPathBlockNodeIds,blockNodeIds);

% write ellipsoid blocks
nBlocks = size(blockNodeIds,1);
hWaitbar = waitbar(0,'progress');
for i = 1:nBlocks
    fprintf('block %d...\n',i);
    thisBlockEllipsoids = [];
    blockStartNode = blockNodeIds(i,1)+1;
    blockEndNode = blockNodeIds(i,2)+1;
    for j = blockStartNode:blockEndNode
        thisNodeEllipsoidIds = nodeEllipsoidsMap{j};
        thisNodeEllipsoids = ellipsoidModels(thisNodeEllipsoidIds);
        thisBlockEllipsoids = [thisBlockEllipsoids thisNodeEllipsoids];
    end
    
    relPathBlock = genRelPathEllipsoidsBlock(sectionId,simVersion,i);
    fprintf('saving to %s...\n',relPathBlock);
    saveEllipsoidModels(relPathBlock,thisBlockEllipsoids);
    waitbar(i/nBlocks);
    waitbarTitle = sprintf('progress: %d/%d',i,nBlocks);
    setWaitbarTitle(hWaitbar,waitbarTitle);
end
close(hWaitbar);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);





