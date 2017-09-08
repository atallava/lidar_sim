%% rel path helpers
genRelPathBlockGroundPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/blocks_info/section_%02d_block_%02d_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathBlockGroundPtsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/blocks_info/section_%02d_block_%02d_ground', ...
    sectionId,sectionId,blockId);

genRelPathBlockNonGroundPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/blocks_info/section_%02d_block_%02d_non_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathBlockNonGroundPtsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/blocks_info/section_%02d_block_%02d_non_ground', ...
    sectionId,sectionId,blockId);

%% triangles
sectionId = 4;
triangleBlockIds = 1:5;
fprintf('triangle blocks...\n');
for i = 1:length(triangleBlockIds)
    fprintf('block %d...\n',i);
    
    blockId = triangleBlockIds(i);
    relPathBlockGroundPts = genRelPathBlockGroundPts(sectionId,blockId);
    pts = loadPts(relPathBlockGroundPts);
    save(genRelPathBlockGroundPtsMat(sectionId,blockId),'pts','relPathBlockGroundPts');
end

%% ellipsoids
sectionId = 3;
ellipsoidBlockIds = 1:23;
simVersion = '070917';
fprintf('ellipsoid blocks...\n');
for i = 1:length(ellipsoidBlockIds)
    fprintf('block %d...\n',i);
    
    blockId = ellipsoidBlockIds(i);
    relPathBlockNonGroundPts = genRelPathBlockNonGroundPts(sectionId,blockId);
    pts = loadPts(relPathBlockNonGroundPts);
    save(genRelPathBlockNonGroundPtsMat(sectionId,blockId),'pts','relPathBlockNonGroundPts');
end