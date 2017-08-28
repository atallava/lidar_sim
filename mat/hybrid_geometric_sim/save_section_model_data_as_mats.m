%% rel path helpers
genRelPathTriangleModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground_triangles.txt', ...
    sectionId,sectionId,blockId);

genRelPathTriangleModelsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sectionId,blockId);

genRelPathBlockGroundPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathBlockGroundPtsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground', ...
    sectionId,sectionId,blockId);

genRelPathEllipsoidModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.txt', ...
    sectionId,sectionId,blockId);

genRelPathEllipsoidModelsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids', ...
    sectionId,sectionId,blockId);

genRelPathBlockNonGroundPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathBlockNonGroundPtsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground', ...
    sectionId,sectionId,blockId);

%% triangles
sectionId = 4;
triangleBlockIds = 1:5;
fprintf('triangle blocks...\n');
for i = 1:length(triangleBlockIds)
    fprintf('block %d...\n',i);
    
    blockId = triangleBlockIds(i);
    relPathTriangleModels = genRelPathTriangleModels(sectionId,blockId);
    triModels = loadTriModels(relPathTriangleModels);
    save(genRelPathTriangleModelsMat(sectionId,blockId),'triModels','relPathTriangleModels');
    relPathBlockGroundPts = genRelPathBlockGroundPts(sectionId,blockId);
    pts = loadPts(relPathBlockGroundPts);
    save(genRelPathBlockGroundPtsMat(sectionId,blockId),'pts','relPathBlockGroundPts');
end

%% ellipsoids
sectionId = 3;
ellipsoidBlockIds = 1:23;
fprintf('ellipsoid blocks...\n');
for i = 1:length(ellipsoidBlockIds)
    fprintf('block %d...\n',i);
    
    blockId = ellipsoidBlockIds(i);
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId,blockId);
    ellipsoidModels = loadEllipsoidModels(relPathEllipsoidModels);
    save(genRelPathEllipsoidModelsMat(sectionId,blockId),'ellipsoidModels','relPathEllipsoidModels');
    relPathBlockNonGroundPts = genRelPathBlockNonGroundPts(sectionId,blockId);
    pts = loadPts(relPathBlockNonGroundPts);
    save(genRelPathBlockNonGroundPtsMat(sectionId,blockId),'pts','relPathBlockNonGroundPts');
end