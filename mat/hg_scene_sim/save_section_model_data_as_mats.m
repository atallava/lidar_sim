%% rel path helpers
genRelPathTriangleModels = @(sectionId,simVersion,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles.txt', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathTriangleModelsMat = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathEllipsoidModels = @(sectionId,simVersion,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids.txt', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathEllipsoidModelsMat = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids', ...
    sectionId,simVersion,sectionId,blockId);

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
end

%% ellipsoids
sectionId = 3;
ellipsoidBlockIds = 1:23;
simVersion = '070917';
fprintf('ellipsoid blocks...\n');
for i = 1:length(ellipsoidBlockIds)
    fprintf('block %d...\n',i);
    
    blockId = ellipsoidBlockIds(i);
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId,simVersion,blockId);
    ellipsoidModels = loadEllipsoidModels(relPathEllipsoidModels);
    save(genRelPathEllipsoidModelsMat(sectionId,simVersion,blockId),'ellipsoidModels','relPathEllipsoidModels');
end