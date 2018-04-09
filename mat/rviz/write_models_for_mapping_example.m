% write models to demonstrate off-road site mapping. these are to be
% vieweing in rviz

sectionIds = [1 3 4];
simVersions = {'130917', '250417', '080917'};
groundModelBlockIdsCell = {1:5, 1:4, 1:5};

%% rel path helpers
genRelPathTriangleModelsBlock = @(sectionId, simVersion, blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathEllipsoidModels = @(sectionId, simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_ellipsoid_models', ...
    sectionId,simVersion);

%% load section models
nSections = length(sectionIds);
ellipsoidModelsCell = cell(1, nSections);
groundTriModelsCell = cell(1, nSections);

for i = 1:nSections
    sectionId = sectionIds(i);
    simVersion = simVersions{i};
    blockIds = groundModelBlockIdsCell{i};
    
    % ground triangles, load and merge individual blocks
    triModelsCell = cell(1, length(blockIds));
    for j = 1:length(blockIds)
        blockId = blockIds(j);
        relPathTriModels = genRelPathTriangleModelsBlock(sectionId, simVersion, blockId);
        load(relPathTriModels, 'triModels');
        triModelsCell{j} = triModels;
    end
    groundTriModelsCell{i} = stitchTriModels(triModelsCell);
    
    % ellipsoids, there exists one mat with all models for scene
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId, simVersion);
    load(relPathEllipsoidModels, 'ellipsoidModels');
    ellipsoidModelsCell{i} = ellipsoidModels;
end

%% only train section
relPathSiteMappingDir = '../data/site_mapping';
mkdir(relPathSiteMappingDir);

relPathGroundTriModels = [relPathSiteMappingDir '/section_3_ground_tri_models.txt'];
relPathEllipsoids = [relPathSiteMappingDir '/section_3_ellipsoids.txt'];

triModelsToWrite = groundTriModelsCell{2};
ellipsoidsToWrite = ellipsoidModelsCell{2};
saveTriModels(relPathGroundTriModels, triModelsToWrite);
saveEllipsoidModels(relPathEllipsoids, ellipsoidsToWrite);

%% train + 4
relPathGroundTriModels = [relPathSiteMappingDir '/sections_34_ground_tri_models.txt'];
relPathEllipsoids = [relPathSiteMappingDir '/sections_34_ellipsoids.txt'];

triModelsToWrite = stitchTriModels(groundTriModelsCell(2:3));
ellipsoidsToWrite = stitchEllipsoidModels(ellipsoidModelsCell(2:3));
saveTriModels(relPathGroundTriModels, triModelsToWrite);
saveEllipsoidModels(relPathEllipsoids, ellipsoidsToWrite);

%% all 
relPathGroundTriModels = [relPathSiteMappingDir '/sections_134_ground_tri_models.txt'];
relPathEllipsoids = [relPathSiteMappingDir '/sections_134_ellipsoids.txt'];

triModelsToWrite = stitchTriModels(groundTriModelsCell);
ellipsoidsToWrite = stitchEllipsoidModels(ellipsoidModelsCell);
saveTriModels(relPathGroundTriModels, triModelsToWrite);
saveEllipsoidModels(relPathEllipsoids, ellipsoidsToWrite);


