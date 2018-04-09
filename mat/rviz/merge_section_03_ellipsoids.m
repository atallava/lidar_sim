% this is for easy processing in the mapping example
sectionId = 3;
simVersion = '250417';

%%
genRelPathEllipsoidModelsMat = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids', ...
    sectionId,simVersion,sectionId,blockId);

%%
blockIds = 1:23;
nBlocks = length(blockIds);
ellipsoidModelsCell = cell(1, nBlocks);
for i = 1:length(blockIds)
    blockId = blockIds(i);
    relPathModels = genRelPathEllipsoidModelsMat(sectionId, simVersion, blockId);
    load(relPathModels, 'ellipsoidModels');
    ellipsoidModelsCell{i} = ellipsoidModels;
end

ellipsoidModels = stitchEllipsoidModels(ellipsoidModelsCell);
relPathOut = sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_ellipsoid_models', ...
    sectionId, simVersion);
save(relPathOut, 'ellipsoidModels');