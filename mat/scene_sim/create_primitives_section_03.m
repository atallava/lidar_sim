%% rel path helpers
genRelPathPrimitiveWorldFrame = @(sectionId,label,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d_world.mat',sectionId,label,elementId);

genRelPathPrimitive = @(sectionId,label,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,label,elementId);

genRelPathPrimitivePatchCellWorldFrame = @(sectionId,label,elementId,cellId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d_%d_world.mat',sectionId,label,elementId,cellId);

genRelPathPrimitive = @(sectionId,label,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,label,elementId);

genRelPathPrimitiveFig = @(sectionId,label,elementId) ...
    sprintf('../figs/sections/section_%02d/primitives/%s/%d',sectionId,label,elementId);

genRelPathPrimitivePng = @(sectionId,label,elementId) ...
    sprintf('../figs/sections/section_%02d/primitives/%s/%d.png',sectionId,label,elementId);

genRelPathEllipsoids = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.mat', ...
    sectionId,sectionId,blockId);

%% load labeling and segments
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

relPathLabeling = '../data/sections/section_03/labeling/smoothed_labeling.mat';
load(relPathLabeling,'labeling');

relPathLabelingPtsCell = '../data/sections/section_03/labeling/smoothed_labeling_segments.mat';
load(relPathLabelingPtsCell,'ptsCell','segmentIds');

%% load ellipsoids
sectionId = 3;
relPathDir = sprintf('../data/sections/section_%02d',sectionId);
pattern = sprintf('section_%02d_block_([0-9]+)_non_ground_ellipsoids.mat', ...
    sectionId);
[matchingFiles,blockIds] = getPatternMatchingFileIds(relPathDir,pattern);
ellipsoidModelsCell = cell(1,length(blockIds));
for i = 1:length(blockIds)
    relPathEllipsoids = genRelPathEllipsoids(sectionId,blockIds(i));
    can = load(relPathEllipsoids,'ellipsoidModels');
    ellipsoidModelsCell{i} = can.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelsCell);

%%
sectionId = 3;
nLabeledSegments = length(ptsCell);
nClassElements = zeros(1,nLabeledSegments);
clockLocal = tic();
for i = 1:nLabeledSegments
    fprintf('labeled pts %d\n',i);
    
    pts = ptsCell{i};
    label = labeling(i);
    nClassElements(label) = nClassElements(label)+1;
    
    clear('saveStruct');
    saveData.relPathPrimitiveWorldFrame = relPathPrimitiveWorldFrame;
    saveData.relPathPrimitive = relPathPrimitive;
    saveData.relPathPrimitiveFig;
    saveData.relPathPrimitivePng;
    createAndSavePrimitive(pts,ellipsoidModels,saveData);
    
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);





