%% rel path helpers
genRelPathDir = @(sectionId) sprintf('../data/sections/section_%02d/non_ground_segmentation', ...
    sectionId);

genRelPathPtsMat = @(sectionId,ptsId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,ptsId);

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load 
% pts
sectionId = 3;
relPathDir = genRelPathDir(sectionId);
pattern = '([0-9]+).mat';
[matchingFiles,fileIds] = getPatternMatchingFileIds(relPathDir,pattern);

nSegments = length(fileIds);
ptsCell = cell(1,nSegments);
for i = 1:length(fileIds)
    segmentId = fileIds(i);
    relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
    load(relPathPtsMat,'pts');
    ptsCell{i} = pts;
end

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

%% divide into sets
tape = calcPtsCellTape(ptsCell);
maxPtsPerSet = 40;
setCell = splitVecIntoSets(tape,maxPtsPerSet);

%% label
% pick a set
setId = 5;
segmentIdsInSet = setCell{setId};
nSegmentsInSet = length(segmentIdsInSet);

labelingData.relPathLabelingOut = 'labeling';
labelingData.loadPartialLabeling = 0;
labelingData.relPathPartialLabeling = 'labeling_set_2';

ptsCellToPass = ptsCell(segmentIdsInSet);
labeling = labelingTool(ptsCellToPass,primitiveClasses,labelingData);


