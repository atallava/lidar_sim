% divide segments into sets if it is hard to plot everything in mat at once

%% rel path helpers
genRelPathDir = @(sectionId) sprintf('../data/sections/section_%02d/non_ground_segmentation', ...
    sectionId);

genRelPathPtsMat = @(sectionId,ptsId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,ptsId);

genRelPathLabelingSetsInfo = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_sets_info.mat',sectionId);

%% load 
% pts
sectionId = 1;
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
% tape = calcPtsCellTape(ptsCell);
% maxPtsPerSet = 230;
% setCell = splitVecIntoSets(tape,maxPtsPerSet);

% a single set
tape = 1:length(ptsCell);
setCell{1} = tape;

%% save
relPathLabelingSetsInfo = genRelPathLabelingSetsInfo(sectionId);
fprintf('saving to %s\n',relPathLabelingSetsInfo);
save(relPathLabelingSetsInfo,'ptsCell','tape','setCell');
