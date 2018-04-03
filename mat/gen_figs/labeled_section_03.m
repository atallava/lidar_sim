% for a snapshot depicting the labeling tool. just a copy of the script in
% scene_sim

%% rel path helpers
genRelPathSetLabeling = @(setId) ...
    sprintf('../data/sections/section_03/labeling/labeling_set_%d',setId);

genRelPathLabelingSetsInfo = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_sets_info.mat',sectionId);

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load 
sectionId = 3;
relPathSetsInfo = genRelPathLabelingSetsInfo(sectionId);
load(relPathSetsInfo,'ptsCell','setCell');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

setIds = 1:5;
nSets = length(setIds);
setLabelings = cell(1,nSets);
for i = 1:nSets
    setId = setIds(i);
    relPathSetLabeling = genRelPathSetLabeling(setId);
    can = load(relPathSetLabeling,'labeling');
    setLabelings{i} = can.labeling;
end

% smash together
ptsCellGlobal = {};
labelingsGlobal = [];
segmentIds = [];
for i = 1:nSets
    ptsCellGlobal = {ptsCellGlobal{:} ptsCell{setCell{i}}}; % confusing
    labelingsGlobal = [labelingsGlobal setLabelings{i}];
    segmentIds = [segmentIds setCell{i}];
end

relPathLabelingsToViz = 'labelings_to_viz';
clear can;
can.labeling = labelingsGlobal;
save('labelings_to_viz','-struct','can');

%% viz
labelingData.loadPartialLabeling = 1;
labelingData.relPathPartialLabeling = '../data/sections/section_03/labeling/smoothed_labeling';

relPathPoseLog = '../data/pose_log.mat';
can = load(relPathPoseLog);
imuData.poseLog = can.poseLog;
imuData.tLog = can.tLog;
imuData.tExtents = [1403045836.830185000 1403045902.775886000]+[-10 10];
imuData.tResn = 4;

%%
% changes to labelingTool.
% legendFontSize = 30. tagFontSize = 20. markerSizeData = 300.
labelingTool(ptsCellGlobal,primitiveClasses,labelingData,imuData);


