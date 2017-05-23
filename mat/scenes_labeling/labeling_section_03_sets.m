%% rel path helpers
genRelPathLabelingSetsInfo = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_sets_info.mat',sectionId);

genRelPathSetLabeling = @(setId) ...
    sprintf('../data/sections/section_03/labeling/labeling_set_%d',setId);

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load
sectionId = 3;
relPathLabelingSetsInfo = genRelPathLabelingSetsInfo(sectionId);
load(relPathLabelingSetsInfo,'ptsCell','tape','setCell');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

% imu poses
relPathPoseLog = '../data/pose_log.mat';
can = load(relPathPoseLog);
imuData.poseLog = can.poseLog;
imuData.tLog = can.tLog;
imuData.tExtents = [1403045836.830185000 1403045902.775886000];
imuData.tResn = 5;

%% label
% pick a set
setId = 1;
segmentIdsInSet = setCell{setId};
nSegmentsInSet = length(segmentIdsInSet);

labelingData.relPathLabelingOut = 'labeling_delete';
labelingData.loadPartialLabeling = 0;
labelingData.relPathPartialLabeling = 'labeling_set_2';

ptsCellToPass = ptsCell(segmentIdsInSet);
labeling = labelingTool(ptsCellToPass,primitiveClasses,labelingData,imuData);

%%
relPathSetLabeling = genRelPathSetLabeling(setId);


