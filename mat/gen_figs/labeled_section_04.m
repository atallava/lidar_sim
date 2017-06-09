%% rel path helpers
genRelPathLabelingSetsInfo = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_sets_info.mat',sectionId);

genRelPathSetLabeling = @(sectionId,setId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_set_%d',sectionId,setId);

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load
sectionId = 4;
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
imuData.tExtents = [1403045911.411350000 1403046027.954617000];
imuData.tResn = 5;

%% label
% pick a set
setId = 1;
segmentIdsInSet = setCell{setId};
nSegmentsInSet = length(segmentIdsInSet);

labelingData.loadPartialLabeling = 1;
labelingData.relPathPartialLabeling = '../data/sections/section_04/labeling/labeling_section_04.mat';

ptsCellToPass = ptsCell(segmentIdsInSet);
labeling = labelingTool(ptsCellToPass,primitiveClasses,labelingData,imuData);
