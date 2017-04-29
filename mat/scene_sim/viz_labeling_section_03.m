%% rel path helpers
genRelPathSetLabeling = @(setId) ...
    sprintf('../data/sections/section_03/labeling/labeling_set_%d',setId);

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load 
relPathSetsInfo = '../data/sections/section_03/labeling/labeling_sets_info';
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
ptsCellToViz = {};
labelingsToViz = [];
for i = 1:nSets
    ptsCellToViz = {ptsCellToViz{:} ptsCell{setCell{i}}}; % confusing
    labelingsToViz = [labelingsToViz setLabelings{i}];
end

% pretty hacky save
relPathLabelingsToViz = 'labelings_to_viz';
clear can;
can.labeling = labelingsToViz;
save('labelings_to_viz','-struct','can');

%% viz
labelingData.relPathLabelingOut = 'labelings_delete';
labelingData.loadPartialLabeling = 1;
labelingData.relPathPartialLabeling = 'labelings_to_viz';

labelingTool(ptsCellToViz,primitiveClasses,labelingData);


