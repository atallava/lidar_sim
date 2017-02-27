% load
relPathPts = 'section_03_block_02_ground';
load(relPathPts,'pts');

relPathEllipsoidModels = 'section_03_block_02_ground_triangles';
load(relPathEllipsoidModels,'triModel');

%% viz
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});

triModelData = triModel;
plotStruct.triModelData = triModelData;

ptsSkip = 10;
plotStruct.pts = pts(1:ptsSkip:end,:);

hfig = plotRangeData(plotStruct);