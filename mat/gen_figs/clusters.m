% load
relPathEllipsoidModels = 'ellipsoid_models';
load(relPathEllipsoidModels,'ellipsoidModels');

relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

%% viz
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});

% subsample ellipsoids
ellipsoidModelIdsToPlot = 1:length(ellipsoidModels);
ellipsoidData.ellipsoidModels = ellipsoidModels(ellipsoidModelIdsToPlot);
plotStruct.ellipsoidData = ellipsoidData;

% subsample pts
ptsSkip = 10;
plotStruct.pts = pts(1:ptsSkip:end,:);

hfig = plotRangeData(plotStruct);