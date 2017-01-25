% load
relPathEllipsoidModels = 'ellipsoid_models';
load(relPathEllipsoidModels,'ellipsoidModels');

relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

%% viz
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModels;
plotStruct.ellipsoidData = ellipsoidData;

plotStruct.pts = pts(1:10:end,:);

hfig = plotRangeData(plotStruct);