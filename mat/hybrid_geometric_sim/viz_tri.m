% load
relPathTriModels = 'ground_model';
load(relPathTriModels,'groundTriModel');

relPathPts = 'rim_stretch_ground_train';
load(relPathPts,'pts');

%% viz
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});

triModelData = groundTriModel;
plotStruct.triModelData = triModelData;

% plotStruct.pts = pts(1:15:end,:);

hfig = plotRangeData(plotStruct);