relPathEllipsoids = 'ellipsoid_models_for_sim_nbr';
load(relPathEllipsoids,'ellipsoidModels');

%% for a pt
nEllipsoids = length(ellipsoidModels);
id = 1262;%randsample(nEllipsoids,1);
pt = ellipsoidModels(id).mu;
radius = 1;
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,pt,radius);

%%
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModelsNbr;
plotStruct.ellipsoidData = ellipsoidData;

hfig = plotRangeData(plotStruct);
scatter3(pt(:,1),pt(:,2),pt(:,3),'r.','sizedata',40);

%% for pts
ids = [1262 500 100];
pts = [];
for i = 1:length(ids)
    pts(i,:) = ellipsoidModels(i).mu;
end
radius = 1;
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,pts,radius);

%%
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModelsNbr;
plotStruct.ellipsoidData = ellipsoidData;

hfig = plotRangeData(plotStruct);
scatter3(pts(:,1),pts(:,2),pts(:,3),'r.','sizedata',40);

%% a ray
rayOrigin = ellipsoidModels(1262).mu;
rayEnd = ellipsoidModels(100).mu;
rayDirn = calcRayDirn(rayOrigin,rayEnd);
radius = 2;
nodesAlongRay = getNodesAlongRay(rayOrigin,rayDirn,0,100,radius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,nodesAlongRay,radius);

%%
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModelsNbr;
plotStruct.ellipsoidData = ellipsoidData;

hfig = plotRangeData(plotStruct);
scatter3(nodesAlongRay(:,1),nodesAlongRay(:,2),nodesAlongRay(:,3),'r.','sizedata',40);
