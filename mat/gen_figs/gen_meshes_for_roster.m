% mesh primitives
genRelPathMeshPrimitive = @(className,elementId) ...
    sprintf('../data/3d_models/primitives/%s/%d',className,elementId);

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%%
classId = 10;
% elementIdsPerClass = {[1,3],[],[],[],[7,10,11,16],[],[],[],[1,2,3,4],[],[2,6,15]} 
elementId  = 12;

className = primitiveClasses{classId};
relPathMeshPrimitive = genRelPathMeshPrimitive(className,elementId);
load(relPathMeshPrimitive,'triModels');

% plot
hfig = vizTriModels(triModels);
viewAngles = ([0 0]);
view(viewAngles);
% for consistency
zLim = zlim;
zLimRange = 16;
zLim = [zLim(1) zLim(1)+zLimRange];
zlim(zLim);
set(gca,'visible','off');

% relPathPng = sprintf('elements_pngs/%s_%d.png',className,elementId);
% export_fig(relPathPng,hfig);

