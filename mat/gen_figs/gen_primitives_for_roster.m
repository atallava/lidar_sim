% primitives
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%%
sectionId = 3;
classId = 11;
% elementIdsPerClass = {[1,3],[],[7,8,10],[],[7,10,11,16],[],[1,6],[],[1,2,3,4],[2,9,15,30],[2,6,15]} 
elementId  = 6;

className = primitiveClasses{classId};
relPathPrimitive = genRelPathPrimitive(sectionId,className,elementId);
load(relPathPrimitive,'T_segment_to_world','pts','obb','ellipsoidModels');

% plot
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = ellipsoidModels;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);
drawObb(hfig,obb,pts);
viewAngles = ([0 0]);
view(viewAngles);
% for consistency
zLim = zlim;
zLimRange = 16;
zLim = [zLim(1) zLim(1)+zLimRange];
zlim(zLim);
set(gca,'visible','off');
set(0,'DefaultFigureColor','remove')

% relPathPng = sprintf('elements_pngs/%s_%d.png',className,elementId);
% export_fig(relPathPng,hfig);

