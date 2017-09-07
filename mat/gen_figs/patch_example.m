% primitives
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

genRelPathPrimitivePatch = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d',sectionId,className,elementId);

genRelPathPrimitivePatchCell = @(sectionId,className,elementId,cellId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d/%d.mat',...
    sectionId,className,elementId,cellId);

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%% pick patch
sectionId = 3;
classId = 6;
elementId  = 10;

className = primitiveClasses{classId};
relPathPrimitivePatch = genRelPathPrimitivePatch(sectionId,className,elementId);
% get the cell ids in that patch
pattern = '([0-9]+)';
[~,cellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);

%% viz
hfig = figure;
hold on; axis equal;
for i = 1:length(cellIds)
    cellId = cellIds(i);
    relPathPrimitivePatchCell = genRelPathPrimitivePatchCell(sectionId,className,elementId,cellId);
    % draw the ellipsoid models, the pts and the obb
    load(relPathPrimitivePatchCell,'ellipsoidModels','pts','obb','T_segment_to_world');
    % transform things back to world
    obb = applyTransfToObb(obb,T_segment_to_world);
    pts = applyTransf(pts,T_segment_to_world);
    ellipsoidModels = applyTransfToEllipsoids(ellipsoidModels,T_segment_to_world);
    drawObb(hfig,obb,pts);
    drawEllipsoids(hfig,ellipsoidModels);
end

%% adjust
box on; grid on;
viewAngles = [94 32];
view(viewAngles);
fontSize = 15;

% for consistency
zLim = zlim;
zLimRange = 15;
zLim = [zLim(1) zLim(1)+zLimRange];
zlim(zLim);

% center the axes ticks
xt = get(gca,'xtick');
xt = flipVecToColumn(xt);
xtc = centerData(xt);
set(gca,'xticklabel',xtc);

yt = get(gca,'ytick');
yt = flipVecToColumn(yt);
ytc = centerData(yt);
set(gca,'yticklabel',ytc);

set(gca,'FontSize',fontSize);

xlabel('x (m)','FontSize',fontSize); ylabel('y (m)','FontSize',fontSize); zlabel('z (m)','FontSize',fontSize);

% set(gca,'visible','off');

% relPathPng = sprintf('elements_pngs/%s_%d.png',className,elementId);
% export_fig(relPathPng,hfig);

