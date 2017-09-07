% primitives
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%% pick element
sectionId = 3;
classId = 5;
elementId  = 7; % [7,12,18]

className = primitiveClasses{classId};
relPathPrimitive = genRelPathPrimitive(sectionId,className,elementId);

%% viz
hfig = figure;
hold on; axis equal;
load(relPathPrimitive,'ellipsoidModels','pts','obb');
drawObb(hfig,obb,pts);
drawEllipsoids(hfig,ellipsoidModels);

%% adjust
box on; grid on;
viewAngles = [94 32];
view(viewAngles);
fontSize = 15;

% for consistency
zLim = zlim;
zLimRange = 4;
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

set(gca,'visible','off');

% relPathPng = sprintf('elements_pngs/%s_%d.png',className,elementId);
% export_fig(relPathPng,hfig);

