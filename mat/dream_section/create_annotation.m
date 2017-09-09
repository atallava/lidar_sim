% cover the ground with medium shrubs
xLims = [-50 50];
yLims = [-30 80];

cellSide = 2;
nXSteps = floor((xLims(2)-xLims(1))/cellSide);
nYSteps = floor((yLims(2)-yLims(1))/cellSide);

% medium shrub height range ~[2.5,5]
% medium tree height range ~[10,20]

%% patch annotation
obbCenterZ = 0.01;

cellObbs = {};
T_cells_to_world = {};
for i = 1:nXSteps
   for j = 1:nYSteps
       x = xLims(1) + (i-1)*cellSide + cellSide*0.5;
       y = yLims(1) + (j-1)*cellSide + cellSide*0.5; 
       center = [x y obbCenterZ];
       ht = uniformSampleInRange(2.5,4,1);
       extents = [-cellSide*0.5 cellSide*0.5; -cellSide*0.5 cellSide*0.5; -ht*0.5 ht*0.5];
       ax1 = [1 0]; ax2 = [0 1];
       obb = struct('center',center,'ax1',ax1,'ax2',ax2,'extents',extents);
       
       T = eye(4,4); T(1:3,4) = center;
       
       cellObbs{end+1} = obb;
       T_cells_to_world{end+1} = T;
   end
end

groundShrubPatchAnnotation.objectClass = 6;
groundShrubPatchAnnotation.cellObbs_world = cellObbs;
groundShrubPatchAnnotation.T_cells_to_world = T_cells_to_world;

%% viz patch obbs
hfig = figure;
axis equal; hold on;
drawObbs(hfig,cellObbs);
plot3([xLims(1) xLims(1)],yLims,[0 0],'r');
plot3([xLims(2) xLims(2)],yLims,[0 0],'r');
plot3(xLims,[yLims(1) yLims(1)],[0 0],'r');
plot3(xLims,[yLims(2) yLims(2)],[0 0],'r');

%% trees
% trees will be in either the left or right box, which leave a column along
% y axis for the vehicle path
boxLeft = [xLims(1) -15; yLims(1) yLims(2)];
boxRight = [15 xLims(2); yLims(1) yLims(2)];
nTrees = 20;
treeAnnotations = cell(1,nTrees);
treeObbs = cell(1,nTrees);
for i = 1:nTrees
    % center
    if rand < 0.5
        x = uniformSampleInRange(boxLeft(1,1),boxLeft(1,2),1);
        y = uniformSampleInRange(boxLeft(2,1),boxLeft(2,2),1);
    else
        x = uniformSampleInRange(boxRight(1,1),boxRight(1,2),1);
        y = uniformSampleInRange(boxRight(2,1),boxRight(2,2),1);
    end
    ht = uniformSampleInRange(10,20,1);
    z = ht*0.5;
    center = [x y z];
    ax1 = [1 0]; ax2 = [0 1];
    
    extentsXLow = uniformSampleInRange(-10,-5,1);
    extentsXHigh = uniformSampleInRange(4,10,1);
    extentsYLow = uniformSampleInRange(-10,-5,1);
    extentsYHigh = uniformSampleInRange(4,10,1);
    extents = [extentsXLow extentsXHigh; extentsYLow extentsYHigh; -ht*0.5 ht*0.5];
    
    obb = struct('center',center,'ax1',ax1,'ax2',ax2,'extents',extents);
    T = eye(4,4); T(1:3,4) = center;
    
    annotation.objectClass = 11;
    annotation.objectObb_world = obb;
    annotation.T_object_to_world = T;
    
    treeObbs{i} = obb;
    treeAnnotations{i} = annotation;
end

%% viz tree obbs
hfig = figure;
axis equal; hold on;
drawObbs(hfig,treeObbs);

%% save
sectionId = 42;
relPathAnnotation = sprintf('../data/sections/section_%02d/scene_annotation',sectionId);
sceneAnnotation = treeAnnotations;
save(relPathAnnotation,'sceneAnnotation');




