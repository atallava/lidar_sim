% cover the ground with medium shrubs
xLims = [-70 70];
yLims = [-30 150];

%% trees
% trees will be in either the left or right box, which leave a column along
% y axis for the vehicle path
boxLeft = [xLims(1) -7; yLims(1) yLims(2)];
boxRight = [7 xLims(2); yLims(1) yLims(2)];
nTrees = 30;
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




