sectionId = 42;

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load
% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

% annotations
relPathAnnotation = sprintf('../data/sections/section_%02d/scene_annotation', sectionId);
load(relPathAnnotation, 'sceneAnnotation');

nClasses = 11;
colorPerClass = distinguishable_colors(nClasses);

%% viz
axesLength = 5;
lineWidth = 2;
tagFontSize = 10;
axFontSize = 10;

hfig = figure();
hold on; box on;
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
set(gca, 'fontsize', axFontSize);

nAnno = length(sceneAnnotation);
for i = 1:nAnno
    objectAnnotation = sceneAnnotation{i};
    objectColor = colorPerClass(objectAnnotation.objectClass, :);
    
    % draw axes
    [axesOrigin, axesEnds] = getAxes3(objectAnnotation.T_object_to_world, axesLength);
    % x axis
    plot3([axesOrigin(1) axesEnds(1,1)], [axesOrigin(2) axesEnds(1,2)], [axesOrigin(3) axesEnds(1,3)], ...
        'color', objectColor, 'linewidth', lineWidth);
    % y axis
    plot3([axesOrigin(1) axesEnds(2,1)], [axesOrigin(2) axesEnds(2,2)], [axesOrigin(3) axesEnds(2,3)], ...
        'color', objectColor, 'linewidth',lineWidth);
    % z axis
    plot3([axesOrigin(1) axesEnds(3,1)], [axesOrigin(2) axesEnds(3,2)], [axesOrigin(3) axesEnds(3,3)], ...
        'color', objectColor, 'linewidth',lineWidth);
    
    % draw obb
    obb = objectAnnotation.objectObb_world;
    drawObb(hfig, obb, [], objectColor);
    
    % text tag
    tagZPad = 0.5;
    tagLocation = obb.center;
    tagLocation(3) = tagLocation(3) + tagZPad;
    tagText = primitiveClasses{objectAnnotation.objectClass};
    tagText = replaceUnderscoreWithSpace(tagText);
    text(tagLocation(1), tagLocation(2), tagLocation(3), tagText, 'color', objectColor);
end

%%
vizSceneAnnotation(sceneAnnotation, primitiveClasses, colorPerClass);
