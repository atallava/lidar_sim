function hfig = vizSceneAnnotation(sceneAnnotation, primitiveClasses, colorPerClass)
%VIZSCENEANNOTATION Feel free to play with settings.
%
% hfig = VIZSCENEANNOTATION(sceneAnnotation)
%
% sceneAnnotation - struct.
%
% hfig            - figure handle.

axesLength = 5;
lineWidth = 4;
tagFontSize = 15;
tagFontWeight = 'bold';
axFontSize = 15;

hfig = figure();
hold on;
box on;
axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
set(gca, 'fontsize', axFontSize);

nAnno = length(sceneAnnotation);
for i = 1:nAnno
    objectAnnotation = sceneAnnotation{i};
    objectColor = colorPerClass(objectAnnotation.objectClass, :);
    
    % draw axes, xyz
    [axesOrigin, axesEnds] = getAxes3(objectAnnotation.T_object_to_world, axesLength);
%     plot3([axesOrigin(1) axesEnds(1,1)], [axesOrigin(2) axesEnds(1,2)], [axesOrigin(3) axesEnds(1,3)], ...
%         'color', objectColor, 'linewidth', lineWidth);
%     plot3([axesOrigin(1) axesEnds(2,1)], [axesOrigin(2) axesEnds(2,2)], [axesOrigin(3) axesEnds(2,3)], ...
%         'color', objectColor, 'linewidth',lineWidth);
%     plot3([axesOrigin(1) axesEnds(3,1)], [axesOrigin(2) axesEnds(3,2)], [axesOrigin(3) axesEnds(3,3)], ...
%         'color', objectColor, 'linewidth',lineWidth);
    
    % draw obb
    obb = objectAnnotation.objectObb_world;
    drawObb(hfig, obb, [], objectColor);
    
    % text tag
    tagLocationPad = [-10 -5 0.5];
    tagLocation = obb.center + tagLocationPad;
    tagText = primitiveClasses{objectAnnotation.objectClass};
    tagText = replaceUnderscoreWithSpace(tagText);
    
    % attempting to prevent cluttered tags
    if (mod(i,5) == 0)
        text(tagLocation(1), tagLocation(2), tagLocation(3), ...
            tagText, 'color', objectColor, 'fontSize', tagFontSize, 'fontWeight', tagFontWeight);
    end
end

end