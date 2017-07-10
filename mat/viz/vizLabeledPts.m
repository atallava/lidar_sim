function hfig = vizLabeledPts(pts,labels,classNames)

marker = '.';
markerSizeData = 20;
classColors = distinguishable_colors(length(classNames));
ptColors = getPtColors(labels,classColors);

hfig = figure;
hold on; axis equal;
box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

scatter3(pts(:,1),pts(:,2),pts(:,3), ...
    'marker',marker,'sizeData',markerSizeData,'CData',ptColors);

%% legend
% plot one point, make invisible, legend for those only
nClasses = length(classNames);
classColorMarkerHandles = gobjects(1,nClasses);
legendEntries = cell(1,nClasses);
pt = pts(1,:);
for i = 1:nClasses
    classColorMarkerHandles(i) = scatter3(pt(1),pt(2),pt(3),...
        'markerEdgeColor',classColors(i,:),'markerFaceColor',classColors(i,:),'visible','off');
    
    legendEntries{i} = sprintf('%s', ...
        replaceUnderscoreWithSpace(classNames{i}));
end
hlegend = legend(classColorMarkerHandles,legendEntries);
% legendFontSize = 8;
% set(hlegend,'fontsize',legendFontSize);
legendPosn = get(hlegend,'position');
legendPosn = legendPosn + [1 1 0 0]*0.1;
set(hlegend,'position',legendPosn);
end

function ptColors = getPtColors(labels,classColors)
classes = unique(labels);
nPts = length(labels);
ptColors = zeros(nPts,3);
for i = 1:length(classes)
    class = classes(i);
    flag = (labels == class);
    % class+1 since class begins at 0
    ptColors(flag,:) = repmat(classColors(class+1,:),sum(flag),1);
end
end