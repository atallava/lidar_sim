someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

%%
nClasses = 11;
colors = distinguishable_colors(nClasses);

legendEntries = cell(1,nClasses);
classColorMarkerHandles = gobjects(1,nClasses);
hfig = figure(); hold on;
for i = 1:nClasses
    classColorMarkerHandles(i) = scatter(0,0,...
        'markerEdgeColor',colors(i,:),'markerFaceColor',colors(i,:),'visible','on');
    
    legendEntries{i} = sprintf('%d: %s',i, ...
        replaceUnderscoreWithSpace(primitiveClasses{i}));
end

%%
legend(classColorMarkerHandles,legendEntries);
% hlegend = legend(classColorMarkerHandles,legendEntries);
% set(hlegend,'fontsize',legendFontSize);
