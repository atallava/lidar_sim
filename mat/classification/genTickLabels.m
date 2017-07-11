function tickLabels = genTickLabels(classes)
%GENTICKLABELS
%
% tickLabels = GENTICKLABELS(classes)
%
% classes    -
%
% tickLabels -

nClasses = length(classes);
tickLabels = cell(1,nClasses);
for i = 1:nClasses
    tickLabels{i} = num2str(classes(i));
end
end