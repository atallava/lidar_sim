function dispClassDistrib(labels,classes)
%DISPCLASSDISTRIB
%
% DISPCLASSDISTRIB(labels,classes)
%
% labels  -
% classes -

classDistrib = calcClassDistrib(labels,classes);

for i = 1:length(classes)
    class = classes(i);
    fprintf('class: %f, distrib: %.2f\n',class,classDistrib(i));
end
end