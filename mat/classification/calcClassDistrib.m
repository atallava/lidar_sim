function classDistrib = calcClassDistrib(labels,classes)
nClasses = length(classes);
classDistrib = zeros(1,nClasses);
for i = 1:nClasses
    class = classes(i);
    classDistrib(i) = sum(labels == class)/length(labels); 
end
end