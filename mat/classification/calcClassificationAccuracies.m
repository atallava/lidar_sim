function accuracies = calcClassificationAccuracies(labels,labelsPred,classes)
nClasses = length(classes);
accuracies = zeros(1,nClasses);
for i = 1:nClasses
    thisClass = classes(i);
    flag = (labels == thisClass);
    accuracies(i) = sum(labelsPred(flag) == thisClass)/sum(flag);
end
end