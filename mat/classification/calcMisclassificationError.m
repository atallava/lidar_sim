function [err,weightedErr] = calcMisclassificationError(labels,labelsPred,classes)
    classDistrib = calcClassDistrib(labels,classes);
    
    flag = (labels ~= labelsPred);
    err = sum(flag)/length(labels);
    
    % labels+1 since labels start at 0
    weightedErr = sum(flag.*classDistrib(labels+1))/length(labels); 
end