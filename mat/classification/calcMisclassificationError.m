function err = calcMisclassificationError(labels,labelsPred)
%CALCMISCLASSIFICATIONERROR
%
% err = CALCMISCLASSIFICATIONERROR(labels,labelsPred)
%
% labels     -
% labelsPred -
%
% err        -

flag = (labels ~= labelsPred);
err = sum(flag)/length(labels);
end