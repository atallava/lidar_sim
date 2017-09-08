function elementId = pickNearestObb(objectObb,primitiveObbs)
%PICKNEARESTOBB
%
% elementId = PICKNEARESTOBB(objectObb,primitiveObbs)
%
% objectObb     -
% primitiveObbs -
%
% elementId     -

nPrimitiveObbs = length(primitiveObbs);
metricVec = zeros(1,nPrimitiveObbs);
for i = 1:nPrimitiveObbs
    primitiveObb = primitiveObbs{i};
    metricVec(i) = calcObbMetric(objectObb,primitiveObb);
end
[sortedMetricVec,sortedIds] = sort(metricVec);
elementId = sortedIds(1);
end