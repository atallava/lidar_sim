function dispRangeError(simDetail)
%DISPRANGEERROR
%
% DISPRANGEERROR(simDetail)
%
% simDetail -

nPackets = size(simDetail.rayOrigins,1);

[meanErrors,trueHits,falseMisses,falseHits,trueMisses] = ...
    deal(zeros(1,nPackets));

for i = 1:nPackets
    realPtsAll = simDetail.realPtsAllCell{i};
    realHitFlag = simDetail.realHitFlagCell{i};
    simPtsAll = simDetail.simPtsAllCell{i};
    simHitFlag = simDetail.simHitFlagCell{i};
    
    nRays = size(realPtsAll,1);
    flag = realHitFlag & simHitFlag;
    trueHits(i) = sum(flag);
    mat = simPtsAll(flag,:)-realPtsAll(flag,:);
    mat = sum(mat.^2,2); 
    thisPacketErrors = sqrt(mat);
    meanErrors(i) = mean(thisPacketErrors);
    
    flag = realHitFlag & ~simHitFlag;
    falseMisses(i) = sum(flag);
    
    flag = ~realHitFlag & simHitFlag;
    falseHits(i) = sum(flag);
    
    flag = ~realHitFlag & ~simHitFlag;
    trueMisses(i) = sum(flag);
end

precision = sum(trueHits)/(sum(trueHits)+sum(falseHits));
recall = sum(trueHits)/(sum(trueHits)+sum(falseMisses));
f1Score = (2*precision*recall)/(precision+recall);

fprintf('mean errors: \n'); dispHelper(meanErrors);
fprintf('true hits: \n'); dispHelper(trueHits);
fprintf('false misses: \n'); dispHelper(falseMisses);
fprintf('false hits: \n'); dispHelper(falseHits);
fprintf('true misses: \n'); dispHelper(trueMisses);
fprintf('precision: %.2f, recall: %.2f, f1 score: %.2f\n',precision,recall,f1Score);
end

function dispHelper(vec)
fprintf('mean: %.2f, var: %.2f\n',mean(vec),var(vec));
end