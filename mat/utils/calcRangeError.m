function [hitErrorsMean,precision,recall] = calcRangeError(simDetail,dispFlag)
%CALCRANGEERROR
%
% [hitErrorsMean,precision,recall] = CALCRANGEERROR(simDetail,dispFlag)
%
% simDetail     - struct. 
% dispFlag      - scalar. defaults to 0.
%
% hitErrorsMean - scalar.
% precision     - scalar.
% recall        - scalar.

if nargin < 2
    dispFlag = 0;
end

nPackets = size(simDetail.rayOrigins,1);
[packetTrueHits,packetFalseMisses,packetFalseHits,packetTrueMisses] = ...
    deal(zeros(1,nPackets));
[packetHitErrors,packetTrueHitsRealRanges] = deal(cell(1,nPackets));

for i = 1:nPackets
    rayOrigin = simDetail.rayOrigins(i,:);
    realPtsAll = simDetail.realPtsAllCell{i};
    realHitFlag = simDetail.realHitFlagCell{i};
    simPtsAll = simDetail.simPtsAllCell{i};
    simHitFlag = simDetail.simHitFlagCell{i};
    
    [trueHits,falseMisses,falseHits,trueMisses] = deal(0);
    [hitErrors,trueHitsRealRanges] = deal([]);
    
    nRays = size(realPtsAll,1);
    for j = 1:nRays
        condn1 = (realHitFlag(j) == 1);
        condn2 = (simHitFlag(j) == 1);
        
        if (condn1 && ~condn2)
            falseMisses = falseMisses+1;
        elseif (condn1 && condn2)
            trueHits = trueHits+1;
            hitErrors = [hitErrors norm(realPtsAll(j,:)-simPtsAll(j,:))];
            trueHitsRealRanges = [trueHitsRealRanges norm(realPtsAll(j,:)-rayOrigin)];
        elseif (~condn1 && condn2)
            falseHits = falseHits+1;
        else
            trueMisses = trueMisses+1;
        end
    end
    
    packetTrueHits(i) = trueHits;
    packetFalseMisses(i) = falseMisses;
    packetFalseHits(i) = falseHits;
    packetTrueMisses(i) = trueMisses;
    packetHitErrors{i} = hitErrors;
    packetTrueHitsRealRanges{i} = trueHitsRealRanges;
end

totalTrueHits = sum(packetTrueHits);
totalFalseMisses = sum(packetFalseMisses);
totalFalseHits = sum(packetFalseHits);
totalTrueMisses = sum(packetTrueMisses);
precision = totalTrueHits/(totalTrueHits+totalFalseHits);
recall = totalTrueHits/(totalTrueHits+totalFalseMisses);
f1Score = calcF1Score(precision,recall);

hitErrorsAcrossPackets = [];
for i = 1:length(packetHitErrors)
    hitErrorsAcrossPackets = [hitErrorsAcrossPackets ...
        packetHitErrors{i}];
end
hitErrorsMean = mean(hitErrorsAcrossPackets);

trueHitsRealRangesAcrossPackets = [];
for i = 1:length(packetHitErrors)
    trueHitsRealRangesAcrossPackets = [trueHitsRealRangesAcrossPackets ...
        packetTrueHitsRealRanges{i}];
end

if dispFlag
    fprintf('hit errors across packets: %.3f %.3f\n', ...
        mean(hitErrorsAcrossPackets),var(hitErrorsAcrossPackets));
    fprintf('real ranges corresponding to true hits, across packets: %.3f, %.3f\n', ...
        mean(trueHitsRealRangesAcrossPackets),var(trueHitsRealRangesAcrossPackets));
    fprintf('totalTrueHits: %d\n',totalTrueHits);
    fprintf('totalFalseMisses: %d\n',totalFalseMisses);
    fprintf('totalFalseHits: %d\n',totalFalseHits);
    fprintf('precision: %.3f, recall: %.3f, f1: %.3f\n',precision,recall,f1Score);
end
end
