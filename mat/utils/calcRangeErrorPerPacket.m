function [packetHitErrorsMean,packetPrecisions,packetRecalls] = calcRangeErrorPerPacket(simDetail)
%CALCRANGEERRORPERPACKET
%
% [packetHitErrorsMean,packetPrecisions,packetRecalls] = CALCRANGEERRORPERPACKET(simDetail)
%
% simDetail           -
%
% packetHitErrorsMean -
% packetPrecisions    -
% packetRecalls       -

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

[packetHitErrorsMean,packetPrecisions,packetRecalls] = deal(zeros(1,nPackets));
for i = 1:nPackets
    packetHitErrorsMean(i) = mean(packetHitErrors{i});
    packetPrecisions(i) = packetTrueHits(i)/(packetTrueHits(i)+packetFalseHits(i));
    packetRecalls(i) = packetTrueHits(i)/(packetTrueHits(i)+packetFalseMisses(i));
end

end
