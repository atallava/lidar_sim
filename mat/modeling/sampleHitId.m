function [hitId,hitFlag] = sampleHitId(hitProbVec,targetIds)
    %SAMPLEHITID
    %
    % [hitId,hitFlag] = SAMPLEHITID(hitProbVec,targetIds)
    %
    % hitProbVec   - length nTargets vector.
    % targetIds - length nTargets vector.
    %
    % hitId     - scalar.
    % hitFlag   - boolean.
    
    nTargets = length(hitProbVec);
    if nargin < 2
        targetIds = 1:nTargets;
    end
    for i = 1:nTargets
        if rand < hitProbVec(i)
            hitId = targetIds(i);
            hitFlag = true;
            return;
        end
    end
    hitId = false;
    hitFlag = false;
end
