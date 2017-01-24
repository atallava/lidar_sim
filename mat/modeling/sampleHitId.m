function [hitId,hitFlag] = sampleHitId(permVec,targetIds)
    %SAMPLEHITID
    %
    % [hitId,hitFlag] = SAMPLEHITID(permVec,targetIds)
    %
    % permVec   - length nTargets vector.
    % targetIds - length nTargets vector.
    %
    % hitId     - scalar.
    % hitFlag   - boolean.
    
    nTargets = length(permVec);
    if nargin < 2
        targetIds = 1:nTargets;
    end
    for i = 1:nTargets
        if rand < permVec(i)
            hitId = targetIds(i);
            hitFlag = true;
            return;
        end
    end
    hitId = false;
    hitFlag = false;
end