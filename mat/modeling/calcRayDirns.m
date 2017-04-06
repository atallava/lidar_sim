function [rayDirns,segmentLengths] = calcRayDirns(raysStart,raysEnd)
    %CALCRAYDIRNS
    %
    % [rayDirns,segmentLengths] = CALCRAYDIRNS(raysStart,raysEnd)
    %
    % raysStart      - [1,3] or [nRays,3] array.
    % raysEnd        - [1,3] or [nRays,3] array.
    %
    % rayDirns       - [nRays,3] array.
    % segmentLengths - [1,nRays] array.
    
    %% input parsing
    nStart = size(raysStart,1);
    nEnd = size(raysEnd,1);
        
    if (nStart == 1) && (nEnd > 1)
        raysStart = repmat(raysStart,nEnd,1);
    end

    if (nEnd == 1) && (nStart > 1)
        raysEnd = repmat(raysEnd,nStart,1);
    end
    
    nStart = size(raysStart,1);
    nEnd = size(raysEnd,1);
        
    condn = (nStart == nEnd);
    msg = sprintf('%s: incorrect sized input. See help. \n',mfilename);
    assert(condn,msg);
    
    %% the actual stuff
    rayDirns = raysEnd-raysStart;
    rowNorms = sqrt(sum(rayDirns.^2,2));
    rayDirns = bsxfun(@rdivide,rayDirns,rowNorms);
    segmentLengths = rowNorms;
end