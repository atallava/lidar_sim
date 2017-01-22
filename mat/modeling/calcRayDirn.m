function [dirn,segmentLength] = calcRayDirn(rayStart,rayEnd)
    %CALCRAYDIRN
    %
    % [dirn,segmentLength] = CALCRAYDIRN(rayStart,rayEnd)
    %
    % rayStart      - length 3 vector.
    % rayEnd        - length 3 vector.
    %
    % dirn          - length 3 vector.
    % segmentLength - scalar.
    
    dirn = rayEnd-rayStart;
    dirn = dirn/norm(dirn);
    segmentLength = norm(rayEnd-rayStart);
end