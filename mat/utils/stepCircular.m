function newPosn = stepCircular(vecLength,currentPosn,stepLength)
    %STEPCIRCULAR
    %
    % newPosn = STEPCIRCULAR(vecLength,currentPosn,stepLength)
    %
    % vecLength   -
    % currentPosn -
    % stepLength  -
    %
    % newPosn     -
    
    newPosn = modN(currentPosn+stepLength,vecLength);
end