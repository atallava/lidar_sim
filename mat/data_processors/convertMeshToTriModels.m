function triModels = convertMeshToTriModels(meshModel,hitProbVec)
    %CONVERTMESHTOTRIMODELS
    %
    % triModels = CONVERTMESHTOTRIMODELS(meshModel)
    %
    % meshModel - struct. fields ('vertices','faces').
    %
    % triModels - struct. fields ('fitPts','tri').
    
    condn = (size(meshModel.faces,2) == 3);
    msg = sprintf('Expected triangle faces.\n');
    assert(condn,msg);
    
    if nargin < 2
        hitProbVec = ones(1,size(meshModel.faces,1));
    end
    
    triModels.ptsFit = meshModel.vertices;
    triModels.tri = meshModel.faces;
    triModels.hitProbVec = hitProbVec;
    triModels.rangeVar = 0;
end