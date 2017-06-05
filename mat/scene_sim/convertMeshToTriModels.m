function triModels = convertMeshToTriModels(meshModel)
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
    
    triModels.ptsFit = meshModel.vertices;
    triModels.tri = meshModel.faces;
end