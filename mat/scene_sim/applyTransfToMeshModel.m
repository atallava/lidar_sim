function model_2 = applyTransfToMeshModel(model_1,T_1_to_2)
    %APPLYTRANSFTOMESHMODEL
    %
    % model_2 = APPLYTRANSFTOMESHMODEL(model_1,T_1_to_2)
    %
    % model_1  -
    % T_1_to_2 -
    %
    % model_2  -
    
    model_2 = model_1;
    model_2.vertices = applyTransf(model_1.vertices,T_1_to_2);
end