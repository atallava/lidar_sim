function triModels_2 = applyTransfToTriModels(triModels_1,T_1_to_2)
    %APPLYTRANSFTOTRIMODELS
    %
    % model_2 = APPLYTRANSFTOTRIMODELS(model_1,T_1_to_2)
    %
    % model_1  -
    % T_1_to_2 -
    %
    % model_2  -
    
    triModels_2 = triModels_1;
    triModels_2.ptsFit = applyTransf(triModels_1.ptsFit,T_1_to_2);
end