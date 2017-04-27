function covMat2 = applyTransfToCov(covMat1,T_1_to_2)
    %APPLYTRANSFTOCOV
    %
    % covMat2 = APPLYTRANSFTOCOV(covMat1,T_1_to_2)
    %
    % covMat1  -
    % T_1_to_2 -
    %
    % covMat2  -
    
    R_1_to_2 = T_1_to_2(1:3,1:3);
    R_2_to_1 = R_1_to_2';
    covMat2 = R_1_to_2*covMat1*R_2_to_1;
end