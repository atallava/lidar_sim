function phrase = printVecAsStlInput(vec)
    %PRINTVECASSTLINPUT
    %
    % phrase = PRINTVECASSTLINPUT(vec)
    %
    % vec    -
    %
    % phrase -
    
    phrase = '{';
    for i = 1:length(vec)
        phrase = [phrase num2str(vec(i)) ','];
    end
    phrase(end) = []; % throw away last comma
    phrase = [phrase '};'];
end