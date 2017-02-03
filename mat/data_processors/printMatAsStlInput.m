function phrase = printMatAsStlInput(mat)
    phrase = '{';
    for i = 1:size(mat,1)
        phrase = sprintf('%s\n',phrase);
        vecPhrase = printVecAsStlInput(mat(i,:));
        vecPhrase(end) = []; % throw away semicolon
        phrase = [phrase vecPhrase ','];
    end
    phrase(end) = []; % throw away last comma
    phrase = [phrase '};'];
end