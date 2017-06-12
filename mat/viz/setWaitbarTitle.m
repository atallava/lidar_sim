function oldTitle = setWaitbarTitle(hWaitbar,newTitle)
    %SETWAITBARTITLE
    %
    % oldTitle = SETWAITBARTITLE(hWaitbar,newTitle)
    %
    % hWaitbar - waitbar handle.
    % newTitle - string.
    %
    % oldTitle - string.
    
    hAx = findobj(hWaitbar,'type','axes');
    hTitle = get(hAx,'title');
    oldTitle = get(hTitle,'string');
    set(hTitle,'string',newTitle);
end