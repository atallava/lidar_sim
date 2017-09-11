%% rel path helpers
genRelPathPrimitiveStats = @(sectionId) ...
    sprintf('../data/sections/section_%02d/primitives/primitive_stats',sectionId);

%% load
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

sectionId = 3;
relPathPrimitiveStats = genRelPathPrimitiveStats(sectionId);
load(relPathPrimitiveStats,'nElementsPerClass','nCellsPerClass', ...
    'nElementPtsPerClass','nElementEllipsoidsPerClass','htsPerClass');

someUsefulPaths;
relPathLatexTable = [pathToM '/' 'eliduenisch-latexTable-5212622'];
addpath(relPathLatexTable);

%% create mat table
nClasses = length(primitiveClasses);
rowNames = cell(nClasses,1);
for i = 1:nClasses
    rowNames{i} = replaceUnderscoreWithSpace( ...
        primitiveClasses{i});
end
[nElements,meanNElementPts,meanNElementEllipsoids,meanHts] = deal(cell(nClasses,1));
writeStd = 0;
for i = 1:nClasses
    nElements{i} = sprintf('$%d$',nElementsPerClass(i));
    nElementPts = nElementPtsPerClass{i};
    nElementEllipsoids = nElementEllipsoidsPerClass{i};
    hts = htsPerClass{i};
    if writeStd
        meanNElementPts{i} = sprintf('$%d \\pm %d$', ...
            floor(mean(nElementPts)),3*floor(std(nElementPts)));
        meanNElementEllipsoids{i} = sprintf('$%d \\pm %d$', ...
            floor(mean(nElementEllipsoids)),3*floor(std(nElementEllipsoids)));
        meanHts{i} = sprintf('$%0.1f \\pm %0.1f$', ...
            mean(hts),3*std(hts));
    else
        meanNElementPts{i} = sprintf('$%d$',floor(mean(nElementPts)));
        meanNElementEllipsoids{i} = sprintf('$%d$',floor(mean(nElementEllipsoids)));
        meanHts{i} = sprintf('$%0.1f$',mean(hts));
    end
end

T = table(nElements,meanNElementPts,meanNElementEllipsoids,meanHts,'rowNames',rowNames);

%% tex table
clear input;
input.data = T;
input.transposeTable = 0;
input.tableBorders = 0;
input.tableColumnAlignment = 'l';
latex = latexTable(input);

