%% load
relPathPrimitivesData = '../data/section_03_primitives_data';
load(relPathPrimitivesData,'primitiveClasses','elementIdsCell','elementPtsCell');

%% calc features
nClasses = length(primitiveClasses);
nElementsPerClass = calcNElementsPerClass(elementIdsCell);
featuresCell = cell(1,nClasses);
for i = 1:length(primitiveClasses)
    elementPts = elementPtsCell{i};
    elementIds = elementIdsCell{i};
    nElements = length(elementIds);
    featuresMat = [];
    for j = 1:nElements
        featuresMat(j,:) = calcPtsFeatures(elementPts{j});
    end
    featuresCell{i} = featuresMat;
end

%% viz errorbar
featureId = 3;

featureMeans = zeros(1,nClasses);
featureStds = zeros(1,nClasses);
for i = 1:nClasses
    vec = mean(featuresCell{i},1);
    featureMeans(i) = vec(featureId);
    vec = std(featuresCell{i},1);
    featureStds(i) = vec(featureId);
end

hfig = figure;
errorbar(1:nClasses,featureMeans,3*featureStds,'--o','linewidth',2);
xlabel('primitive class');
ylabel('feature mean');
title(sprintf('feature %d',featureId));
set(gca,'xtick',1:nClasses); 
grid on;

%% viz hists
featureId = 1;

for i = 1:nClasses
    mat = featuresCell{i};
    vec = mat(:,featureId);
    
    figure;
    hist(vec);
    title(sprintf('feature %d class %d',featureId,i));
end
