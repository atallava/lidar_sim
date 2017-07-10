load fisheriris;
X = meas;
Y = species;

%% vanilla test
mdl = fitcecoc(X,Y);

%% testing different learners
learners = 'svm';
mdl = fitcecoc(X,Y,'Learners',learners);    

%% testing optimize
learners = 'svm';
mdl = fitcecoc(X,Y,'OptimizeHyperparameters','auto');

%% testing numeric labels
YNumeric = zeros(size(Y));
YNumeric(strcmp(Y,'setosa')) = 0;
YNumeric(strcmp(Y,'versicolor')) = 1;
YNumeric(strcmp(Y,'virginica')) = 2;

%%
mdlNumeric = fitcecoc(X,YNumeric);

%%
dimX = size(X,2);
Xq = rand(5,dimX)*5;
YPred = predict(mdl,Xq)
YNPred = predict(mdlNumeric,Xq)