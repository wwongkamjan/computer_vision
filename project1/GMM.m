k=5;
%training
%[p,meank,covk]=trainGMM(10)

%testing
test_image = imread('train_images/280.jpg');
probM = testGMM(test_image, p, meank, covk, k);
dist = measureDepth(probM);
plotGMM(probM,covk,meank,k,dist)

