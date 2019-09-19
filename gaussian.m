% get training images and their roipoly
load training_data

% get data input from RGB of each orange pixel (of each image)

RGBM = toRGBmatrix([], I1, t1);
RGBM = toRGBmatrix(RGBM, I2, t2);
RGBM = toRGBmatrix(RGBM, I3, t3);
RGBM = toRGBmatrix(RGBM, I4, t4);
RGBM = toRGBmatrix(RGBM, I5, t5);
RGBM = toRGBmatrix(RGBM, I6, t6);
RGBM = toRGBmatrix(RGBM, I7, t7);
RGBM = toRGBmatrix(RGBM, I8, t8);
RGBM = toRGBmatrix(RGBM, I9, t9);
RGBM = toRGBmatrix(RGBM, I10, t10);
RGBM = toRGBmatrix(RGBM, I11, t11);
RGBM = toRGBmatrix(RGBM, I12, t12);
RGBM = toRGBmatrix(RGBM, I13, t13);
RGBM = toRGBmatrix(RGBM, I14, t14);
RGBM = toRGBmatrix(RGBM, I15, t15);
RGBM = toRGBmatrix(RGBM, I16, t16);
RGBM = toRGBmatrix(RGBM, I17, t17);
RGBM = toRGBmatrix(RGBM, I18, t18);
RGBM = toRGBmatrix(RGBM, I19, t19);
RGBM = toRGBmatrix(RGBM, I20, t20);
RGBM = toRGBmatrix(RGBM, I21, t21);
RGBM = toRGBmatrix(RGBM, I22, t22);
RGBM = toRGBmatrix(RGBM, I23, t23);

%Color Classification using a Single Gaussian
% find mean
[numRows,numCols] = size(RGBM);
datasize = numCols;
meanRGB = sum(RGBM,2)/datasize

% find cov
meanR = meanRGB(1)'*ones(1,12930);
meanG = meanRGB(2)'*ones(1,12930);
meanB = meanRGB(3)'*ones(1,12930);

R = double(RGBM(1,:))-meanR;
G = double(RGBM(2,:))-meanG;
B = double(RGBM(1,:))-meanB;

covRR = (R*R')/datasize;
covRG = (R*G')/datasize;
covRB = (R*B')/datasize;
covGG = (G*G')/datasize;
covGB = (G*B')/datasize;
covBB = (B*B')/datasize;

covRGB = [covRR covRG covRB; covRG covGG covGB; covRB covGB covBB]

%test images
test1 = imread('test_images/1.jpg');
test2 = imread('test_images/2.jpg');
test3 = imread('test_images/3.jpg');
test4 = imread('test_images/4.jpg');
test5 = imread('test_images/5.jpg');
test6 = imread('test_images/6.jpg');
test7 = imread('test_images/7.jpg');
test8 = imread('test_images/8.jpg');

%orange threshold = 1.3*10^-7
runimage(probMatrix(test1, covRGB, meanRGB))
runimage(probMatrix(test2, covRGB, meanRGB))
runimage(probMatrix(test3, covRGB, meanRGB))
runimage(probMatrix(test4, covRGB, meanRGB))
runimage(probMatrix(test5, covRGB, meanRGB))
runimage(probMatrix(test6, covRGB, meanRGB))
runimage(probMatrix(test7, covRGB, meanRGB))
runimage(probMatrix(test8, covRGB, meanRGB))

function RGBMatrix = toRGBmatrix(RGBMatrix, Image, Roipoly)
Roipoly = uint8(Roipoly);
R1 = Roipoly.*Image(:,:,1); %Red
G1 = Roipoly.*Image(:,:,2); %Green
B1 = Roipoly.*Image(:,:,3); %Blue

R1 = R1(R1 > 0);
R1 = R1';
G1 = G1(G1 > 0);
G1 = G1';
B1 = B1(B1 > 0);
B1 = B1';

newRGB = [R1; G1; B1];
RGBMatrix = [RGBMatrix newRGB];
end

function probM = probMatrix(testImage, covRGB, meanRGB)
sizeM = size(testImage);
sizeRow = sizeM(1,1);
sizeColumn = sizeM(1,2);
probM = zeros(sizeRow,sizeColumn);
testImage = double(testImage);

for i = 1:sizeRow
    for j= 1:sizeColumn
        
    temp = [testImage(i,j,1)-meanRGB(1,1) ; testImage(i,j,2)-meanRGB(2,1) ; testImage(i,j,3)-meanRGB(3,1)];
    probM(i,j) = (1/(sqrt((2*pi).^3*det(covRGB))))*exp(((-1)/2).*temp'*inv(covRGB)*temp);
    
    end
end

%p(x|orange).p(orange)
probM=probM*0.5;
end

function runimage(matrix)
figure
mycolors = [0 0 0.5; 0 1 0; 1 0 0; 1 1 0 ; 1 0.5 0];
image(matrix,'CDataMapping','scaled')
colormap(mycolors)
colorbar
caxis([0.5*10.^(-7) 1.5*10.^(-7)])
end

