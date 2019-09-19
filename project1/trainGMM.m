function [p,meank,covk]=trainGMM(k)
% get training images and their roipoly
load training_data.mat

% get data input from RGB of each orange pixel (of each image)

RGBM = toRGBmatrix([], I1, t1);
RGBM = toRGBmatrix(RGBM, I2, t2);
RGBM = toRGBmatrix(RGBM, I3, t3);
%RGBM = toRGBmatrix(RGBM, I4, t4);
RGBM = toRGBmatrix(RGBM, I5, t5);
RGBM = toRGBmatrix(RGBM, I6, t6);
RGBM = toRGBmatrix(RGBM, I7, t7);
RGBM = toRGBmatrix(RGBM, I8, t8);
%RGBM = toRGBmatrix(RGBM, I9, t9);
RGBM = toRGBmatrix(RGBM, I10, t10);
RGBM = toRGBmatrix(RGBM, I11, t11);
RGBM = toRGBmatrix(RGBM, I12, t12);
RGBM = toRGBmatrix(RGBM, I13, t13);
RGBM = toRGBmatrix(RGBM, I14, t14);
%RGBM = toRGBmatrix(RGBM, I15, t15);
RGBM = toRGBmatrix(RGBM, I16, t16);
RGBM = toRGBmatrix(RGBM, I17, t17);
RGBM = toRGBmatrix(RGBM, I18, t18);
RGBM = toRGBmatrix(RGBM, I19, t19);
RGBM = toRGBmatrix(RGBM, I19, t19);
RGBM = toRGBmatrix(RGBM, I20, t20);
RGBM = toRGBmatrix(RGBM, I21, t21);
RGBM = toRGBmatrix(RGBM, I22, t22);
%RGBM = toRGBmatrix(RGBM, I23, t23);

% expectation

% initialization
p = 1/k*ones(1,k);
%p = rand(1,k);
meank = rand(3,k)*255;

covk = zeros(3,3,k);
for i = 1:k
    covk(:,:,i) = 0.5*eye(3)*10^4;
end

for j = 1:k
maxi = 20;
i=0;
mean_temp=zeros(3,1);
while i <= maxi && abs(sum(meank(:,j) - mean_temp,'all')) > 1
    
    if i >= 1
        meank(:,j) = mean_temp;
        covk(:,:,j) = cov_temp;
        p(:,j) = p_temp;
    end

%cluster weight -> we care only one cluster (orange)
cw = clusterweight(RGBM, p, meank, covk, j,k);


%maximization
mean_temp = newmean(RGBM,cw);
cov_temp = newcov(RGBM,cw, mean_temp);
p_temp = newp(RGBM,cw);

i=i+1;

end
end
end

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


function cw = clusterweight(data,p,meank,covk,currentk,maxk)
data = double(data);
sizeM = size(data);
sizeColumn = sizeM(1,2);
cw = zeros(1,sizeColumn);

for i = 1:sizeColumn
    sumscale = 0;
    for j = 1:maxk
    scale = scaledg(data,p, meank, covk,i,j);
    sumscale = sumscale + scale;
    end
    cw(1,i) = scaledg(data,p, meank, covk,i,currentk)/sumscale;
end

end

function sg = scaledg(data,p, meank, covk, dataindex,k)
data = double(data);
temp = [data(1,dataindex)-meank(1,k) ; data(2,dataindex)-meank(2,k) ; data(3,dataindex)-meank(3,k)];
sg = p(1,k)*(1/(sqrt((2*pi).^3*det(covk(:,:,k)))))*exp(((-1)/2).*temp'*inv(covk(:,:,k))*temp);
end

function mean = newmean(data, cw)
data = double(data);
dataR = data(1,:);
dataG = data(2,:);
dataB = data(3,:);
meanR = sum(dataR*cw','all')/sum(cw,'all');
meanG = sum(dataG*cw','all')/sum(cw,'all');
meanB = sum(dataB*cw','all')/sum(cw,'all');
%sum(data'.*cw',2)
%meantest = sum(cw'.*data',2)/sum(cw,'all')
mean = [meanR; meanG; meanB];
end

function cov = newcov(data, cw, mean)
data = double(data);
sizeM = size(data);
sizeColumn = sizeM(1,2);
dataR = data(1,:);
dataG = data(2,:);
dataB = data(3,:);
meanR = mean(1,1)*ones(1,sizeColumn);
meanG = mean(2,1)*ones(1,sizeColumn);
meanB = mean(3,1)*ones(1,sizeColumn);
R = dataR - meanR;
G = dataG - meanG;
B = dataB - meanB;

RR=0;
RG=0;
RB=0;
GG=0;
GB=0;
BB=0;

for i= 1:sizeColumn
    RR = RR+cw(1,i)*R(1,i)*R(1,i);
    RG = RG+cw(1,i)*R(1,i)*G(1,i);
    RB = RB+cw(1,i)*R(1,i)*B(1,i);
    GG = GG+cw(1,i)*G(1,i)*G(1,i);
    GB = GB+cw(1,i)*G(1,i)*B(1,i);
    BB = BB+cw(1,i)*B(1,i)*B(1,i);
end

covRR = RR/sum(cw,'all');
covRG = RG/sum(cw,'all');
covRB = RB/sum(cw,'all');
covGG = GG/sum(cw,'all');
covGB = GB/sum(cw,'all');
covBB = BB/sum(cw,'all');

cov = [covRR covRG covRB; covRG covGG covGB; covRB covGB covBB];


end

function p = newp(data,cw)
data = double(data);
sizeM = size(data);
sizeColumn = sizeM(1,2);
p = sum(cw,'all')/sizeColumn;
end



