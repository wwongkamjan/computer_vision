function probM = testGMM(test_image,p,meank,covk,k)

sizeM = size(test_image);
sizeRow = sizeM(1,1);
sizeColumn = sizeM(1,2);
probM = zeros(sizeRow,sizeColumn);
for i = 1:k
    probM = probM + probMatrix(test_image, covk(:,:,i), meank(:,i), p(:,i));
end
end

function probM = probMatrix(testImage, covRGB, meanRGB, p1)
sizeM = size(testImage);
sizeRow = sizeM(1,1);
sizeColumn = sizeM(1,2);
probM = zeros(sizeRow,sizeColumn);
testImage = double(testImage);

for i = 1:sizeRow
    for j= 1:sizeColumn
    temp = [testImage(i,j,1)-meanRGB(1,1) ; testImage(i,j,2)-meanRGB(2,1) ; testImage(i,j,3)-meanRGB(3,1)];
    probM(i,j) = p1*(1/(sqrt((2*pi).^3*det(covRGB))))*exp(((-1)/2).*temp'*inv(covRGB)*temp);
    
    end
end
end
