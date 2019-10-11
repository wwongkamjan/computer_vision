function [pano] = MyPanorama()
clear all;

path = fileparts(which('MyPanorama.m'));
folder = regexprep(path, 'Code', '');
folder = strcat(folder, 'Images\Input\');
S = dir(fullfile(folder,'*.jpg'));
scale = 1;
if size(S,1) > 3 
    scale = 0.4;
end

imds = imageDatastore(folder,'FileExtensions',{'.jpg','.jpeg'});

num_images = numel(imds.Files);
train_raw = cell(num_images);

for i = 1:num_images
    train_raw{i} = readimage(imds,i);
end

train = cell(num_images);


for i = 1:num_images
    train{i} = imresize(train_raw{i}, scale);
end


train_gray = cell(num_images);
corners = cell(num_images);
imageSize = zeros(num_images,2);

for i = 1:num_images
    train_gray{i} = rgb2gray(train{i});
    imageSize(i,:) = size(train_gray{i});
    corners{i} = cornermetric(train_gray{i});
end

Nbest = 200;
ratio = 0.5;
n_max = 200;

x_best = cell(num_images);
y_best = cell(num_images);
cbest = cell(num_images);

for i = 1:num_images
    [x_best{i}, y_best{i}, cbest{i}] = ANMS(corners{i}, Nbest);
end

for i = 1:num_images
    feature_vector{i} = featureDescriptor(train_gray{i}, x_best{i}, y_best{i});
end

tforms(num_images) = projective2d(eye(3));

for i = 2:num_images
  [matched_points{i - 1}, matched_points{i}] = featureMatching(cbest{i - 1}, cbest{i}, feature_vector{i - 1}, feature_vector{i}, ratio); 
  [inliers{i - 1}, inliers{i}, H] = RANSAC(matched_points{i - 1}, matched_points{i}, n_max);
  %Image after RANSAC
  figure;
  hold on;
  showMatchedFeatures(train{i -1}, train{i}, inliers{i-1}, inliers{i}, 'montage');
  hold off;
  
  %tforms(i) = projective2d(inv(H)');
  tforms(i) = estimateGeometricTransform(inliers{i}, inliers{i - 1},'projective', 'Confidence', 80, 'MaxNumTrials', 2000);
  tforms(i).T = tforms(i).T*tforms(i - 1).T;
end


% Compute the output limits  for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);
Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

%maxImageSize = max(size(train1_1), size(train1_2))
maxImageSize = max(imageSize);

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
train_init = cell(num_images);
for i = 1:num_images
    train_init{i} = imresize(train{i}, 0.5);
end
panorama = zeros([height width 3], 'like', train_init{num_images});

blender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource','Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], double(xLimits), double(yLimits));

for i = 1:num_images
    
    I = readimage(imds, i);   
    I = imresize(I, scale);
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
imshow(panorama)

pano = panorama;

end
